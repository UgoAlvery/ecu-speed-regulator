# TECHNICAL_REFERENCE — ECU Régulateur de Vitesse

> Fichier de référence interne. À relire en début de chaque session de travail
> pour garantir la cohérence architecturale entre les conversations.

---

## Décisions architecturales actées

### Langage
- **C** (pas C++). ESP-IDF est une SDK C, FreeRTOS est pensé C.
- Pas de malloc dans les tâches temps réel — buffers statiques uniquement.

### Découpage en fichiers
```
main.c              → app_main : init + xTaskCreate + rien d'autre
ecu_state.h / .c    → état partagé centralisé (accès sous mutex)
protocol.h / .c     → lib pure sans état global (encode, decode, CRC)
pid.h / .c          → lib pure sans état global (struct pid_t passée en paramètre)
failsafe.h / .c     → task_failsafe + ISR GPIO
task_rx.h / .c      → UART init + parser état machine + compteurs RX
task_control.h / .c → dispatch trames + boucle PID 100 ms
task_tx.h / .c      → émission UART protégée par mutex
task_telemetry.h/.c → émission STATS 1 s
```

### Décision : pas de task_dispatcher
`task_dispatcher` a été volontairement supprimé. Le dispatch (switch sur frame.type
→ ecu_state) est intégré dans `task_control` en début de cycle, de manière
non bloquante (`xQueueReceive` avec timeout 0). Justification : un seul
producteur (task_rx), un seul consommateur, logique triviale — une tâche
dédiée serait du overhead sans bénéfice.

---

## Tâches FreeRTOS

| Tâche            | Priorité | Périodicité    | Stack (mots) | Mécanisme de réveil                         |
|------------------|----------|----------------|--------------|---------------------------------------------|
| `task_failsafe`  | 5        | < 5 ms réponse | 2048         | sémaphore binaire (ISR GPIO) + polling tick |
| `task_rx`        | 4        | event-driven   | 4096         | `uart_read_bytes` bloquant 50 ms            |
| `task_control`   | 4        | 100 ms strict  | 4096         | `vTaskDelayUntil`                           |
| `task_tx`        | 3        | event-driven   | 2048         | `xQueueReceive` bloquant                    |
| `task_telemetry` | 2        | 1 s            | 2048         | `vTaskDelayUntil`                           |

> `task_rx` et `task_control` ont la même priorité (4). C'est intentionnel :
> task_rx est event-driven (bloquée sur UART), task_control est périodique.
> Elles ne se préemptent pas mutuellement en pratique.

---

## Primitives FreeRTOS utilisées

| Primitive             | Nom               | Partagé entre                                           |
|-----------------------|-------------------|---------------------------------------------------------|
| Queue (ecu_frame_t)   | `queue_frames`    | task_rx → task_control                                  |
| Queue (tx_message_t)  | `queue_tx`        | task_control, task_telemetry, task_failsafe → task_tx   |
| Mutex (héritage prio) | `mutex_ecu_state` | task_control, task_failsafe, task_telemetry ↔ ecu_state |
| Sémaphore binaire     | `sem_failsafe`    | ISR GPIO → task_failsafe                                |

> `configUSE_MUTEXES = 1` et `configUSE_RECURSIVE_MUTEXES = 1` requis dans
> sdkconfig pour l'héritage de priorité sur les mutex.

---

## Module protocol.h / .c — interface figée

```c
#define PROTOCOL_START_BYTE        0xAA
#define PROTOCOL_MAX_PAYLOAD_SIZE  128
#define PROTOCOL_MAX_FRAME_SIZE    (1 + 2 + 1 + PROTOCOL_MAX_PAYLOAD_SIZE + 2)

#define MSG_SETPOINT  0x01
#define MSG_SPEED     0x02
#define MSG_MODE_SET  0x05
#define MSG_OUTPUT    0x80
#define MSG_STATS     0x83
#define MSG_ALARM     0x85
#define MSG_DBG       0xFF

typedef struct {
    uint8_t  type;
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    uint16_t payload_len;
} ecu_frame_t;

uint16_t protocol_compute_crc(const uint8_t *data, size_t len);
size_t  protocol_encode(uint8_t *dst, uint8_t type,
                        const uint8_t *payload, uint16_t payload_len);
bool    protocol_decode(const uint8_t *frame, size_t frame_len,
                        ecu_frame_t *out);
```

**Règles** :
- `protocol.h` n'inclut pas `<driver/uart.h>` — lib pure
- `protocol_decode` travaille sur une trame déjà complète en mémoire
- CRC = CRC-16/CCITT (polynôme 0x1021, init 0x0000, variante XMODEM), calculé
  sur tous les octets sauf START ; transmis en little-endian sur 2 octets
- LEN = taille(TYPE + PAYLOAD), little-endian sur 2 octets
- Retour 0 / false en cas d'erreur, jamais d'assert en prod

---

## Module task_rx.h / .c — interface figée

```c
#define UART_NUM        UART_NUM_0
#define UART_BUF_SIZE   1024
#define UART_BAUD_RATE  115200
#define TXD_PIN         17
#define RXD_PIN         16
#define RX_QUEUE_SIZE   16
#define PARSER_TIMEOUT_MS 50   // reset parser si trame incomplète après 50ms

void task_rx_init(QueueHandle_t rx_queue);
void task_rx(void *pvParameters);

// Accesseurs compteurs (lus par task_telemetry via ecu_state)
uint32_t task_rx_get_count_valid(void);
uint32_t task_rx_get_count_crc_err(void);
uint32_t task_rx_get_count_dropped(void);
```

**Règles** :
- `uart_init()` est statique dans `task_rx.c`, appelée par `task_rx_init`
- Parser = machine à états dans `parser_feed_byte()` — fonction séparée, testable
- `xQueueSend` avec timeout 0 (non bloquant) — si queue pleine → dropped++
- Compteurs `volatile`, pas de mutex — un seul écrivain (task_rx), lecture
  périodique par task_telemetry (cohérence approximative acceptable)
- `task_rx` ne crée pas la queue — elle lui est passée par `task_rx_init`

**Machine à états parser** :
```
WAIT_START → (0xAA) → READ_LEN → (2 octets) → READ_PAYLOAD → (LEN octets) → READ_CRC
    ▲                                                                              │
    └──────────────────── reset (CRC invalide / timeout / LEN incohérent) ────────┘
```

**Gestion du last_rx_tick** :
- À chaque trame valide : `ecu_state_set_last_rx_tick(xTaskGetTickCount())`
- `task_failsafe` lit `ecu_state_get_last_rx_tick()` pour le timeout 2 s
- Pas de dépendance directe task_rx ↔ task_failsafe — tout passe par ecu_state

---

## Module ecu_state.h / .c — ✅ implémenté

État centralisé de l'ECU. Tous les accès sont protégés par `mutex_ecu_state`.

```c
typedef enum {
    ECU_MODE_OFF    = 0,
    ECU_MODE_MANUAL = 1,
    ECU_MODE_AUTO   = 2,
} ecu_mode_t;

typedef struct {
    ecu_mode_t mode;
    float      setpoint;
    float      speed;
    float      output;
    TickType_t last_rx_tick;   // pour le failsafe timeout 2s
} ecu_state_t;

void       ecu_state_init(void);
// Setters (prennent le mutex en interne)
void       ecu_state_set_mode(ecu_mode_t mode);
void       ecu_state_set_speed(float speed);
void       ecu_state_set_setpoint(float setpoint);
void       ecu_state_set_output(float output);
void       ecu_state_set_last_rx_tick(TickType_t tick);
// Getters (prennent le mutex en interne)
ecu_mode_t ecu_state_get_mode(void);
float      ecu_state_get_speed(void);
float      ecu_state_get_setpoint(void);
float      ecu_state_get_output(void);
TickType_t ecu_state_get_last_rx_tick(void);
```

**Règles** :
- Mutex créé dans `ecu_state_init()`, appelée dans `app_main` avant les tâches
- Chaque setter/getter prend et relâche le mutex — pas d'exposition du mutex
- Pas de getter/setter combiné pour éviter les races entre read-modify-write

---

## Module pid.h / .c — à implémenter

Lib pure, pas d'état global. L'état PID est dans une struct passée par pointeur.

```c
typedef struct {
    float kp, ki, kd;
    float dt;
    float out_min, out_max;
    float integral;
    float prev_err;
} pid_t;

void  pid_init(pid_t *pid, float kp, float ki, float kd,
               float dt, float out_min, float out_max);
void  pid_reset(pid_t *pid);   // remet integral=0, prev_err=0
float pid_compute(pid_t *pid, float setpoint, float measure);
```

**Valeurs par défaut** : Kp=2.0, Ki=0.8, Kd=0.02, dt=0.1, min=0.0, max=255.0

**Anti-windup** : gel de l'intégrale si output saturé et erreur dans le même sens.

---

## Module failsafe.h / .c — à implémenter

```c
#define FAILSAFE_GPIO_PIN      GPIO_NUM_4   // à adapter
#define FAILSAFE_TIMEOUT_MS    2000
#define FAILSAFE_RESPONSE_MS   5

void failsafe_init(QueueHandle_t tx_queue);
void task_failsafe(void *pvParameters);
```

**Règles** :
- ISR GPIO → `xSemaphoreGiveFromISR(sem_failsafe)`
- task_failsafe bloquée sur `xSemaphoreTake(sem_failsafe, pdMS_TO_TICKS(100))`
- En parallèle, polling du last_rx_tick toutes les 100 ms
- Au déclenchement : ecu_state → OFF, output=0, puis ALARM dans queue_tx
- Reprise : task_control détecte MODE_SET → revalide le mode normalement

---

## Tests hébergés (host) — ✅ implémenté

`protocol.c` et `pid.c` sont testés nativement (GCC, sans ESP-IDF/FreeRTOS), profitant de
leur statut de libs pures.

```
test/unit/test_runner.h    → harness minimal, sans dépendance externe
test/unit/test_protocol.c  → 60 tests (CRC, encode, decode, roundtrip, détection d'erreur)
test/unit/test_pid.c       → 35 tests (init, reset, P/I/D, saturation, anti-windup)
test/unit/Makefile         → build natif, -fsanitize=address,undefined
```

`make -C test/unit run` : build + exécution des deux suites. 95 tests, 0 échec, aucun
warning, aucun trigger ASan/UBSan.

---

## Fuzzing — protocol_decode — ✅ implémenté

`protocol_decode` est la seule fonction du firmware qui consomme un flux non fiable
(UART) — candidate naturelle au fuzzing. Harness libFuzzer (clang, compilation host,
`protocol.c` étant une lib pure) :

```
test/fuzz/fuzz_protocol_decode.c → harness LLVMFuzzerTestOneInput(data, size)
test/fuzz/gen_seeds.c            → génère un corpus de départ via protocol_encode
                                    (trame vide par type connu, payload petit/max,
                                    CRC corrompu, LEN gonflé, START invalide)
test/fuzz/Makefile               → make fuzz [SECONDS=60] ; make repro CRASH=<file>
```

Build : `-fsanitize=fuzzer,address,undefined`. `corpus/` et `findings/` sont générés,
non versionnés (`.gitignore` dédié).

**Résultat** (session de validation, 60 s) : ~29,3 millions d'exécutions, 0 crash, 0
timeout, 0 trigger ASan/UBSan, `findings/` vide. Couverture stabilisée à 31 arêtes / 67
features — cohérent avec une fonction de validation à chemin court, sans boucle
dépendante de l'entrée non bornée. Confirme la garantie du protocole : toute trame
malformée (CRC invalide, LEN incohérent, START invalide, troncature) est rejetée
silencieusement (`false`), sans plantage ni lecture hors bornes.

---

## Ordre d'implémentation conseillé

1. ✅ `protocol.h / .c` — lib pure, testable sans FreeRTOS
2. ✅ `task_rx.h / .c` — UART + parser état machine
3. ✅ `ecu_state.h / .c` — état partagé + mutex
4. ✅ `pid.h / .c` — lib pure, testable sans FreeRTOS
5. ✅ `task_tx.h / .c` — émission protégée
6. ✅ `task_control.h / .c` — boucle 100 ms + dispatch + PID
7. ✅ `failsafe.h / .c` — ISR + timeout
8. ✅ `task_telemetry.h / .c` — STATS 1 s
9. ✅ `main.c` — assemblage final

---

## Contraintes de timing

| Contrainte                   | Valeur | Mécanisme de respect                    |
|------------------------------|--------|-----------------------------------------|
| Période régulation           | 100 ms | `vTaskDelayUntil` dans task_control     |
| Jitter max régulation        | ± 5 ms | Priorité 4, pas de blocage long         |
| Réponse failsafe GPIO        | < 5 ms | ISR + sémaphore, priorité 5             |
| Timeout silence failsafe     | 2 s    | Polling last_rx_tick dans task_failsafe |
| Reset parser trame partielle | 50 ms  | Timeout `uart_read_bytes`               |

---

## Ce qui n'est PAS dans ce projet

- Pas de malloc / free dans les tâches (buffers statiques ou sur stack)
- Pas de `vTaskDelay` dans task_control (utiliser `vTaskDelayUntil`)
- Pas de variable globale non protégée partagée entre plusieurs tâches
  (exception : compteurs `volatile` de task_rx, un seul écrivain)
- Pas d'architecture monolithique (tout dans main ou dans une seule tâche)
- Pas de `configUSE_MUTEXES = 0` — l'héritage de priorité doit être activé