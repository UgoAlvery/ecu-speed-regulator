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
board_pins.h        → brochage ESP32 #1, source unique des numéros de GPIO
ecu_state.h / .c    → état partagé centralisé (accès sous mutex)
protocol.h / .c     → lib pure sans état global (encode, decode, CRC)
pid.h / .c          → lib pure sans état global (struct pid_t passée en paramètre)
failsafe.h / .c     → task_failsafe + ISR GPIO + LED d'état
failsafe_line.h/.c  → machine à états de la ligne NC (latch, anti-rebond) — lib pure
task_rx.h / .c      → UART init + parser état machine + compteurs RX
task_control.h / .c → dispatch trames + boucle PID 100 ms + unique émetteur d'OUTPUT
task_tx.h / .c      → émission UART (seul écrivain UART, seul encodeur)
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

| Tâche            | Priorité | Périodicité    | Stack (octets) | Mécanisme de réveil                         |
|------------------|----------|----------------|----------------|---------------------------------------------|
| `task_failsafe`  | 5        | < 5 ms réponse | 2048           | sémaphore binaire (ISR GPIO) + polling 100 ms |
| `task_rx`        | 4        | event-driven   | 4096           | `uart_read_bytes` bloquant 50 ms            |
| `task_control`   | 4        | 100 ms strict  | 4096           | `vTaskDelayUntil`                           |
| `task_tx`        | 3        | event-driven   | 4096           | `xQueueReceive` borné 500 ms (TWDT)         |
| `task_telemetry` | 2        | 1 s            | 2048           | `vTaskDelayUntil`                           |

> Sous ESP-IDF, la profondeur de pile de `xTaskCreate` est en **octets**
> (contrairement au FreeRTOS vanilla) : macros `STACK_*_BYTES` de `main.c`,
> valeurs ci-dessus = tailles réellement allouées. Marges non mesurées — cf.
> `rta_fmea.md` R-8.

> `task_rx` et `task_control` ont la même priorité (4). En régime nominal
> (task_rx bloquée sur UART l'essentiel du temps) elles ne se préemptent
> quasiment pas. **Sous flood UART soutenu, la RTA formelle (`rta_fmea.md`
> §5) montre que ce n'est pas garanti** : si le coût de traitement d'un octet
> `C_rx` atteint 86,8 µs (temps d'un octet à 115 200 bauds), `task_rx` reste
> continûment *ready* ; round-robin (`configUSE_TIME_SLICING=1`, tranche
> 10 ms, « best effort » sous IDF) et passage de mutex à priorité égale
> donnent alors un pire cas de gigue de 10–20 ms, hors budget ±5 ms d'EF-01.
> Si `C_rx < 86,8 µs` (attendu), l'interférence se limite au vidage du
> backlog RX. Tranché par la mesure de `C_rx` (axe 1 action 2) — risque
> ouvert, pas une garantie. Correctif structurel proposé : `rta_fmea.md` R-1.

---

## Primitives FreeRTOS utilisées

| Primitive             | Nom               | Partagé entre                                           |
|-----------------------|-------------------|---------------------------------------------------------|
| Queue (ecu_frame_t)   | `queue_frames`    | task_rx → task_control (SETPOINT, SPEED ; 16 places)    |
| Queue (ecu_mode_t)    | `queue_mode`      | task_rx → task_control (MODE_SET ; 1 place, `xQueueOverwrite`) |
| Queue (tx_message_t)  | `queue_tx`        | task_control, task_telemetry, task_failsafe → task_tx   |
| Mutex (héritage prio) | `mutex_ecu_state` | task_rx, task_control, task_failsafe ↔ ecu_state        |
| Sémaphore binaire     | `sem_failsafe`    | ISR GPIO → task_failsafe                                |

> `configUSE_MUTEXES = 1` et `configUSE_RECURSIVE_MUTEXES = 1` requis dans
> sdkconfig pour l'héritage de priorité sur les mutex.

> **RTA formelle + FMEA (axe 1, actions 3–4)** : analyse structurelle
> terminée — modélisation sporadique des 5 tâches, `B_ctrl = 0` (aucune tâche
> moins prioritaire ne partage le mutex), `B_fs ≤ 4·C_cs` en double cœur,
> cas `task_rx`/`task_control` à priorité égale, chemin failsafe, table FMEA
> et recommandations R-1…R-8 — voir `rta_fmea.md` à la racine du repo.
> Quantification numérique des WCET encore ⏳ en attente de la campagne de
> mesure (axe 1, action 2).

---

## Module protocol.h / .c — interface figée

```c
#define PROTOCOL_START_BYTE        0xAA
#define PROTOCOL_MAX_PAYLOAD_SIZE  128
#define PROTOCOL_NONCE_SIZE        2
#define PROTOCOL_MAX_FRAME_SIZE    (1 + 2 + 1 + PROTOCOL_NONCE_SIZE + PROTOCOL_MAX_PAYLOAD_SIZE + 2)

#define MSG_SETPOINT  0x01
#define MSG_SPEED     0x02
#define MSG_MODE_SET  0x05
#define MSG_OUTPUT    0x80
#define MSG_STATS     0x83
#define MSG_ALARM     0x85
#define MSG_DBG       0xFF

typedef struct {
    uint8_t  type;
    uint16_t nonce;
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    uint16_t payload_len;
} ecu_frame_t;

typedef struct {
    uint16_t last_nonce;
} protocol_nonce_ctx_t;

uint16_t protocol_compute_crc(const uint8_t *data, size_t len);
size_t  protocol_encode(uint8_t *dst, uint8_t type, uint16_t nonce,
                        const uint8_t *payload, uint16_t payload_len);
bool    protocol_decode(const uint8_t *frame, size_t frame_len,
                        ecu_frame_t *out);
void    protocol_nonce_ctx_init(protocol_nonce_ctx_t *ctx);
bool    protocol_nonce_check_and_update(protocol_nonce_ctx_t *ctx, uint16_t nonce);

#define PROTOCOL_MODE_MAX  2u
bool    protocol_parse_f32(const ecu_frame_t *frame, float *out);    // len == 4, isfinite
bool    protocol_parse_mode(const ecu_frame_t *frame, uint8_t *out); // len == 1, <= MODE_MAX
```

**Règles** :
- `protocol.h` n'inclut pas `<driver/uart.h>` — lib pure
- `protocol_decode` travaille sur une trame déjà complète en mémoire
- CRC = CRC-16/CCITT (polynôme 0x1021, init 0x0000, variante XMODEM), calculé
  sur tous les octets sauf START ; transmis en little-endian sur 2 octets
- LEN = taille(TYPE + NONCE + PAYLOAD), little-endian sur 2 octets
- NONCE = compteur 16 bits anti-rejeu, cf. section « Authentification légère » ci-dessous
- Retour 0 / false en cas d'erreur, jamais d'assert en prod
- Validation applicative des payloads (`protocol_parse_*`) dans la lib pure, pas dans
  les tâches : longueur **exacte** du type (un payload trop long est aussi rejeté) et
  domaine de valeur. `*out` n'est jamais modifié en cas de rejet. Testé en host

---

## Module task_rx.h / .c — interface figée

```c
#define UART_NUM        UART_NUM_2   // protocole ; UART0 = console seule
#define UART_BUF_SIZE   1024
#define UART_BAUD_RATE  115200
// Broches : UART_PROTO_TX_PIN (17) / UART_PROTO_RX_PIN (16), dans board_pins.h
#define RX_QUEUE_SIZE   16
#define PARSER_TIMEOUT_MS 50   // reset parser si trame incomplète après 50ms

void task_rx_init(QueueHandle_t rx_queue, QueueHandle_t mode_queue);
void task_rx(void *pvParameters);

// Accesseurs compteurs (lus par task_telemetry via ecu_state)
uint32_t task_rx_get_count_valid(void);
uint32_t task_rx_get_count_crc_err(void);
uint32_t task_rx_get_count_dropped(void);
uint32_t task_rx_get_count_replay(void);
uint32_t task_rx_get_count_mode_invalid(void);      // additionné à rx_invalid
uint32_t task_rx_get_count_mode_overwritten(void);
```

**Règles** :
- `uart_init()` est statique dans `task_rx.c`, appelée par `task_rx_init` ; installe le
  driver UART2 et route ses broches (`uart_set_pin`). `task_tx` écrit sur ce même port
  (`UART_NUM`) — il reste le seul écrivain
- **Aucun `ESP_LOG*` sur le chemin de réception ni dans le cycle de `task_control`** :
  tout rejet est compté (STATS), jamais loggué. Un log par trame est déclenchable par
  l'émetteur et coûte jusqu'à ~87 µs par caractère dans le cycle (`rta_fmea.md` §4.3)
- Parser = machine à états dans `parser_feed_byte()` — fonction séparée, testable
- `xQueueSend` avec timeout 0 (non bloquant) — si queue pleine → dropped++
- Compteurs `volatile`, pas de mutex — un seul écrivain (task_rx), lecture
  périodique par task_telemetry (cohérence approximative acceptable)
- `task_rx` ne crée pas la queue — elle lui est passée par `task_rx_init`
- Contexte `protocol_nonce_ctx_t` détenu localement (statique) dans `task_rx.c`, comme
  `frame_parser_t` — initialisé dans `task_rx_init`. Vérification du nonce **après**
  `protocol_decode()` (CRC/format déjà validés), **avant** l'envoi dans `queue_frames` :
  trame rejetée → `s_count_replay++`, pas d'enqueue, pas de blocage des trames suivantes
- **MODE_SET → `queue_mode`, jamais `queue_frames`** (`rta_fmea.md` R-6) : validé par
  `protocol_parse_mode` (invalide → `s_count_mode_invalid++`), puis
  `xQueueOverwrite(queue_mode)` — n'échoue jamais, le dernier MODE_SET gagne. Si la queue
  contenait déjà une demande non lue (`uxQueueMessagesWaiting == 1`) →
  `s_count_mode_overwritten++` (indicatif : task_control peut lire entre le test et
  l'écriture, sous-comptage d'au plus 1)
- task_rx **ne prend pas de nouvel accès** à `mutex_ecu_state` pour MODE_SET : son seul
  accès reste `set_last_rx_tick`. Le verrou failsafe (refus d'AUTO) est appliqué par
  task_control via `ecu_state_try_set_mode`

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
    bool       failsafe_locked; // ligne failsafe GPIO ouverte : AUTO interdit
} ecu_state_t;

void       ecu_state_init(void);
// Setters simples (prennent le mutex en interne)
void       ecu_state_set_speed(float speed);
void       ecu_state_set_setpoint(float setpoint);
void       ecu_state_set_last_rx_tick(TickType_t tick);
void       ecu_state_set_failsafe_locked(bool locked);
// Lecture-modification-écriture atomiques (une seule prise du mutex)
bool       ecu_state_try_set_mode(ecu_mode_t mode);      // refuse AUTO si failsafe_locked
float      ecu_state_commit_output(float requested);     // AUTO ? requested : 0, retourne la valeur
void       ecu_state_enter_failsafe(bool lock);          // [verrou] + output=0 + OFF
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
- **Toute lecture-modification-écriture passe par une fonction dédiée du module
  ecu_state, exécutée sous une seule prise du mutex.** Un appelant ne compose jamais
  un `get` suivi d'un `set` pour décider d'une écriture : une autre tâche pourrait
  modifier l'état entre les deux appels. Les getters et setters simples restent
  réservés aux accès indépendants (une seule lecture, ou une écriture inconditionnelle)
- Fonctions de ce type : `ecu_state_try_set_mode` (refuse AUTO si `failsafe_locked`),
  `ecu_state_commit_output` (relit le mode et fixe la commande), `ecu_state_enter_failsafe`
  (verrou + commande 0 + OFF). Il n'existe **pas** de `set_mode` ni de `set_output`
  inconditionnels : le mode ne s'écrit que par ces fonctions, ce qui rend le verrou
  failsafe impossible à contourner. `output` n'a qu'un écrivain en régime normal :
  task_control, via `commit_output`
  Chacune garde une section critique courte et bornée : quelques champs copiés ou
  comparés, aucun appel bloquant, aucun log

---

## Module pid.h / .c — ✅ implémenté

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

**Entrées non finies** : si `setpoint - measure` n'est pas fini, ou si la somme
P+I+D vaut NaN (débordements opposés), `pid_compute` retourne `out_min` **sans
modifier l'état** (`integral`, `prev_err`). Sans ce garde, un seul NaN reçu sur
le bus (CRC valide) figeait l'intégrale à NaN et toutes les sorties suivantes
jusqu'au prochain `pid_reset()` (cf. `rta_fmea.md` F-05). Défense en profondeur :
`task_control` rejette déjà les SETPOINT/SPEED non finis au dispatch (dernière
valeur valide conservée).

---

## Module failsafe.h / .c — ✅ implémenté

```c
// FAILSAFE_GPIO_PIN (4) et FAILSAFE_LED_PIN (25) : board_pins.h
#define FAILSAFE_TIMEOUT_MS    2000
#define FAILSAFE_RESPONSE_MS   5

void failsafe_init(QueueHandle_t tx_queue);
void task_failsafe(void *pvParameters);
```

**Règles** :
- Ligne en **contact normalement fermé** : jumper GPIO4 → GND, pull-up interne.
  Repos = 0, ouverture (jumper retiré ou fil coupé) = 1. Interruption
  `GPIO_INTR_POSEDGE` (ouverture)
- ISR GPIO → `xSemaphoreGiveFromISR(sem_failsafe)`
- task_failsafe bloquée sur `xSemaphoreTake(sem_failsafe, <temps restant jusqu'au
  prochain poll>)` : polling à période fixe de 100 ms même sous rafale de fronts
- À chaque poll : niveau de GPIO4 relu (ligne déjà ouverte, front manqué,
  déverrouillage) + contrôle du last_rx_tick
- Au boot (début de task_failsafe) : niveau relu ; ligne ouverte → failsafe immédiat,
  ALARM « GPIO line open at boot »
- Latch et anti-rebond dans `failsafe_line.c` (lib pure, testée en host) :
  `TRIP` au premier front ou niveau ouvert, puis `NONE` tant que verrouillé (une seule
  ALARM par entrée en failsafe) ; `UNLOCK` après 2 polls consécutifs ligne fermée
- Au déclenchement GPIO : `ecu_state_enter_failsafe(true)` (verrou + output=0 + OFF en
  une prise du mutex), puis ALARM dans queue_tx, LED GPIO25 allumée. **task_failsafe
  n'émet pas d'OUTPUT** : task_control émet OUTPUT(0) à son prochain cycle (≤ 100 ms),
  puis à chaque cycle
- Silence UART : même séquence sans verrou ; fin du failsafe UART quand le mode
  quitte OFF (reprise EF-06)
- LED GPIO25 = failsafe GPIO actif **ou** failsafe UART actif ; seul task_failsafe
  l'écrit
- Reprise : MODE_SET appliqué via `ecu_state_try_set_mode` — AUTO refusé tant que
  `failsafe_locked`, compteur `mode_refused` (STATS)
- EF-05 (< 5 ms) tenue pour l'état interne (OFF, output=0) ; ALARM et OUTPUT(0) sur la
  ligne sont bornés séparément — cf. `rta_fmea.md` §6.1

---

## Authentification légère — nonce anti-rejeu — ✅ implémenté

Choix : nonce/compteur anti-rejeu plutôt que HMAC tronqué (les deux options étaient
ouvertes par la roadmap). Justification : coût quasi nul (2 octets/trame, pas de calcul
crypto), suffisant contre la menace réaliste sur ce bus (rejeu d'une trame de commande
capturée), et cohérent avec le principe « authentification **légère** » — un HMAC
apporterait l'intégrité de contenu (déjà couverte par le CRC16 contre les erreurs, pas
contre la falsification volontaire) mais serait disproportionné pour un TP durci en
projet perso plutôt qu'un produit réellement exposé à un adversaire actif.

**API (`protocol.c`, lib pure — cf. interface ci-dessus)** :
- `protocol_nonce_ctx_init(ctx)` : remet `last_nonce = 0`
- `protocol_nonce_check_and_update(ctx, nonce)` : accepte ssi `nonce` est postérieur à
  `ctx->last_nonce` au sens de l'arithmétique modulaire 16 bits signée
  (`(int16_t)(nonce - last_nonce) > 0`) — tolère le wraparound normal (65536 trames à
  10 Hz ≈ 109 min) comme les numéros de séquence TCP. Met à jour `ctx` uniquement en cas
  d'acceptation ; ne modifie rien et retourne `false` sinon (rejeu, duplication, retard
  hors ordre)
- Un contexte par flux à protéger, passé par pointeur (même pattern que `pid_t`) — pas
  d'état global dans `protocol.c`

**Intégration firmware** :
- Émission (`task_tx.c`) : compteur statique `s_tx_nonce` (task_tx est le seul écrivain
  UART — même exception structurelle que les compteurs `volatile` de `task_rx`), pré-
  incrémenté avant chaque `protocol_encode()` → séquence 1, 2, 3, ... partagée par tous
  les types de message sortants (OUTPUT, STATS, ALARM, DBG)
- Réception (`task_rx.c`) : `protocol_nonce_ctx_t` statique, initialisé dans
  `task_rx_init`. Vérifié après `protocol_decode()` réussi, avant l'enqueue dans
  `queue_frames` — trame rejetée → `s_count_replay++`, comportement silencieux identique
  aux autres rejets protocolaires
- Télémétrie : `rx_replay` ajouté à STATS (`TELEMETRY_STATS_COUNT` 5→6 à cette étape,
  7 après l'ajout de `tx_dropped` — cf. section « Télémétrie STATS » pour l'ordre final
  du payload)

**Limitation documentée** : pas d'état persistant entre redémarrages — après un reboot de
l'ECU, `last_nonce` repart à 0, ce qui rouvre une fenêtre de rejeu jusqu'à la première
trame légitime reçue (qui la referme aussitôt, puisque son nonce devient le nouveau
plancher). Acceptable pour une authentification *légère* sur un bus UART point-à-point ;
documenté explicitement plutôt que passé sous silence.

`test/ecu_tester.py` : nonce sortant auto-incrémenté par `build_frame`/`send_frame`,
vérification symétrique des nonces entrants (`is_nonce_newer`, même règle modulaire),
et action de stress dédiée (rejeu de la dernière trame SPEED acceptée, cf. `run_stress_test`).

---

## Télémétrie STATS (0x83) — ordre du payload

`TELEMETRY_STATS_COUNT = 10` × `uint32_t` little-endian = 40 octets, émis toutes les 1 s :

| # | Champ          | Écrivain unique   | Signification                                           |
|---|----------------|-------------------|---------------------------------------------------------|
| 0 | `rx_valid`     | task_rx           | Trames décodées (CRC OK) et nonce accepté               |
| 1 | `rx_crc_error` | task_rx           | Trames rejetées par `protocol_decode`                   |
| 2 | `rx_dropped`   | task_rx           | LEN incohérent, trame partielle expirée, `queue_frames` pleine |
| 3 | `rx_replay`    | task_rx           | Nonce rejeté (rejeu / duplication / hors ordre)         |
| 4 | `tx_output`    | task_tx           | Trames OUTPUT effectivement écrites sur l'UART (10/s dans tous les modes depuis D7) |
| 5 | `tx_dropped`   | task_control      | OUTPUT non émis faute de place dans `queue_tx`          |
| 6 | `rx_invalid`   | task_control + task_rx (somme de deux compteurs) | Trame saine (CRC + nonce OK) rejetée au dispatch : longueur de payload ≠ taille du type, valeur non finie / hors domaine, type non attendu en RX |
| 7 | `mode_refused` | task_control      | MODE_SET(AUTO) refusé : ligne failsafe GPIO ouverte (verrou) |
| 8 | `mode_cmd_overwritten` | task_rx   | MODE_SET écrasé dans `queue_mode` avant lecture par task_control |
| 9 | `uptime_s`     | task_telemetry    | Secondes depuis le démarrage de task_telemetry          |

`tx_dropped` a été ajouté lors de la RTA : `enqueue_output()` bloquait jusqu'à 10 ms
sur `queue_tx` pleine, terme de blocage hors budget ±5 ms — désormais timeout 0 +
compteur (cf. `rta_fmea.md` §4.3). `rx_invalid` remplace les `ESP_LOGW` du dispatch
(D1). Côté PC : tuple `STATS_FIELDS` dans `test/ecu_tester.py`, à tenir dans le même
ordre.

---

## Watchdog matériel (TWDT) — ✅ implémenté

Complète le failsafe applicatif, qui dépend lui-même du bon fonctionnement des tâches.

- `sdkconfig` : `CONFIG_ESP_TASK_WDT_PANIC=y`, `CONFIG_ESP_TASK_WDT_TIMEOUT_S=2`
  (tâches IDLE des deux cœurs toujours surveillées)
- Les 5 tâches s'abonnent (`esp_task_wdt_add(NULL)`) au démarrage et appellent
  `esp_task_wdt_reset()` à chaque itération de leur boucle
- `task_tx` : `xQueueReceive` borné à `TASK_TX_IDLE_WAKE_MS` (500 ms) au lieu de
  `portMAX_DELAY`, pour nourrir le TWDT même sans trafic sortant
- Expiration → panic → reboot → `ecu_state_init()` → mode OFF, output 0 (état sûr)
- ⏳ Validation sur cible à faire (boucle infinie injectée volontairement)

Détail des marges par tâche : `rta_fmea.md` §8.

---

## Tests hébergés (host) — ✅ implémenté

`protocol.c`, `pid.c` et `failsafe_line.c` sont testés nativement (GCC, sans ESP-IDF/FreeRTOS), profitant de
leur statut de libs pures.

```
test/unit/test_runner.h    → harness minimal, sans dépendance externe
test/unit/test_protocol.c  → 113 assertions (CRC, encode, decode, roundtrip, détection
                              d'erreur, nonce anti-rejeu, validation des payloads)
test/unit/test_pid.c       → 41 assertions (init, reset, P/I/D, saturation, anti-windup,
                              entrées NaN/Inf)
test/unit/test_failsafe_line.c → 34 assertions (latch, debounce, ligne ouverte au boot,
                              déverrouillage anti-rebond)
test/unit/Makefile         → build natif, -fsanitize=address,undefined
```

`make -C test/unit run` : build + exécution des trois suites. 188 assertions, 0 échec, aucun
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

**Résultat** (session de validation, 60 s, format initial sans NONCE) : ~29,3 millions
d'exécutions, 0 crash, 0 timeout, 0 trigger ASan/UBSan, `findings/` vide. Couverture
stabilisée à 31 arêtes / 67 features.

**Re-validation** (30 s, après ajout du champ NONCE — cf. section Authentification
légère) : ~23,7 millions d'exécutions, 0 crash, 0 timeout, 0 trigger ASan/UBSan,
`findings/` vide, couverture stabilisée à 36 arêtes / 69 features (légère hausse
cohérente avec le chemin de décodage un peu plus long). Confirme la garantie du
protocole : toute trame malformée (CRC invalide, LEN incohérent, START invalide,
troncature) est rejetée silencieusement (`false`), sans plantage ni lecture hors bornes.
`protocol_decode` ne vérifie pas le nonce (ce n'est pas son rôle — cf. séparation
décrite dans la section Authentification légère), le fuzzing de cette fonction ne
couvre donc pas `protocol_nonce_check_and_update`, testée unitairement à la place.

---

## CI — GitHub Actions — ✅ implémenté

`.github/workflows/ci.yml`, déclenché sur push/PR vers `main`, 3 jobs indépendants :

| Job              | Runner       | Contenu                                                        |
|------------------|--------------|-----------------------------------------------------------------|
| `firmware-build` | ubuntu-latest| `espressif/esp-idf-ci-action@v1` (image Docker ESP-IDF v5.3.5), `idf.py build` cible esp32 |
| `hosted-tests`   | ubuntu-latest| `make -C test/unit run` — 188 assertions, ASan/UBSan                 |
| `fuzz-smoke`     | ubuntu-latest| build + corpus de départ + 30 s de fuzzing libFuzzer sur `protocol_decode`, artefacts uploadés si crash |

`fuzz-smoke` n'est pas une campagne de fuzzing complète (voir section Fuzzing ci-dessus
pour la session de validation longue) : c'est une régression rapide qui détecte tout
crash/UB déjà connu ré-introduit par un changement, sans allonger significativement le
temps de CI.

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
  (exception : compteurs `volatile` à écrivain unique — task_rx, task_tx,
  task_control/`tx_dropped`)
- Pas d'architecture monolithique (tout dans main ou dans une seule tâche)
- Pas de `configUSE_MUTEXES = 0` — l'héritage de priorité doit être activé