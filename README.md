# ECU — Régulateur de Vitesse Temps Réel

[![CI](https://github.com/UgoAlvery/ecu-speed-regulator/actions/workflows/ci.yml/badge.svg)](https://github.com/UgoAlvery/ecu-speed-regulator/actions/workflows/ci.yml)

> TP EPITA 2026 · Systèmes Embarqués / Temps Réel
> Plateforme : **ESP32** · RTOS : **FreeRTOS** · Toolchain : **ESP-IDF v5.3**

Implémentation d'un ECU (*Electronic Control Unit*) de régulation de vitesse embarqué. L'ECU reçoit des consignes et mesures de vitesse via UART, calcule une commande moteur par algorithme PID, et garantit un comportement déterministe et sûr sous forte charge.

---

## Sommaire

- [Architecture](#architecture)
- [Protocole de communication](#protocole-de-communication)
- [Modes de fonctionnement](#modes-de-fonctionnement)
- [Exigences fonctionnelles](#exigences-fonctionnelles)
- [Exigences non fonctionnelles](#exigences-non-fonctionnelles)
- [Algorithme PID](#algorithme-pid)
- [Mécanisme Failsafe](#mécanisme-failsafe)
- [Analyse temps réel & sûreté](#analyse-temps-réel--sûreté)
- [Structure du projet](#structure-du-projet)
- [Build & Flash](#build--flash)
- [Tests](#tests)

---

## Architecture

Le projet repose sur une architecture **multi-tâches FreeRTOS** strictement isolée. Chaque tâche a une responsabilité unique ; la communication inter-tâches passe exclusivement par des **queues** et **mutex** FreeRTOS.

```
                        ┌──────────────────────────────────────────┐
                        │                  ESP32                   │
                        │                                          │
  [Script Python] ──UART2─► task_rx                                │
                        │      │ queue_frames   │ queue_mode (1,   │
                        │      │ (SETPOINT,     │  MODE_SET,       │
                        │      │  SPEED)        │  overwrite)      │
                        │      ▼                ▼                  │
                        │   task_control ──► [queue_tx]            │
                        │   (dispatch +  │                         │
                        │    PID 100ms)  │                         │
                        │                ▼                         │
                        │   task_telemetry ──► [queue_tx]          │
                        │                                          │
                        │   task_failsafe ──► [queue_tx]           │
                        │   (timeout 2s +                          │
                        │    GPIO ISR)                             │
                        │                ▼                         │
                        │            task_tx ──UART2─► [Script Python] │
                        └──────────────────────────────────────────┘
```

### Tâches FreeRTOS

| Tâche            | Fichier            | Rôle                                                        | Priorité | Périodicité      |
|------------------|--------------------|-------------------------------------------------------------|----------|------------------|
| `task_rx`        | `task_rx.c`        | Lecture UART octet par octet, parser état machine, CRC      | 4        | Event-driven     |
| `task_control`   | `task_control.c`   | Dispatch trames reçues + calcul PID + émission OUTPUT       | 4        | 100 ms (strict)  |
| `task_tx`        | `task_tx.c`        | Émission série protégée par mutex — aucun entrelacement     | 3        | Event-driven     |
| `task_telemetry` | `task_telemetry.c` | Émission STATS 0x83                                         | 2        | 1 s              |
| `task_failsafe`  | `failsafe.c`       | Supervision timeout 2 s + front GPIO externe (ISR)          | 5        | < 5 ms réponse   |

> **Choix architectural** : le dispatch des trames reçues est intégré dans `task_control` et non dans une tâche dédiée. La logique de dispatch (switch sur frame.type → mise à jour ecu_state) est triviale et ne justifie pas le coût d'une tâche supplémentaire (stack, context switch). `task_control` vide la queue de manière non bloquante en début de chaque cycle de 100 ms, puis calcule le PID.

### Ressources partagées

| Ressource              | Type FreeRTOS | Producteur(s)                                     | Consommateur(s)                                   |
|------------------------|---------------|---------------------------------------------------|---------------------------------------------------|
| `queue_frames`         | Queue (16)    | `task_rx` (SETPOINT, SPEED)                       | `task_control`                                    |
| `queue_mode`           | Queue (1), `xQueueOverwrite` | `task_rx` (MODE_SET)               | `task_control`                                    |
| `queue_tx`             | Queue         | `task_control`, `task_telemetry`, `task_failsafe` | `task_tx`                                         |
| `ecu_state_t`          | Mutex         | `task_rx`, `task_control`, `task_failsafe`        | `task_control`, `task_failsafe`                   |
| Compteurs statistiques | `volatile`, écrivain unique | `task_rx`, `task_tx`, `task_control` | `task_telemetry`                              |

> **MODE_SET ne passe pas par `queue_frames`**. Sous flood, `queue_frames` pleine perd la trame
> la plus récente — ce pouvait être un `MODE_SET(OFF)`. `queue_mode` (longueur 1, écrite par
> `xQueueOverwrite`) n'échoue jamais : le dernier MODE_SET reçu est toujours appliqué, au début
> du cycle de régulation suivant. Un MODE_SET écrasé avant lecture est compté
> (`mode_cmd_overwritten`).

---

## Protocole de Communication

Le bus CAN est simulé par une liaison **UART à 115 200 bauds**, sur l'**UART2** de l'ESP32
(TX = GPIO17, RX = GPIO16, cf. `main/board_pins.h`). L'UART0 (port USB natif de la carte)
est réservé à la console `ESP_LOG` / `idf.py monitor` : aucun log ne peut donc s'insérer
dans une trame du protocole.

### Format de trame

```
┌────────┬──────────┬────────┬──────────┬──────────────────┬──────────┐
│ START  │   LEN    │  TYPE  │  NONCE   │     PAYLOAD       │  CRC16   │
│ 0xAA   │ 2 octets │ 1 oct. │ 2 octets │    N octets       │ 2 octets │
└────────┴──────────┴────────┴──────────┴──────────────────┴──────────┘
```

- **START** : `0xAA` — marqueur de début de trame
- **LEN** : `uint16_t` little-endian — taille de `(TYPE + NONCE + PAYLOAD)`
- **TYPE** : identifiant du message
- **NONCE** : `uint16_t` little-endian — compteur strictement croissant côté émetteur,
  anti-rejeu (voir [Authentification légère](#authentification-légère))
- **PAYLOAD** : données utiles (taille variable)
- **CRC16** : CRC-16/CCITT (poly `0x1021`, init `0x0000`) sur tous les octets **sauf**
  le START, transmis little-endian

> Endianness : Little-Endian pour `float` (IEEE 754), `uint16_t`, `uint32_t`.

### Table des messages

| ID     | Nom        | Payload          | Direction | Description                        |
|--------|------------|------------------|-----------|------------------------------------|
| `0x01` | `SETPOINT` | `float` (4 B)    | RX        | Consigne de vitesse cible          |
| `0x02` | `SPEED`    | `float` (4 B)    | RX        | Vitesse mesurée courante           |
| `0x05` | `MODE_SET` | `uint8_t` (1 B)  | RX        | Changement de mode (0/1/2)         |
| `0x80` | `OUTPUT`   | `float` (4 B)    | TX        | Commande moteur calculée           |
| `0x83` | `STATS`    | `uint32_t × N`   | TX        | Télémétrie — compteurs cumulatifs  |
| `0x85` | `ALARM`    | `string`         | TX        | Alerte critique (failsafe)         |
| `0xFF` | `DBG`      | `string`         | TX        | Message de debug                   |

### Authentification légère

Chaque trame porte un **nonce** 16 bits, incrémenté par l'émetteur à chaque envoi. Le
récepteur (`task_rx`) n'accepte que les nonces strictement postérieurs au dernier accepté
— comparaison modulaire tolérante au wraparound, comme les numéros de séquence TCP. Toute
trame avec un nonce égal ou antérieur (rejeu, duplication, retard hors ordre) est rejetée
silencieusement, sans affecter le traitement des trames suivantes.

C'est une authentification *faible* — pas de MAC, un attaquant capable de forger le CRC
peut forger un nonce valide — mais elle ferme gratuitement la classe d'attaques la plus
simple contre un ECU : le rejeu d'une trame de commande légitime capturée sur le bus.

### Gestion des erreurs de protocole

Toute trame présentant un CRC invalide, une longueur incohérente, un ID inconnu ou un nonce
rejeté est **rejetée silencieusement** — sans blocage ni perte d'état. Une trame saine au
niveau protocole mais dont le payload n'a pas exactement la taille attendue pour son type,
ou porte une valeur hors domaine (float non fini, mode > 2), est rejetée de la même façon.
Les compteurs `rx_crc_error`, `rx_dropped`, `rx_replay` et `rx_invalid` sont incrémentés
selon le cas et remontés via STATS. Aucun rejet ne produit de log : un log par trame serait
déclenchable par l'émetteur et coûterait du temps dans le cycle de régulation.

---

## Modes de fonctionnement

| Mode     | Valeur | Comportement                                           |
|----------|--------|--------------------------------------------------------|
| `OFF`    | `0x00` | ECU inactif. `OUTPUT = 0` émis toutes les 100 ms.      |
| `MANUAL` | `0x01` | `OUTPUT = 0` émis toutes les 100 ms. Réservé usage futur. |
| `AUTO`   | `0x02` | Régulation PID active. OUTPUT émis toutes les 100 ms.  |

**OUTPUT est émis à chaque cycle de 100 ms, dans tous les modes**, et par une seule tâche :
`task_control`. Il vaut 0 hors `AUTO`. Une trame perdue sur la ligne est donc remplacée en
≤ 100 ms, et le flux d'OUTPUT sert de **signal de vie** à l'actionneur. Une variante « un seul
`OUTPUT = 0` à la sortie d'AUTO » a été écartée : si cette trame unique était perdue,
l'actionneur garderait sa dernière commande indéfiniment.

Un changement de mode (`MODE_SET`) est appliqué **au cycle de régulation suivant** : `task_control`
lit `queue_mode` en tout début de cycle. La latence entre la réception et l'application est donc
**≤ 100 ms**, plus le temps de réponse de `task_control` (quelques dizaines de µs). Elle est
**bornée et déterministe**, et ne dépend pas de la charge sur `queue_frames`. Toute transition hors
`AUTO` remet la commande à 0, émise dès ce cycle, et réinitialise l'intégrateur PID.

### Exigence future — ECU moteur (actionneur, axe 4)

| Réf.  | Exigence |
|-------|----------|
| EM-01 | **L'ECU moteur coupe le moteur si aucune trame OUTPUT n'est reçue pendant 300 ms**, soit 3 cycles. Cela couvre la coupure de l'ECU, du bus ou de son alimentation (`rta_fmea.md` F-16). Le testeur mesure déjà l'écart maximal entre deux OUTPUT. |

---

## Exigences fonctionnelles

| Réf.  | Exigence              | Détail                                                                            |
|-------|-----------------------|-----------------------------------------------------------------------------------|
| EF-01 | Régulation périodique | OUTPUT émis toutes les **100 ms ± 5 ms** (jitter max), dans tous les modes (0 hors AUTO) |
| EF-02 | Algorithme PID        | PID discret avec anti-windup, sortie saturée dans `[0, 255]`                      |
| EF-03 | Robustesse récepteur  | Trames malformées / incomplètes / invalides traitées sans blocage ni perte d'état |
| EF-04 | Télémétrie            | STATS émis toutes les **1 s** : rx_valid, rx_crc_error, rx_dropped, rx_replay, tx_output, tx_dropped, rx_invalid, mode_refused, mode_cmd_overwritten, uptime_s |
| EF-05 | Failsafe              | Déclenchement sur silence > 2 s **ou** ouverture de la ligne GPIO NC. **< 5 ms sur l'état interne** (commande à 0, mode OFF). ALARM et OUTPUT(0) sur la ligne bornés séparément (`rta_fmea.md` §6.1) |
| EF-06 | Reprise failsafe      | Reprise en mode AUTO dès réception d'un nouveau `MODE_SET` valide, ligne GPIO fermée |

---

## Exigences non fonctionnelles

| Réf.   | Exigence                                                                             |
|--------|--------------------------------------------------------------------------------------|
| ENF-01 | **Déterminisme** — période de régulation garantie même sous flood UART               |
| ENF-02 | **Isolation des défaillances** — anomalie RX sans impact sur régulation ni failsafe  |
| ENF-03 | **Intégrité des émissions** — aucun entrelacement d'octets inter-trames (`task_tx` seul écrivain UART) |
| ENF-04 | **Absence d'inversion de priorité** — mutex avec héritage de priorité activé         |
| ENF-05 | **Supervision matérielle** — Task Watchdog (TWDT) sur les 5 tâches, reboot en état sûr (OFF) après 2 s de blocage |

---

## Algorithme PID

Régulateur PID discret avec **anti-windup par gel de l'intégrale** en saturation.

```
e[k]     = setpoint - measure
integral = integral + e[k] × dt      (bloqué si output saturé et erreur dans le même sens)
output   = Kp×e[k] + Ki×integral + Kd×(e[k] - e[k-1])/dt
output   = clamp(output, 0.0, 255.0)
```

Entrée ou sortie non finie (NaN/Inf) → sortie forcée à `0.0`, état PID inchangé. Les
SETPOINT/SPEED non finis sont en plus rejetés dès le dispatch.

| Paramètre      | Valeur par défaut |
|----------------|-------------------|
| `Kp`           | `2.0`             |
| `Ki`           | `0.8`             |
| `Kd`           | `0.02`            |
| `dt`           | `0.1 s` (100 ms)  |
| Sortie min/max | `0.0 / 255.0`     |

---

## Mécanisme Failsafe

Deux événements déclenchent le failsafe **indépendamment** :

1. **Silence UART** — aucune trame valide reçue depuis **> 2 secondes**, détecté par `task_failsafe` via comparaison de `xTaskGetTickCount()` avec le dernier tick valide stocké dans `ecu_state`.
2. **Ligne failsafe GPIO ouverte** — contact **normalement fermé** : un jumper relie en
   permanence GPIO4 à GND (pull-up interne). Retirer le jumper **ou couper le fil** fait passer
   l'entrée à 1 : la rupture du câblage est elle-même une demande d'arrêt, il n'existe plus de
   défaut latent. Détection :
   - **front montant** → ISR → sémaphore binaire → `task_failsafe`, déclenchement dès le premier
     front, sans attente de stabilisation (réponse **< 5 ms** sur l'état interne) ;
   - **niveau** relu au boot et à chaque polling (100 ms) : une ligne déjà ouverte avant le
     démarrage, qui ne produira jamais de front, est détectée (ALARM « line open at boot »).

Actions au déclenchement :
- Commande moteur forcée à `0` et passage en mode `OFF` — **< 5 ms** après le front (EF-05)
- `OUTPUT = 0` émis par `task_control` à son prochain cycle, puis à chaque cycle.
  **L'arrêt physique du moteur a lieu quand l'organe actionneur reçoit cet `OUTPUT = 0`**,
  et non au changement d'état interne de l'ECU : ≤ 100 ms (attente du cycle) + temps de ligne,
  soit ~1 ms à vide et ≤ 41 ms au pire (`rta_fmea.md` §6.1). Au plus une trame non nulle déjà
  validée peut partir après le failsafe ; elle est remplacée en ≤ 100 ms (§6.1 bis)
- Émission d'un message `ALARM (0x85)` avec cause identifiée en clair — **une seule fois par
  entrée en failsafe** : les rebonds et fronts suivants n'en produisent pas d'autre
- LED GPIO25 allumée tant que le failsafe est actif

**Reprise** (EF-06) : réception d'un message `MODE_SET` valide. **Exception** : tant que la ligne
GPIO est ouverte, le failsafe reste verrouillé — un `MODE_SET(AUTO)` est refusé et compté
(`mode_refused` dans STATS). Le verrou se lève quand la ligne est relue fermée sur 2 polls
consécutifs (100–200 ms, anti-rebond à la remise du jumper) ; un nouveau `MODE_SET(AUTO)` est
alors nécessaire.

**Dernier rempart** : si une tâche (y compris `task_failsafe`) se bloque, le Task Watchdog
ESP-IDF déclenche un reboot au bout de 2 s ; l'ECU redémarre en mode `OFF`, commande 0.

## Analyse temps réel & sûreté

[`rta_fmea.md`](rta_fmea.md) : modèle de tâches sporadiques, équations de RTA, bornes de
blocage sur le mutex, cas `task_rx`/`task_control` à priorité égale, décomposition du
chemin failsafe, AMDEC/FMEA (20 modes de défaillance) et recommandations. Les WCET sont
encore symboliques en attente de la campagne de mesure sur cible.

---

## Structure du projet

```
ecu/
├── CMakeLists.txt
└── main/
    ├── CMakeLists.txt
    ├── main.c                  ← app_main : init + création tâches + démarrage scheduler
    ├── board_pins.h            ← Brochage ESP32 #1 (UART2, failsafe, LED, TWAI réservé)
    ├── ecu_state.h / .c        ← État partagé (mode, setpoint, speed, output, last_rx_tick)
    ├── protocol.h / .c         ← encode_frame, decode_frame, compute_crc — lib pure sans état
    ├── pid.h / .c              ← Calcul PID discret — lib pure sans état global
    ├── failsafe.h / .c         ← task_failsafe + ISR GPIO + LED
    ├── failsafe_line.h / .c    ← Machine à états ligne NC (latch, anti-rebond) — lib pure
    ├── task_rx.h / .c          ← Lecture UART + parser état machine
    ├── task_control.h / .c     ← Dispatch trames + boucle PID 100 ms
    ├── task_tx.h / .c          ← Émission série protégée mutex
    └── task_telemetry.h / .c   ← Émission STATS 1 s
```

---

## Build & Flash

```bash
# Activer l'environnement IDF
. ~/esp/esp-idf/export.sh

# Compiler
idf.py build

# Flasher et ouvrir la console (port USB natif de la carte, UART0)
idf.py -p /dev/ttyUSB0 flash monitor
```

> Deux ports série : le port USB natif de la carte (flash + console `ESP_LOG`) et un
> adaptateur USB-TTL 3,3 V câblé sur l'UART2 protocole (GPIO16 ← TX adaptateur,
> GPIO17 → RX adaptateur, GND commun). Le script de test parle **uniquement** à
> l'adaptateur.

| Commande            | Action                                 |
|---------------------|----------------------------------------|
| `idf.py build`      | Compilation                            |
| `idf.py flash`      | Flash du binaire                       |
| `idf.py monitor`    | Console série (quitter : `Ctrl+]`)     |
| `idf.py clean`      | Suppression build                      |
| `idf.py fullclean`  | Reset complet du dossier build         |
| `idf.py menuconfig` | Configuration FreeRTOS, UART, horloge  |

---

## Tests

### Tests automatisés (sans matériel)

```bash
make -C test/unit run                 # 188 assertions protocol.c, pid.c, failsafe_line.c (ASan/UBSan)
make -C test/fuzz fuzz SECONDS=60     # fuzzing libFuzzer de protocol_decode
```

Exécutés à chaque push par la CI GitHub Actions, avec le build firmware ESP-IDF.

### Test sur cible (`test/ecu_tester.py`)

| Phase                 | Durée | Description                                                        |
|-----------------------|-------|--------------------------------------------------------------------|
| Fonctionnement normal | 30 s  | Envoi SETPOINT + SPEED, vérification OUTPUT toutes les 100 ms      |
| Stress test           | —     | Flood de messages, trames fragmentées, CRC invalides, rejeu de trame |
| Reprise post-stress   | 5 s   | Vérification que la régulation repart correctement                 |
| Test failsafe         | 2.5 s | Silence radio → vérification OUTPUT = 0 et ALARM reçu              |
| Signal de vie         | —     | Écart max entre deux OUTPUT reçus ≤ 300 ms (EM-01), sur toute la session |

```bash
pip install pyserial
python test/ecu_tester.py   # SERIAL_PORT = adaptateur USB-TTL (UART2), pas le port USB de la carte
```

> Le jumper failsafe GPIO4 → GND doit être en place : sans lui, l'ECU démarre en failsafe
> verrouillé, refuse `MODE_SET(AUTO)` (STATS `mode_refused` qui monte) et n'émet aucun OUTPUT.