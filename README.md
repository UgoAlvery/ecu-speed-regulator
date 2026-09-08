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
  [Script Python] ──UART──► task_rx                                │
                        │      │ queue_frames                      │
                        │      ▼                                   │
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
                        │            task_tx ──UART──► [Script Python] │
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
| `queue_frames`         | Queue         | `task_rx`                                         | `task_control`                                    |
| `queue_tx`             | Queue         | `task_control`, `task_telemetry`, `task_failsafe` | `task_tx`                                         |
| `ecu_state_t`          | Mutex         | `task_control`, `task_failsafe`                   | `task_control`, `task_telemetry`, `task_failsafe` |
| Compteurs statistiques | `volatile`    | `task_rx`                                         | `task_telemetry`                                  |

---

## Protocole de Communication

Le bus CAN est simulé par une liaison **UART à 115 200 bauds**.

### Format de trame

```
┌────────┬──────────┬────────┬──────────────────┬───────┐
│ START  │   LEN    │  TYPE  │     PAYLOAD       │  CRC  │
│ 0xAA   │ 2 octets │ 1 oct. │    N octets       │ 1 oct.│
└────────┴──────────┴────────┴──────────────────┴───────┘
```

- **START** : `0xAA` — marqueur de début de trame
- **LEN** : `uint16_t` little-endian — taille de `(TYPE + PAYLOAD)`
- **TYPE** : identifiant du message
- **PAYLOAD** : données utiles (taille variable)
- **CRC** : XOR de tous les octets **sauf** le START

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

### Gestion des erreurs de protocole

Toute trame présentant un CRC invalide, une longueur incohérente ou un ID inconnu est **rejetée silencieusement** — sans blocage ni perte d'état. Les compteurs `rx_crc_error` et `rx_dropped` sont incrémentés selon le cas et remontés via STATS.

---

## Modes de fonctionnement

| Mode     | Valeur | Comportement                                           |
|----------|--------|--------------------------------------------------------|
| `OFF`    | `0x00` | ECU inactif. Commande moteur forcée à 0.               |
| `MANUAL` | `0x01` | Commande moteur à 0. Réservé usage futur.              |
| `AUTO`   | `0x02` | Régulation PID active. OUTPUT émis toutes les 100 ms.  |

Un changement de mode est pris en compte **immédiatement** par `task_control`. Toute transition hors `AUTO` remet la commande moteur à 0 et réinitialise l'intégrateur PID.

---

## Exigences fonctionnelles

| Réf.  | Exigence              | Détail                                                                            |
|-------|-----------------------|-----------------------------------------------------------------------------------|
| EF-01 | Régulation périodique | OUTPUT émis toutes les **100 ms ± 5 ms** (jitter max) en mode AUTO                |
| EF-02 | Algorithme PID        | PID discret avec anti-windup, sortie saturée dans `[0, 255]`                      |
| EF-03 | Robustesse récepteur  | Trames malformées / incomplètes / invalides traitées sans blocage ni perte d'état |
| EF-04 | Télémétrie            | STATS émis toutes les **1 s** : rx_valid, rx_crc_error, rx_dropped, tx_output     |
| EF-05 | Failsafe              | Déclenchement sur silence > 2 s **ou** front GPIO externe (réponse < 5 ms)        |
| EF-06 | Reprise failsafe      | Reprise en mode AUTO dès réception d'un nouveau `MODE_SET` valide                 |

---

## Exigences non fonctionnelles

| Réf.   | Exigence                                                                             |
|--------|--------------------------------------------------------------------------------------|
| ENF-01 | **Déterminisme** — période de régulation garantie même sous flood UART               |
| ENF-02 | **Isolation des défaillances** — anomalie RX sans impact sur régulation ni failsafe  |
| ENF-03 | **Intégrité des émissions** — aucun entrelacement d'octets inter-trames (mutex TX)   |
| ENF-04 | **Absence d'inversion de priorité** — mutex avec héritage de priorité activé         |

---

## Algorithme PID

Régulateur PID discret avec **anti-windup par gel de l'intégrale** en saturation.

```
e[k]     = setpoint - measure
integral = integral + e[k] × dt      (bloqué si output saturé et erreur dans le même sens)
output   = Kp×e[k] + Ki×integral + Kd×(e[k] - e[k-1])/dt
output   = clamp(output, 0.0, 255.0)
```

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
2. **Front GPIO externe** — ISR qui donne un sémaphore binaire à `task_failsafe`, temps de réponse **< 5 ms**.

Actions au déclenchement (atomiques) :
- Commande moteur forcée à `0`
- Passage en mode `OFF`
- Émission immédiate d'un message `ALARM (0x85)` avec cause identifiée en clair

**Reprise** : réception d'un message `MODE_SET` valide relance l'ECU normalement depuis n'importe quel mode.

---

## Structure du projet

```
ecu/
├── CMakeLists.txt
└── main/
    ├── CMakeLists.txt
    ├── main.c                  ← app_main : init + création tâches + démarrage scheduler
    ├── ecu_state.h / .c        ← État partagé (mode, setpoint, speed, output, last_rx_tick)
    ├── protocol.h / .c         ← encode_frame, decode_frame, compute_crc — lib pure sans état
    ├── pid.h / .c              ← Calcul PID discret — lib pure sans état global
    ├── failsafe.h / .c         ← task_failsafe + ISR GPIO
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

# Flasher et ouvrir le moniteur série
idf.py -p /dev/ttyUSB0 flash monitor
```

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

| Phase                 | Durée | Description                                                        |
|-----------------------|-------|--------------------------------------------------------------------|
| Fonctionnement normal | 30 s  | Envoi SETPOINT + SPEED, vérification OUTPUT toutes les 100 ms      |
| Stress test           | —     | Flood de messages, trames fragmentées, CRC invalides, IDs inconnus |
| Reprise post-stress   | 5 s   | Vérification que la régulation repart correctement                 |
| Test failsafe         | 2.5 s | Silence radio → vérification OUTPUT = 0 et ALARM reçu              |

```bash
pip install pyserial
python test/ecu_tester.py   # adapter SERIAL_PORT si nécessaire
```