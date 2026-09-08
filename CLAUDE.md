# CLAUDE.md — ecu-speed-regulator

## Contexte du projet

ECU (Electronic Control Unit) de régulation de vitesse temps réel, sur ESP32 (Xtensa LX6),
FreeRTOS, ESP-IDF v5.3. Point de départ : TP EPITA 2026 (Christophe Duvernois), en cours de
transformation en projet personnel portfolio (voir roadmap). Chaque axe d'amélioration doit
produire une **preuve** (mesure, test automatisé, document d'analyse) — pas juste une
fonctionnalité de plus.

**Avant toute implémentation ou modification substantielle, lire `technical_reference.md` à
la racine du repo.** C'est la référence architecturale figée du projet — plus fiable que la
mémoire de session. Après une implémentation, mettre à jour son statut dans ce fichier
(⬜ → ✅).

## Environnement

- OS de dev : WSL2 (Ubuntu 24.04 LTS) sur Windows
- IDE : CLion (le dossier ESP-IDF lié dans CLion est une référence au SDK global — ne jamais
  le committer)
- Toolchain : ESP-IDF v5.3, activé via `. ~/esp/esp-idf/export.sh`
- Cible : ESP32 classique (`idf.py set-target esp32`)
- Versionnement : Git/GitHub, SSH Ed25519, repo `ecu-speed-regulator`
- `sdkconfig` **doit être committé** (paramètres FreeRTOS/ESP-IDF identiques entre sessions)
- WSL2 n'hérite pas des variables d'environnement dans CLion → les profils CMake doivent
  hardcoder `IDF_PATH` et les chemins toolchain

## Commandes de build

```bash
idf.py build                          # compilation
idf.py -p /dev/ttyUSB0 flash monitor  # flash + console série (quitter: Ctrl+])
idf.py clean                          # si bug de compilation étrange
idf.py fullclean                      # reset complet du dossier build
idf.py menuconfig                     # config FreeRTOS / UART / horloge
```

## Langage et style

- **C uniquement** (pas de C++) — ESP-IDF et FreeRTOS sont pensés C.
- **Pas de malloc/free dans les tâches temps réel** — buffers statiques ou sur stack
  uniquement.
- Pas de variable globale non protégée partagée entre tâches (exception : compteurs
  `volatile` de `task_rx`, un seul écrivain — cohérence approximative acceptable).
- Pas de `vTaskDelay` dans `task_control` — utiliser `vTaskDelayUntil` pour la période stricte
  100 ms.
- Retour 0/false en cas d'erreur dans les libs pures, jamais d'assert en prod (`NDEBUG`
  désactive `assert()`, donc les analyseurs statiques ne le traitent pas comme garantie de
  non-nullité — préférer des guards explicites).
- `vTaskStartScheduler()` n'est pas nécessaire en ESP-IDF ; retourner depuis `app_main` est
  safe.

## Architecture (ne pas dévier sans justification documentée)

Architecture multi-tâches FreeRTOS strictement isolée. Communication inter-tâches
exclusivement par queues et mutex — jamais de variable globale partagée non protégée.

```
task_rx ──queue_frames──► task_control ──┐
                                          ├──queue_tx──► task_tx ──UART──►
task_telemetry ──────────────────────────┤
task_failsafe ────────────────────────────┘
```

| Tâche            | Priorité | Périodicité      | Réveil                              |
|------------------|----------|------------------|--------------------------------------|
| `task_failsafe`  | 5        | < 5 ms réponse   | sémaphore binaire (ISR GPIO) + poll  |
| `task_rx`        | 4        | event-driven     | `uart_read_bytes` bloquant 50 ms     |
| `task_control`   | 4        | 100 ms strict    | `vTaskDelayUntil`                    |
| `task_tx`        | 3        | event-driven     | `xQueueReceive` bloquant             |
| `task_telemetry` | 2        | 1 s              | `vTaskDelayUntil`                    |

Règles structurelles à préserver :
- `protocol.c` et `pid.c` sont des **libs pures sans dépendance FreeRTOS/ESP-IDF**
  (`protocol.h` n'inclut pas `<driver/uart.h>`) — elles doivent rester testables sur machine
  de dev sans hardware ni scheduler.
- Pas de `task_dispatcher` dédiée : le dispatch (switch sur `frame.type` → `ecu_state`) est
  intégré dans `task_control` en début de cycle, non bloquant (`xQueueReceive` timeout 0).
  Un seul producteur/consommateur, logique triviale — ne pas réintroduire cette tâche sans
  raison forte.
- `task_tx` est le seul écrivain UART et le seul consommateur de `queue_tx` — exclusion
  mutuelle structurelle, pas besoin de mutex (conformité ENF-03, pas d'entrelacement de
  trames).
- L'encodage de trame est centralisé dans `task_tx`, pas dans les tâches productrices ; la
  queue transporte des `tx_message_t` compacts.
- `last_rx_tick` transite par `ecu_state` — pas de dépendance directe `task_rx` ↔
  `task_failsafe`.
- Tout accès à `ecu_state_t` passe par les setters/getters dédiés (mutex pris en interne) —
  jamais de getter/setter combiné (évite les races read-modify-write), jamais d'exposition
  directe du mutex.
- `configUSE_MUTEXES = 1` et `configUSE_RECURSIVE_MUTEXES = 1` requis dans `sdkconfig`
  (héritage de priorité sur `mutex_ecu_state`, conformité ENF-04).

## Protocole (format figé, cf. `technical_reference.md` pour l'interface C)

```
[START: 0xAA] [LEN: 2 octets] [TYPE: 1 octet] [PAYLOAD: N octets] [CRC16: 2 octets LE]
```
- LEN = taille(TYPE + PAYLOAD), little-endian
- CRC16 = CRC-16/CCITT (polynôme 0x1021, init 0x0000) sur tous les octets sauf START,
  transmis little-endian sur 2 octets. Détecte rafales ≤16 bits et toute double erreur.
- Toute trame CRC invalide / longueur incohérente / ID inconnu → rejet silencieux, sans
  bloquer le traitement des trames suivantes ; incrémenter le compteur d'erreur concerné.

## État d'avancement

Tous les modules firmware sont implémentés et validés hardware (PID, stress test, failsafe
silence UART). Reste en attente : test du failsafe GPIO (bloqué faute de jumper wire),
et tout l'axe hardware (second ESP32, CAN, moteur/encodeur réel) — en attente de livraison
matériel.

Travail en cours, exécutable sans hardware, dans cet ordre (voir roadmap complète pour le
détail) :
1. ✅ CRC16 en remplacement du XOR sur `protocol.c` + `task_rx.c` + script Python
2. ✅ Mise à jour `technical_reference.md` + script Python de test en conséquence
3. ✅ Tests unitaires hébergés (harness maison, sans dépendance externe) sur `protocol.c`
   et `pid.c`
4. ✅ Fuzzing de `protocol_decode` (libFuzzer/clang, compilation host)
5. ✅ CI GitHub Actions (build firmware + tests hébergés + smoke-test fuzzing)
6. Authentification légère (nonce/anti-rejeu) sur `protocol.c`
7. RTA formelle (modèle sporadique, blocking term du mutex chiffré) + FMEA/AMDEC
8. Watchdog Task Watchdog Timer (TWDT) — implémentation possible, validation attend le
   hardware

## Documentation

- `README.md` : vue d'ensemble présentable (architecture, protocole, exigences, build)
- `technical_reference.md` : référence interne figée, à tenir à jour à chaque module terminé
- `archi.md` : arborescence des fichiers du projet
- Rapport de TP : corrigé et finalisé — ne pas le retoucher sauf demande explicite

## Conventions de communication

- Réponses techniques concises, en français, pour tout ce qui touche à l'embarqué/RTOS.
- Signaler le code mort et les warnings d'analyse statique plutôt que de les ignorer.
- Préférer fournir des fichiers corrigés/complets plutôt que des diffs ou suggestions
  partielles.
