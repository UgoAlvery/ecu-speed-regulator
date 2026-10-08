# RTA & FMEA — ECU Régulateur de Vitesse

> Roadmap, axe 1 (actions 3, 4 et 5). Ce document rassemble l'analyse
> d'ordonnançabilité (RTA) des 5 tâches FreeRTOS et l'AMDEC/FMEA du système.
>
> **Statut** : analyse **structurelle** terminée (modèle de tâches, termes de
> blocage, cas de priorité égale, chemin failsafe). Les **WCET** restent
> symboliques (`C_x`). Ils seront remplis par la campagne de mesure (axe 1,
> action 2). Les cases ⏳ marquent tout ce qui attend une mesure : ne pas les
> lire comme des garanties.

---

## 1. Hypothèses de plateforme

Valeurs extraites de `sdkconfig` et `main.c` (état actuel du repo) :

| Paramètre | Valeur | Conséquence pour l'analyse |
|---|---|---|
| CPU | ESP32 double cœur, 160 MHz (`CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ`) | — |
| Noyau | FreeRTOS ESP-IDF v5.3 (variante IDF, `CONFIG_FREERTOS_SMP` désactivé) | Round-robin « best effort » (cf. §5) |
| Tick | `CONFIG_FREERTOS_HZ=100` → **10 ms** | Granularité de `vTaskDelayUntil`, tranche de time-slicing |
| Time slicing | activé (défaut IDF) | Rotation entre tâches prêtes de même priorité à chaque tick |
| Affinité | `xTaskCreate` → `tskNO_AFFINITY` | Les 5 tâches peuvent tourner sur l'un ou l'autre cœur |
| UART protocole | UART2 (GPIO17 TX / GPIO16 RX), 115 200 bauds 8N1 → **86,8 µs/octet** | Débit max 11 520 o/s |
| Buffer RX UART | `UART_BUF_SIZE * 2` = 2048 o | ≈ 178 ms de flood absorbable |
| Buffer TX UART | 0 (pas de ring buffer) | `uart_write_bytes` bloque jusqu'à ce que la trame tienne dans la FIFO HW (128 o) |
| Console `ESP_LOG` | UART0, séparé du protocole depuis D1 | Plus d'octet de log dans une trame ; plus aucun log dans le cycle de `task_control` (§4.3) |
| Mutex | `mutex_ecu_state`, héritage de priorité | Un seul mutex, jamais imbriqué |

**Choix d'analyse** : la RTA est conduite en **monoprocesseur**, comme si les
5 tâches étaient sur le même cœur. C'est un majorant sûr. Le second cœur ne
peut que réduire les temps de réponse tant que les tâches ne sont pas épinglées
et qu'aucune autre charge n'occupe l'autre cœur. Sa seule exception est
l'accès au mutex, où le double cœur *augmente* le blocage possible (§4.2).

---

## 2. Modèle de tâches

Les tâches événementielles sont modélisées comme **sporadiques**, avec un temps
d'inter-arrivée minimal réel (et non une période fictive).

| Tâche | Prio | Modèle | T / inter-arrivée min | D | C (WCET) |
|---|---|---|---|---|---|
| `task_failsafe` | 5 | Périodique (polling) + sporadique (ISR GPIO) | 100 ms (polling) ; GPIO : non borné, cf. §6.2 | 5 ms (EF-05) | `C_fs` ⏳ |
| `task_rx` | 4 | Sporadique, par octet | 86,8 µs (1 octet à 115 200 bauds) | — (aucune perte tant que le buffer de 2048 o ne déborde pas) | `C_rx` par octet ⏳ |
| `task_control` | 4 | Périodique strict | 100 ms | 100 ms, gigue de libération ±5 ms (EF-01) | `C_ctrl` ⏳ |
| `task_tx` | 3 | Sporadique, par message | Charge nominale : 10 OUTPUT (tous modes, depuis D7) + 1 STATS /s (+ 1 ALARM par failsafe) | — | `C_tx` par message ⏳ (inclut l'attente FIFO UART) |
| `task_telemetry` | 2 | Périodique | 1 s | 1 s | `C_tel` ⏳ |

Interférences hors tâches (ISR), à mesurer avec le reste : tick (100 Hz), ISR
UART RX (seuil FIFO / timeout RX) et ISR GPIO failsafe.

### Ordres de grandeur, avant mesure

Ce ne sont **pas** des résultats. Ils servent seulement à dimensionner la
campagne de mesure et à repérer les termes qui comptent.

- `C_ctrl` : un vidage de `queue_frames` (≤ 16 trames) + 4 à 5 accès sous
  mutex + un calcul PID en `float` (FPU matérielle). Dizaines de µs, au plus
  quelques centaines avec une queue pleine. **Sauf log** : un `ESP_LOGI` sur
  UART0 coûte jusqu'à ~87 µs par caractère si la FIFO est pleine (§4.3).
- `C_rx` : `uart_read_bytes` d'un seul octet (prise du ring buffer) + une
  transition du parser. Quelques µs à quelques dizaines de µs. **Le point
  critique est de savoir si `C_rx` reste < 86,8 µs** : sous cette valeur,
  `task_rx` ne peut pas rester prête en continu sous flood (§5).
- `C_tx` : il est dominé par l'UART, pas par le CPU. Une trame OUTPUT fait
  12 o, soit 1,04 ms de ligne ; une trame STATS fait 36 o, soit 3,13 ms.

### Tailles de trame (surcoût fixe de 8 o : START + LEN + TYPE + NONCE + CRC)

| Trame | Payload | Taille | Temps de ligne à 115 200 bauds |
|---|---|---|---|
| MODE_SET | 1 o | 9 o | 0,78 ms |
| SETPOINT / SPEED / OUTPUT | 4 o | 12 o | 1,04 ms |
| STATS | 48 o (12 × u32, depuis D9) | 56 o | 4,86 ms |
| ALARM « UART silence » | 27 o | 35 o | 3,04 ms |
| ALARM « GPIO » (cause la plus longue, depuis D2) | 33 o | 41 o | 3,56 ms |

---

## 3. Équation de RTA

Pour chaque tâche *i* :

```
R_i = C_i + B_i + Σ_{j ∈ hp(i)} ⌈R_i / T_j⌉ · C_j + I_eq(i) + I_isr(i)
```

- `hp(i)` : tâches de priorité **strictement** supérieure
- `B_i` : blocage par une tâche de priorité inférieure (§4)
- `I_eq(i)` : interférence d'une tâche de **même** priorité. Elle n'apparaît
  pas dans la RTA classique et est traitée à part au §5.
- `I_isr(i)` : ISR (tick, UART, GPIO)

### `task_control` (le cas qui porte EF-01)

- `hp(ctrl) = {failsafe}` → `⌈R/100 ms⌉ · C_fs`, soit **une** activation de
  polling par cycle, plus les éventuelles rafales GPIO (§6.2).
- `B_ctrl = 0` côté priorités inférieures : `task_tx` et `task_telemetry`
  **n'accèdent pas** à `mutex_ecu_state` (tx ne touche pas `ecu_state`,
  telemetry ne lit que les compteurs `volatile`). Aucune tâche de priorité
  inférieure ne peut donc bloquer `task_control` via le mutex.
- `I_eq(ctrl)` : interférence de `task_rx` (§5).

```
R_ctrl ≤ C_ctrl + C_fs + I_eq(ctrl) + I_isr(ctrl)
```

L'exigence EF-01 (gigue ±5 ms) porte sur la **date d'émission** de l'OUTPUT,
pas seulement sur l'échéance de 100 ms. La libération est calée sur le tick
(`vTaskDelayUntil`), donc la gigue de sortie vaut `R_ctrl` moins sa valeur
minimale. **Condition suffisante** :

```
C_fs + I_eq(ctrl) + I_isr(ctrl) + (C_ctrl_max − C_ctrl_min) ≤ 5 ms
```

`C_fs` et `I_isr` sont de l'ordre de la dizaine de µs. **Tout le budget
dépend donc de `I_eq(ctrl)`**, c'est-à-dire du cas de priorité égale du §5.

### `task_failsafe`

`hp(fs) = ∅`. `R_fs = C_fs + B_fs + I_isr`, avec `B_fs` borné au §4.2. Voir
le §6 pour le chemin complet jusqu'à la trame sur le bus.

### `task_tx` et `task_telemetry`

Ces deux tâches n'ont pas d'échéance dure. Le seul critère est l'absence de
famine. La charge en émission est d'environ 176 o/s (10 × 12 + 56), soit
1,53 % du lien, dans tous les modes depuis D7, ce qui laisse une marge très large. Les priorités 4 et 5
cumulent `C_fs/100 ms + C_ctrl/100 ms + 11 520 · C_rx` : tant que
`C_rx < 86,8 µs`, la charge reste < 1 et `task_tx` progresse, même sous flood.

---

## 4. Termes de blocage — `mutex_ecu_state`

### 4.1 Sections critiques

Chaque getter et setter d'`ecu_state.c` prend le mutex, copie **un seul
champ** (4 octets) puis le rend. Aucune section critique ne contient d'appel
bloquant ni d'imbrication de verrou.

| Tâche | Accès par activation |
|---|---|
| `task_rx` | 1 (`set_last_rx_tick`) par trame valide |
| `task_control` | 1 (`get_mode`) + 1 (`commit_output`, tous modes, depuis D7) + 2 en AUTO (`get_setpoint`, `get_speed`) + 1 par trame dispatchée, + 1 par MODE_SET (`try_set_mode`) |
| `task_failsafe` | 2 au polling (`get_mode`, `get_last_rx_tick`) + 1 au déverrouillage (`set_failsafe_locked`) ; **1** au déclenchement (`enter_failsafe` : verrou + commande 0 + OFF en une prise, depuis D7). Pire réveil : déclenchement sur front + poll dans la même itération = **3** |
| `task_telemetry`, `task_tx` | 0 |

`C_cs` désigne la durée d'une section critique : prise et restitution du
mutex, avec le spinlock IDF. Ordre de grandeur : quelques µs. ⏳ à mesurer.

### 4.2 Bornes

- **Monocœur, héritage de priorité** (Priority Inheritance Protocol) : une
  tâche est bloquée au plus une fois par section critique d'une tâche moins
  prioritaire. Pour `task_failsafe` : `B_fs ≤ C_cs`. Une fois le mutex
  relâché, rx et control (prio 4) ne peuvent plus le reprendre avant que
  failsafe (prio 5) ne se bloque.
- **Double cœur** : rx ou control, s'ils tournent sur l'autre cœur, peuvent
  reprendre le mutex *entre* deux accès de failsafe. Chaque accès peut donc
  être bloqué : `B_fs ≤ 3 · C_cs` (3 accès au plus par réveil depuis D7 ; 5 entre D2 et D7). Cela
  reste négligeable devant 5 ms.
- **Deadlock** : structurellement impossible. Il n'y a qu'un mutex, jamais
  pris de façon imbriquée, et aucun appel bloquant sous le mutex.

### 4.3 Termes de blocage cachés trouvés pendant l'analyse

1. **`enqueue_output()` bloquait jusqu'à 10 ms** sur `xQueueSend` quand
   `queue_tx` était pleine. Ce terme n'était pas borné dans le budget de
   ±5 ms. ✅ **Corrigé** : timeout 0 et compteur `tx_dropped` dans STATS.
2. **`ESP_LOGW` sur ce même chemin** : il se déclenchait exactement en
   surcharge, quand `queue_tx` est pleine. La console partage UART0 avec le
   protocole, donc un log d'environ 40 caractères peut coûter jusqu'à ~3,5 ms
   si la FIFO TX est occupée par `task_tx`. ✅ **Corrigé** : log supprimé, le
   compteur suffit.
3. **Logs restants dans `task_control`** : `ESP_LOGI` sur MODE_SET et sur les
   transitions de mode, `ESP_LOGW` sur payload trop court. Majorant : environ
   50 caractères × 86,8 µs ≈ **4,3 ms par log**.
   - MODE_SET et transition : sur événement, 2 logs au plus dans un même
     cycle. Le cycle où le mode change peut dépasser le budget de ±5 ms.
   - **Payload trop court : déclenchable par l'émetteur.** Une trame
     SETPOINT avec LEN valide mais un payload de 0 à 3 o passe le CRC. Un
     flood de telles trames produit jusqu'à 16 logs par cycle (taille de
     `queue_frames`), soit environ 70 ms : **amplification** d'un flood en
     violation d'EF-01.
   - Le rejet des valeurs non finies ajouté au §7 (F-05) loggue en
     `ESP_LOGD` (compilé mais filtré au niveau INFO) pour ne pas recréer ce
     problème.

   ✅ **Traité (D1)** : tous les `ESP_LOG*` de la boucle et du dispatch de
   `task_control` ont été supprimés, remplacés par le compteur `rx_invalid`
   (STATS). Il ne reste que le log de démarrage, hors cycle. Le protocole
   est en plus sur UART2, donc une écriture console ne concurrence plus la
   FIFO TX de `task_tx`.

### 4.4 `queue_mode` (D3) — impact nul sur les termes de blocage

MODE_SET ne passe plus par `queue_frames` mais par `queue_mode` : queue
FreeRTOS de longueur 1 contenant un `ecu_mode_t`, écrite par task_rx avec
`xQueueOverwrite`, lue par task_control avec `xQueueReceive(…, 0)` en tout
début de cycle.

- **Ce n'est pas un mutex** : une opération de queue est une section critique
  noyau (spinlock IDF) de durée constante, sans attente et sans héritage de
  priorité. Aucune des deux opérations n'est bloquante (`xQueueOverwrite` ne
  peut pas échouer, `xQueueReceive` a un timeout de 0).
- **task_rx** : aucun nouvel accès à `mutex_ecu_state`. Son seul accès reste
  `set_last_rx_tick` (§4.1). Il ne fait que valider le payload
  (`protocol_parse_mode`, pur) et écraser la queue.
- **task_control** : `+1` `xQueueReceive` non bloquant par cycle. La demande de
  mode appliquée passe par `ecu_state_try_set_mode`, soit **une** section
  critique de même durée que l'ancien `set_mode` du dispatch. Le nombre
  d'accès au mutex par cycle est inchangé.
- `B_ctrl` reste nul (aucune tâche moins prioritaire ne partage le mutex),
  `B_fs` est inchangé par D3.
- Latence d'application d'un MODE_SET : il est lu au début du cycle suivant,
  soit **≤ 100 ms + `R_ctrl`**, bornée et indépendante de la charge sur
  `queue_frames`. Deux MODE_SET dans le même cycle : seul le dernier est
  appliqué (`mode_cmd_overwritten` dans STATS).

---

## 5. Cas `task_rx` / `task_control` à priorité égale (4)

C'est l'hypothèse « elles ne se préemptent pas en pratique » de la version
initiale de `technical_reference.md`. Elle n'était **pas prouvée**.

### 5.1 Mécanismes d'interférence (monocœur)

**(a) Libération de `task_control` pendant que `task_rx` s'exécute.** Le tick
qui libère control la place en fin de liste prête de priorité 4. En FreeRTOS
standard, avec `configUSE_TIME_SLICING=1`, ce tick provoque un changement de
contexte dès que plus d'une tâche est prête à la priorité courante. Avec
deux tâches seulement, la rotation sélectionne control. Le délai est alors
≈ 0. **Mais** l'ordonnanceur IDF documente son round-robin comme « best
effort » : une tâche peut être sautée selon l'état des deux cœurs. On ne peut
donc pas compter sur ce comportement. **Majorant sûr : une tranche, soit
10 ms.**

**(b) Passage du mutex.** Control se bloque sur `mutex_ecu_state` tenu par rx
(`set_last_rx_tick`). Au `xSemaphoreGive`, FreeRTOS ne préempte que si la
tâche débloquée a une priorité **strictement** supérieure. Rx, de même
priorité, continue donc de tourner jusqu'à ce qu'elle se bloque
(`uart_read_bytes` sur un ring buffer vide) ou jusqu'au tick suivant (time
slicing). **Borne : min(10 ms, temps de vidage du backlog RX).**

**(c) `task_rx` prête en continu.** Cela n'arrive que si
`C_rx ≥ 86,8 µs/octet`, c'est-à-dire si rx consomme les octets moins vite
qu'ils n'arrivent. Sinon rx vide son backlog puis se bloque. Le backlog
accumulé pendant que rx était préemptée vaut à peu près
`(C_ctrl + C_fs) × 11,5 o/ms`, soit quelques octets.

### 5.2 Synthèse

| Hypothèse sur `C_rx` | `I_eq(ctrl)` | EF-01 (±5 ms) |
|---|---|---|
| `C_rx < 86,8 µs` (attendu) | ≤ temps de vidage du backlog, soit quelques dizaines de µs à ~1 ms | ✅ tenu (sous réserve de la mesure) |
| `C_rx ≥ 86,8 µs` | jusqu'à 10 ms par mécanisme ; (a) + (b) dans le même cycle → **10 à 20 ms** | ❌ violé |

**Double cœur** : comme les tâches ne sont pas épinglées, quand control est
débloquée alors que l'autre cœur exécute IDLE, l'IDF provoque un *yield* de
l'autre cœur. Control y démarre immédiatement. Le pire cas monocœur exige
que les **deux** cœurs soient occupés par des tâches de priorité ≥ 4 (rx +
failsafe), ce qui est rare. Mais le pire cas n'est pas exclu, et
l'épinglage futur (par exemple TWAI ou Wi-Fi en axe 4) peut le rendre
réaliste.

**Statut** : ⏳ **risque ouvert**. Deux mesures le tranchent : `C_rx` et la
gigue de sortie de `task_control` sous flood (campagne WCET, §8). Le
correctif structurel est la recommandation R-1.

---

## 6. Chemin failsafe et EF-05 (< 5 ms)

### 6.1 Décomposition du chemin GPIO

Depuis D7, **task_control est l'unique émetteur d'OUTPUT** : il en émet un à
chaque cycle de 100 ms, dans tous les modes, à 0 hors AUTO. task_failsafe ne
fait que mettre l'état à jour et envoyer l'ALARM.

```
front GPIO4 ─► ISR (xSemaphoreGiveFromISR) ─► task_failsafe (prio 5)
            ─► ecu_state_enter_failsafe : verrou, output=0, OFF  ← (1) état interne sûr
            ─► queue_tx, EN TÊTE : ALARM  (SendToFront, timeout 0) ← (2) annonce
   prochain cycle task_control (≤ 100 ms) :
            ─► ecu_state_commit_output → 0 ─► queue_tx : OUTPUT(0) ← (3) arrêt moteur
            ─► task_tx (prio 3) ─► uart_write_bytes ─► ligne UART2
```

| Étape | Borne | Commentaire |
|---|---|---|
| (1) État interne sûr | latence ISR + `C_fs` + `B_fs` ≈ **dizaines de µs** | Prio 5, préemption immédiate. ✅ largement < 5 ms (⏳ à confirmer en mesure) |
| (2) ALARM sur la ligne | temps de ligne (ci-dessous) + interférence de task_tx par les tâches de prio ≥ 4 | Borne structurelle ci-dessous |
| (3) OUTPUT(0) sur la ligne | attente du prochain cycle de task_control (**≤ 100 ms** + `R_ctrl`) + temps de ligne | L'arrêt physique du moteur a lieu ici |

**Borne du temps de ligne.** Hypothèse : le message entre dans `queue_tx`.
Sinon il est perdu : l'ALARM est comptée dans `alarm_dropped`, l'OUTPUT dans
`tx_dropped` et remplacé au cycle suivant. Le protocole est sur UART2 depuis
D1, donc aucun octet de console ne s'intercale. La FIFO TX matérielle peut
contenir jusqu'à **128 o** déjà écrits.

- **ALARM (D8, `xQueueSendToFront`)** : elle passe devant tous les messages en
  attente. Restent devant elle la FIFO (≤ 128 o) et la trame que task_tx est
  en train d'écrire (≤ 56 o, un STATS). Seule une autre ALARM postée en tête
  *après* elle peut la doubler, et il y en a au plus une autre par épisode :
  silence UART puis ligne GPIO, ou l'inverse.
- **OUTPUT(0)** (ajouté en queue) : au plus 7 messages devant, plus la FIFO.

| Scénario | (2) Fin de l'ALARM (41 o) | (3) Fin de l'OUTPUT(0) (12 o), après le cycle |
|---|---|---|
| Nominal (queue et FIFO vides) | **3,6 ms** | **1,0 ms** |
| Pessimiste, ALARM seule | 128 + 56 + 41 = 225 o → **19,5 ms** | — |
| Pessimiste, une seconde ALARM la double | 128 + 56 + 41 + 41 = 266 o → **23,1 ms** | — |
| Pessimiste (FIFO pleine, 7 messages de taille max = STATS 56 o) | (avant D8, en fin de file : 128 + 392 + 41 = 561 o → 48,7 ms) | 128 + 392 + 12 = 532 o → **46,2 ms** |

S'y ajoute l'interférence de `task_tx` par les tâches de priorité ≥ 4 pendant
cette fenêtre (`C_ctrl`, `C_fs`, `C_rx` × octets reçus). Elle est finie tant
que `C_rx < 86,8 µs` (§5). ⏳ À mesurer : front GPIO4 → fin de trame ALARM
et fin de la première trame OUTPUT(0) sur GPIO17.

**Conclusion** : EF-05 (< 5 ms) porte sur **l'état interne**. L'ALARM est
bornée séparément : **≤ 19,5 ms** de temps de ligne au pire (≤ 23,1 ms si une
seconde ALARM la double), ~3,6 ms à vide. En fin de file (avant D8), elle
aurait été de 48,7 ms avec les trames STATS actuelles. L'**arrêt physique** du moteur a lieu à la réception de
l'OUTPUT(0) par l'actionneur : **≤ 100 ms + `R_ctrl` + temps de ligne**,
soit ~101 ms en nominal. Avant D7, le failsafe postait lui-même un OUTPUT(0)
(~1 ms en nominal). Le gain de D7 (écrivain unique, plus de course,
répétition toutes les 100 ms) se paie donc par un arrêt annoncé jusqu'à un
cycle plus tard.

### 6.1 bis Risque résiduel de l'écrivain unique (D7)

`task_control` relit le mode et fixe la commande dans **une seule** opération
atomique, `ecu_state_commit_output`, juste avant la mise en queue. Un
failsafe survenu pendant le calcul du PID est donc vu à ce moment-là et
force 0.

Il reste une fenêtre entre `commit_output` et `xQueueSend`. Si le failsafe
passe exactement là, sur l'autre cœur ou par préemption à priorité 5, **au
plus une** trame OUTPUT non nulle déjà validée part après le passage en OFF,
voire après l'ALARM. Elle est remplacée par OUTPUT(0) au cycle suivant, soit
**≤ 100 ms**. La commande non nulle ne peut pas persister : OUTPUT est émis
à chaque cycle, et l'état OFF est maintenu par le verrou (GPIO) ou jusqu'au
prochain MODE_SET (UART).

### 6.2 Rebonds GPIO

**Avant D2** : `GPIO_INTR_ANYEDGE`, sans anti-rebond. Chaque front d'une
rafale de rebonds (1 à 10 ms) relançait `trigger_failsafe` et postait
2 messages : `queue_tx` saturée d'ALARM redondantes, et le relâchement
re-déclenchait aussi le failsafe.

**Depuis D2** (contact NC, `GPIO_INTR_POSEDGE`, latch `failsafe_line.c`) :

- **une seule** ALARM par entrée en failsafe. Les fronts suivants réveillent
  encore la tâche (le sémaphore binaire fusionne les `Give`), mais tombent
  sur le latch : ~quelques µs par réveil, sans accès au mutex ni message ;
- le polling garde sa période de 100 ms même sous rafale (attente bornée
  par la prochaine échéance) ;
- le déverrouillage exige 2 polls consécutifs ligne fermée, donc les
  rebonds à la remise du jumper ne produisent ni déverrouillage prématuré ni
  nouvelle ALARM.

Interférence de priorité 5 sur control pendant une rafale : bornée par
`n_fronts × C_fs_latch`, avec `C_fs_latch` ≪ `C_fs`. ⏳ À mesurer avec la
sonde GPIO23 (§9) en retirant le jumper.

### 6.3 Chemin « silence UART »

Le polling a lieu toutes les 100 ms. La détection se produit donc entre
**2,0 et 2,1 s** (+1 tick) après la dernière trame valide. Seules les trames
**valides** (CRC + nonce acceptés) rafraîchissent `last_rx_tick` : un flux
de bruit ou de rejeux ne maintient pas l'ECU en vie. ✅

---

## 7. FMEA / AMDEC

Échelles : **G** = gravité (1 négligeable → 4 commande moteur dangereuse),
**O** = occurrence (1 improbable → 4 fréquent), **D** = non-détection
(1 détecté à coup sûr → 4 latent). **Criticité = G × O × D.**

| ID | Mode de défaillance | Effet | Détection | Mitigation en place | G | O | D | C | Risque résiduel / action |
|---|---|---|---|---|---|---|---|---|---|
| F-01 | Silence UART (câble, PC planté) | Dernière consigne maintenue | `task_failsafe`, timeout 2 s | Passage en OFF, OUTPUT(0), ALARM | 3 | 3 | 1 | 9 | 2,1 s au plus sur la dernière consigne. Acceptable pour le TP. |
| F-02 | Flood UART | `queue_frames` saturée (16), drop de la trame **la plus récente** | `rx_dropped` (STATS) | Parser non bloquant, buffer de 2048 o | 3 | 2 | 2 | 12 | ✅ **Traité** (`eb9c6d1`, R-6) pour MODE_SET : `queue_mode` écrasable, jamais perdu. Reste : SETPOINT/SPEED peuvent toujours être perdus sous flood (dernière valeur valide conservée, compté dans `rx_dropped`) |
| F-03 | Corruption de trame (bruit) | Valeur erronée appliquée | CRC-16/CCITT | Rejet silencieux, `rx_crc_error` | 4 | 2 | 1 | 8 | Erreurs ≤ 3 bits et rafales ≤ 16 bits toujours détectées. Probabilité résiduelle d'environ 2⁻¹⁶ pour un motif aléatoire au-delà. |
| F-04 | Rejeu d'une trame capturée | Commande périmée réappliquée | Nonce anti-rejeu | Rejet, `rx_replay` | 3 | 1 | 1 | 3 | Fenêtre ouverte après reboot (limitation documentée) |
| F-05 | Valeur **non finie** (NaN/Inf) avec un CRC valide | **Avant correctif** : intégrale du PID figée à NaN, OUTPUT = NaN jusqu'au prochain changement de mode | — | ✅ **Corrigé** (`8a54c7c`, puis `1ca0e2d`) : rejet par `protocol_parse_f32` + garde dans `pid_compute` (repli sur `out_min`) + tests unitaires | 4 | 1 | 1 | 4 | Défense en profondeur sur deux niveaux |
| F-06 | Valeur finie mais aberrante (setpoint = 10⁶) | PID saturé à 255 : pleine puissance | `rx_out_of_range` (STATS) | Bornes SETPOINT [0, 220], SPEED [0, 242] ; dernière valeur valide conservée | 4 | 1 | 1 | 4 | ✅ **Traité** (`0051fd1`, R-7). Reste : une valeur plausible mais fausse dans les bornes n'est pas détectable par l'ECU |
| F-07 | `queue_tx` pleine | OUTPUT non émis | `tx_dropped` (STATS) | Envoi non bloquant | 2 | 2 | 1 | 4 | — |
| F-08 | `queue_tx` pleine **au moment du failsafe** | ALARM perdue : arrêt non annoncé | `alarm_dropped` (STATS) | ALARM en tête de file (`SendToFront`) ; OUTPUT(0) répété à chaque cycle par task_control (une trame perdue est remplacée en ≤ 100 ms) | 3 | 1 | 1 | 3 | ✅ **Traité** (`e066822` pour OUTPUT, `4b79d62` pour l'ALARM). L'ALARM reste perdue si les 8 places sont occupées, mais c'est désormais compté |
| F-09 | Tâche bloquée (boucle infinie, attente sans fin) | Régulation ou supervision figée | ✅ **TWDT** (`0ce7e7e`) : les 5 tâches sont abonnées, timeout de 2 s | Panic → reboot → mode OFF au boot | 4 | 1 | 1 | 4 | ~2 s + temps de boot sans commande. L'état au boot est sûr. |
| F-10 | Débordement de pile | Corruption mémoire, comportement indéfini | Canary FreeRTOS (`CHECK_STACKOVERFLOW_CANARY`) | Panic → reboot | 4 | 1 | 2 | 8 | **Ouvert** : marges non mesurées → R-8 (⏳ sur carte) |
| F-11 | Deadlock du mutex | Tâches figées | TWDT | Impossible structurellement (§4.2) | 4 | 1 | 1 | 4 | — |
| F-12 | Inversion de priorité | Failsafe retardé | — | Héritage de priorité, `B_fs ≤ 3·C_cs` | 3 | 1 | 2 | 6 | — |
| F-13 | Bouton failsafe : fil coupé, contact défectueux | Failsafe GPIO **indisponible** (pull-up → ligne haute en permanence) | **Aucune** (défaut latent) | Le timeout UART reste actif | 4 | 2 | 4 | **32** | ✅ **Traité** (`233f002`, R-5) : contact NC, un fil coupé déclenche le failsafe. Niveau relu au boot et à chaque poll. Nouvelle cotation D = 1 → C = 8. ⏳ Validation sur carte |
| F-14 | Logs `ESP_LOG` sur UART0 | ASCII injecté dans le flux protocole, temps de blocage dans le cycle | Parser PC (resynchronisation sur 0xAA + CRC) | Protocole sur UART2, plus aucun log dans le cycle de `task_control` | 2 | 3 | 2 | 12 | ✅ **Traité** (`1ca0e2d`, R-3) : console et protocole séparés, rejets comptés (`rx_invalid`) |
| F-15 | Rebonds du bouton failsafe | Rafale de failsafes, `queue_tx` saturée | — | Sémaphore binaire (fusion des Give) | 1 | 3 | 2 | 6 | ✅ **Traité** (`233f002`) : latch `failsafe_line.c`, une ALARM par entrée en failsafe, déverrouillage après 2 polls fermés |
| F-16 | Coupure d'alimentation de l'ECU | Plus aucune commande émise | Côté superviseur ou véhicule seulement | Aucune côté ECU | 4 | 2 | 3 | 24 | **Axe 4** : l'ECU véhicule doit avoir son propre timeout CAN → moteur à 0 |
| F-17 | Dépassement de compteur | `uint32` : ~38 jours à 1 280 trames/s. Tick 32 bits à 100 Hz : 497 jours | — | Soustraction modulaire `now - last_tick` correcte | 1 | 1 | 2 | 2 | — |
| F-18 | Gigue > ±5 ms sous flood (§5) | EF-01 violée | ⏳ mesure | — | 2 | 2 | 3 | 12 | **Ouvert** : ⏳ mesure de `C_rx` ; R-1 (changement de priorité) hors périmètre de la session D1–D5 |
| F-19 | `MODE_SET(OFF)` hors failsafe (constaté en rédigeant D4) | Commande interne à 0, mais **aucune trame OUTPUT = 0 émise** : l'actionneur garde la dernière commande non nulle reçue | Testeur : écart max entre OUTPUT ; futur ECU moteur : coupure après 300 ms sans OUTPUT (EM-01) | OUTPUT émis à chaque cycle dans tous les modes, 0 hors AUTO | 4 | 1 | 1 | 4 | ✅ **Traité** (`e066822`, R-9) |
| F-20 | Course failsafe / `task_control` (constaté en rédigeant D4) | task_control, en plein cycle AUTO, poste son OUTPUT(pid) **après** l'OUTPUT(0) du failsafe (double cœur, ou failsafe qui préempte entre `get_mode` et `enqueue_output`). La dernière commande sur le bus est alors non nulle | — | Écrivain unique d'OUTPUT (task_control), `commit_output` atomique juste avant la mise en queue | 4 | 1 | 2 | 8 | ✅ **Traité** (`e066822`, R-9). Résiduel documenté (§6.1 bis) : au plus une trame non nulle déjà validée après le failsafe, remplacée par OUTPUT(0) en ≤ 100 ms |

### Recommandations

| Réf. | Recommandation | Couvre | Statut |
|---|---|---|---|
| R-1 | Rendre `task_control` **strictement** plus prioritaire que `task_rx` (par exemple rx et tx à 3). Le buffer RX de 2048 o absorbe 178 ms de flood alors que `C_ctrl` ≪ 1 ms : aucune perte. Le cas du §5 disparaît. À faire **après** la mesure, pour documenter l'avant/après. | F-18 | **Ouvert** : décision de priorité réservée, ⏳ mesure de `C_rx` |
| R-2 | — (numéro non attribué) | — | — |
| R-3 | Protocole sur UART2 (GPIO16/17), UART0 réservé à la console. Plus aucun log déclenchable par une trame reçue dans le cycle de `task_control` : compteurs à la place. | F-14, §4.3 | ✅ **Traité** — `1ca0e2d` |
| R-4 | (a) Redéfinir EF-05 en deux temps : état interne < 5 ms, annonce sur le bus bornée séparément. (b) Au failsafe, `xQueueSendToFront` pour OUTPUT(0) et ALARM + compteur `failsafe_tx_dropped`. | F-08, §6.1 | ✅ **Traité** : (a) `4cd8f02` ; (b) `4b79d62` pour l'ALARM (`SendToFront`, compteur `alarm_dropped`). L'OUTPUT(0) n'est plus posté par le failsafe : il est répété par task_control (`e066822`) |
| R-5 | Ligne failsafe en **contact NC**, `GPIO_INTR_POSEDGE`, latch + anti-rebond logiciel, niveau relu au boot et au polling, verrou AUTO tant que la ligne est ouverte, LED GPIO25. (La désactivation de l'interruption dans l'ISR, envisagée au départ, est remplacée par le latch : un réveil sur rebond coûte quelques µs sans effet.) | F-13, F-15 | ✅ **Traité** — `233f002` (⏳ validation sur carte) |
| R-6 | MODE_SET hors de `queue_frames` : `queue_mode` de longueur 1 écrite par `xQueueOverwrite`, lue en début de cycle | F-02 | ✅ **Traité** — `eb9c6d1` |
| R-7 | Bornes de plausibilité sur SETPOINT `[0, 220]` et SPEED `[0, 242]`, constantes nommées dans `protocol.h`, compteur `rx_out_of_range`. À réviser avec l'actionneur réel (axe 4). | F-06 | ✅ **Traité** — `0051fd1` |
| R-8 | (a) Nommage `STACK_*_BYTES`. (b) Mesurer `uxTaskGetStackHighWaterMark` pour chaque tâche (mode diagnostic, axe 5) et dimensionner les piles. | F-10 | **Partiel** : (a) ✅ `cc7d9a6` ; (b) ⏳ sur carte |
| R-9 | Écrivain unique d'OUTPUT : task_control émet OUTPUT à chaque cycle, dans tous les modes (0 hors AUTO), et relit le mode atomiquement juste avant la mise en queue. La variante « un seul OUTPUT(0) à la sortie d'AUTO » a été écartée : si cette trame est perdue, F-19 revient. | F-19, F-20 | ✅ **Traité** — `e066822` |

---

## 8. Watchdog matériel (axe 1, action 5) — ✅ implémenté

Le failsafe applicatif dépend des tâches elles-mêmes : si `task_failsafe` est
bloquée, plus rien ne supervise. Le TWDT (Task Watchdog Timer d'ESP-IDF)
couvre ce cas.

- `sdkconfig` : `CONFIG_ESP_TASK_WDT_PANIC=y` (reboot au lieu d'un simple
  log), `CONFIG_ESP_TASK_WDT_TIMEOUT_S=2`. Les tâches IDLE des deux cœurs
  restent surveillées (défaut).
- Chaque tâche s'abonne (`esp_task_wdt_add(NULL)`) et nourrit le watchdog à
  chaque itération (`esp_task_wdt_reset()`). Périodes de réveil maximales :

| Tâche | Réveil garanti au plus tard | Marge vs 2 s |
|---|---|---|
| `task_rx` | 50 ms (`PARSER_TIMEOUT_MS`) | ×40 |
| `task_control` | 100 ms | ×20 |
| `task_failsafe` | 100 ms (polling) | ×20 |
| `task_tx` | 500 ms (`TASK_TX_IDLE_WAKE_MS`, remplace `portMAX_DELAY`) | ×4 |
| `task_telemetry` | 1 s | ×2 |

Reboot → `ecu_state_init()` → **mode OFF, output 0** : l'état après reset
est l'état sûr. ⏳ À valider sur cible en injectant une boucle infinie dans
une tâche (test destructif, à faire volontairement).

---

## 9. Campagne de mesure à mener (axe 1, action 2)

Elle remplit les ⏳ ci-dessus. GPIO libres sur l'ESP32 #1 d'après
`brochage_materiel_GPIO.md` : **18, 19, 23, 26, 27**.

| Sonde | GPIO | Instrumentation | Donne |
|---|---|---|---|
| `task_control` | 18 | haut en début de cycle (après `vTaskDelayUntil`), bas après `enqueue_output` | `C_ctrl`, gigue de sortie |
| `task_rx` | 19 | haut/bas autour de `parser_feed_byte` | `C_rx` (critère < 86,8 µs du §5) |
| `task_failsafe` | 23 | haut à l'entrée de `trigger_failsafe` | latence front GPIO4 → état sûr (§6.1, étape 1) |

Scénarios, chacun sous charge nominale **et** sous flood (`ecu_tester.py`,
phase stress) :

1. Gigue de la période de `task_control` : histogramme sur au moins 10 000
   cycles, min/max/écart-type.
2. `C_rx` par octet sous flood, puis vérifier si rx est prête en continu.
3. Latence GPIO4 → GPIO23 (état interne) et GPIO4 → fin de trame OUTPUT(0)
   puis ALARM sur GPIO17 (annonce sur le bus, à comparer à la borne de 41 ms
   du §6.1).
4. ~~Cycle avec MODE_SET : impact des `ESP_LOGI`~~ — sans objet depuis D1
   (plus aucun log dans le cycle). À la place : latence réception MODE_SET →
   changement d'OUTPUT (≤ 100 ms attendus, §4.4).
5. Comparaison avant/après R-1.

Validations fonctionnelles sur carte (hors mesure de temps) :

6. Contact NC : retrait du jumper en marche (une seule ALARM, LED allumée,
   `MODE_SET(AUTO)` refusé et `mode_refused` qui monte), remise du jumper
   (déverrouillage, puis AUTO accepté), **boot avec le jumper déjà retiré**
   (ALARM « line open at boot »).
7. TWDT : boucle infinie injectée volontairement dans une tâche → reboot
   en ~2 s, redémarrage en OFF.
8. Stress test complet `ecu_tester.py` via l'adaptateur USB-TTL (UART2),
   avec MODE_SET(OFF) envoyé au milieu du flood : il doit être appliqué
   (§4.4). Le testeur doit afficher « SIGNAL DE VIE OK » (écart max entre
   OUTPUT ≤ 300 ms), et `rx_out_of_range` doit monter avec les SPEED
   aléatoires du flood.
9. Piles : `uxTaskGetStackHighWaterMark` relevé pour les 5 tâches après le
   stress test (marge restante en octets), puis dimensionnement des
   `STACK_*_BYTES` (R-8 b).
10. OUTPUT(0) sur la ligne après retrait du jumper : sur GPIO17, la première
    trame OUTPUT = 0 doit suivre le front GPIO4 dans le prochain cycle de
    task_control, soit **≤ 100 ms + temps de ligne** (~101 ms à vide,
    §6.1). Vérifier aussi qu'aucune trame non nulle ne suit cet OUTPUT(0)
    (§6.1 bis : au plus une, et seulement avant lui).

En complément : SEGGER SystemView via `esp_apptrace` pour l'historique
d'ordonnancement (préemptions, migrations entre cœurs).
