# Roadmap — Du TP ECU au projet personnel

> Document de pilotage. Objectif : transformer un TP validé (architecture FreeRTOS,
> hardware bring-up réussi, rapport rendu) en projet personnel présentable — CV,
> portfolio, entretiens techniques.
>
> Principe directeur : chaque axe doit ajouter une **preuve** (mesure, test automatisé,
> document d'analyse), pas juste une fonctionnalité de plus. Un projet qui "fait plus de
> choses" est moins convaincant qu'un projet dont chaque affirmation est démontrée.

---

## Vue d'ensemble des axes

| # | Axe | Valeur ajoutée | Effort | Dépendances |
|---|-----|-----------------|--------|--------------|
| 1 | Sûreté & analyse temps réel (WCET, RTA, FMEA) | Très haute (cœur du profil) | Moyen | Aucune |
| 2 | Robustesse protocolaire (CRC16, watchdog HW) | Haute | Faible | Aucune |
| 3 | Tests automatisés (unitaires, fuzzing, CI) | Haute | Moyen | Axe 2 (CRC change les tests existants) |
| 4 | Extension matérielle (vrai CAN, capteur/actionneur réel) | Haute (démonstrateur) | Élevé | Axes 1–2 stabilisés d'abord |
| 5 | Observabilité (tracing, dashboard) | Moyenne | Moyen | Peut se faire en parallèle de 3–4 |
| 6 | Documentation & valorisation (safety case, README) | Haute (c'est ce qu'on lit en premier) | Faible-Moyen | Se nourrit de tous les axes précédents, donc en continu + finalisation en dernier |

**Logique de priorisation** : on renforce d'abord ce qui existe (sûreté, robustesse, tests)
avant d'étendre le périmètre (hardware réel). Un projet solide sur un scope maîtrisé vaut
mieux qu'un projet qui s'étale mais dont les bases ne sont pas prouvées. La documentation
se construit en continu (chaque mesure/test alimente le rapport) mais se finalise à la fin.

---

## Axe 1 — Sûreté & analyse temps réel (priorité 1)

**Pourquoi en premier** : c'est l'axe qui distingue un "TP qui marche" d'un "ingénieur
systèmes temps réel qui sait le prouver". C'est aussi celui qui est le plus cohérent avec
un profil embarqué industriel/sûreté.

### Actions concrètes

1. **Test failsafe GPIO manquant** — encore en attente (jumper wire manquant lors de la
   dernière session hardware). À faire en premier, c'est la seule exigence fonctionnelle
   du TP pas encore validée empiriquement.
2. **WCET mesuré** (pas estimé) :
   - Instrumentation GPIO toggle en entrée/sortie de `task_control`, capture à
     l'analyseur logique, sous flood UART actif.
   - Alternative/complément : SEGGER SystemView via `esp_apptrace` (natif ESP-IDF) pour
     avoir l'historique d'ordonnancement complet, pas juste un pire cas ponctuel.
3. **RTA formelle** sur les 5 tâches :
   - Modéliser `task_rx`, `task_tx`, `task_failsafe` comme sporadiques (temps
     d'inter-arrivée minimal, pas une période fictive).
   - Calculer le blocking term introduit par `mutex_ecu_state` (héritage de priorité) —
     le chiffrer, pas juste affirmer qu'il est activé.
   - Vérifier et documenter formellement le cas `task_rx` / `task_control` à priorité
     égale (config `configUSE_TIME_SLICING`) — actuellement une hypothèse non prouvée.
4. **FMEA/AMDEC** — table modes de défaillance → effet → détection → mitigation →
   risque résiduel (voir squelette déjà esquissé : silence UART, flood, collision CRC,
   stack overflow, deadlock mutex, coupure alimentation, GPIO non testé).
5. **Watchdog matériel** (Task Watchdog Timer ESP-IDF) en complément du failsafe
   applicatif — actuellement le failsafe dépend du bon fonctionnement des tâches
   elles-mêmes ; un TWDT couvre le cas où une tâche est totalement bloquée.

**Livrable de fin d'axe** : document d'analyse temps réel + sûreté (peut être une section
dédiée du rapport, ou un document séparé type "safety case").

---

## Axe 2 — Robustesse protocolaire (priorité 2)

**Pourquoi tôt** : ce sont des changements structurels sur `protocol.c`. Les faire avant
d'écrire les tests unitaires (axe 3) évite de devoir les réécrire.

### Actions concrètes

1. **CRC16 (ou CRC8 polynomial)** à la place du XOR simple — documenter explicitement
   pourquoi le XOR du TP est insuffisant (n'importe quelle erreur à nombre pair de bits
   inversés passe inaperçue).
2. **Authentification légère des trames** (HMAC tronqué ou simple nonce/compteur
   anti-rejeu) — pertinent pour un profil sécurité embarquée, montre que tu penses
   au-delà de la robustesse fonctionnelle.
3. Mettre à jour `technical_reference.md` et le script de test Python en conséquence.

**Livrable de fin d'axe** : `protocol.c` v2 + note de justification des choix crypto/CRC.

---

## Axe 3 — Tests automatisés (priorité 3)

**Pourquoi c'est le vrai delta "projet perso" vs "TP"** : un TP est validé une fois par un
enseignant. Un projet perso doit se valider tout seul, en continu.

### Actions concrètes

1. **Tests unitaires hébergés** (sur machine de dev, pas sur cible) pour les modules libs
   pures : `protocol.c` et `pid.c`. Unity (natif ESP-IDF) ou GoogleTest si tu veux rester
   proche de ton usage C++ habituel — nécessite d'isoler ces fichiers de toute dépendance
   FreeRTOS/ESP-IDF (déjà annoncé comme "lib pure sans état global", donc faisable sans
   trop de refactor).
2. **Fuzzing de `protocol_decode`** — c'est la fonction qui parse un flux non fiable,
   candidate naturelle. AFL ou libFuzzer, compilé nativement (host).
3. **CI GitHub Actions** : build firmware ESP-IDF + exécution des tests hébergés à chaque
   push. Le repo `ecu-speed-regulator` existe déjà, donc juste ajouter le workflow.
4. **Automatisation du test hardware-in-the-loop** — le script Python de stress test
   existe déjà manuellement ; l'intégrer dans une routine reproductible (rapport de test
   généré automatiquement : jitter mesuré, taux de trames droppées, etc.) plutôt qu'une
   lecture visuelle de la console série.

**Livrable de fin d'axe** : badge CI dans le README + rapport de couverture de test.

---

## Axe 4 — Extension matérielle (priorité 4)

**Pourquoi après, pas avant** : c'est le plus gros effort et le plus gros risque de
régression. Le faire une fois le cœur du système durci évite de "prouver la sûreté" sur
une base qui va encore bouger.

### Actions concrètes, par ordre de complexité croissante

1. **Capteur/actionneur réel** — remplacer la vitesse simulée par un encodeur réel et
   l'OUTPUT par un vrai moteur DC + driver (H-bridge). Change la nature du projet :
   "simulation logicielle" → "boucle de contrôle physique réelle".
2. **Vrai bus CAN** via le contrôleur TWAI intégré à l'ESP32, entre deux cartes —
   transforme le "protocole inspiré du CAN, simulé sur UART" en vrai réseau
   multi-ECU. C'est l'extension la plus démonstrative pour un profil automobile/industriel.
3. **Dashboard web léger** (ESP32 en AP + petit serveur HTTP) affichant la télémétrie en
   direct — optionnel, surtout utile si tu veux une démo visuelle facile à montrer.

**Livrable de fin d'axe** : vidéo/démo du système physique + mise à jour de l'architecture
documentée (deux ECU, bus CAN réel).

---

## Axe 5 — Observabilité (transverse, peut se faire en continu)

- SystemView / tracing FreeRTOS (déjà mentionné en axe 1 pour le WCET, réutilisable ici
  pour du monitoring continu).
- Mode diagnostic (déjà identifié précédemment) : commande dédiée qui dump l'état
  interne (compteurs, stack high-water-mark de chaque tâche, dernier tick RX) sur
  demande, utile en debug comme en démo.

---

## Axe 6 — Documentation & valorisation (continu + finalisation)

- **Pendant** chaque axe : noter les résultats au fur et à mesure (mesures WCET, résultats
  FMEA, résultats CI) plutôt que de tout rédiger à la fin de mémoire.
- **À la fin** :
  - README avec résultats de tests réels, schéma matériel, éventuellement diagramme de
    séquence (Mermaid) du déclenchement failsafe.
  - Document "safety case" séparé du rapport de TP — rare chez les étudiants, très
    parlant pour un profil sûreté/sécurité en entretien.
  - Nettoyage final du historique Git / structure repo si besoin (cohérent avec ta
    règle déjà en place : `sdkconfig` versionné, `build/` ignoré).

---

## Plan d'action synthétique

| Phase | Contenu | Sortie |
|-------|---------|--------|
| **Phase 0** | Test failsafe GPIO manquant | Exigence TP 100 % validée |
| **Phase 1** | WCET mesuré + RTA formelle + FMEA + watchdog HW | Document sûreté / temps réel |
| **Phase 2** | CRC16 + authentification légère | `protocol.c` v2 |
| **Phase 3** | Tests unitaires hébergés + fuzzing + CI | Pipeline de validation automatique |
| **Phase 4** | Extension hardware (capteur/actionneur réel, puis CAN réel) | Démonstrateur physique |
| **Phase 5** | Observabilité (tracing, mode diagnostic) | Outils de monitoring |
| **Phase 6** | Finalisation documentation (README, safety case) | Projet présentable |

Chaque phase est indépendamment "montrable" — si tu t'arrêtes après la Phase 1 ou 3,
le projet a déjà gagné en crédibilité par rapport au TP initial. Ça permet d'avancer par
sessions sans dépendre de finir tout le plan pour que ça vaille le coup.
