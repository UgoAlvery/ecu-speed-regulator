# ECU – Brochage matériel (GPIO)

Oct 7, 2026 · @Ugo ALVERY

## Vue d'ensemble

Deux ESP32 DevKit 30 broches, reliés par un bus CAN (TWAI intégré + transceiver SN65HVD230 sur chaque carte).

| Carte | Rôle | Alimentation |
| --- | --- | --- |
| ESP32 #1 | ECU (PID, failsafe, télémétrie) | USB PC |
| ESP32 #2 | Véhicule (PWM moteur, lecture encodeur) | USB PC |
| Moteur FIT0522 | Actionneur | Bloc secteur 6 V / 3 A, via le module MOSFET GT1168 uniquement |

Masse commune obligatoire : GND ESP32 #1, GND ESP32 #2, GND des deux SN65HVD230, GND adaptateur USB-TTL, G encodeur, GND logique du GT1168. Le 6 V moteur n'est jamais relié aux broches 3V3/5V/VIN des ESP32.

## ESP32 #1 — ECU

| GPIO | Direction | Vers | Usage firmware |
| --- | --- | --- | --- |
| GPIO21 | Sortie | CTX / TXD du SN65HVD230 #1 | TWAI TX |
| GPIO22 | Entrée | CRX / RXD du SN65HVD230 #1 | TWAI RX |
| GPIO4 | Entrée | Jumper permanent vers GND (contact normalement fermé) | Failsafe externe, pull-up interne. Repos = 0 ; jumper retiré ou fil coupé = 1 → front montant, ISR |
| GPIO25 | Sortie | Résistance 330 Ω → LED → GND | LED d'état failsafe |
| GPIO16 | Entrée | TX de l'adaptateur USB-TTL | UART protocole (RX), optionnel : injection et stress test Python |
| GPIO17 | Sortie | RX de l'adaptateur USB-TTL | UART protocole (TX), optionnel |
| 3V3 | Alim | VCC du SN65HVD230 #1 | 3,3 V uniquement |
| GND | Masse | SN65HVD230 #1, jumper failsafe, LED, adaptateur USB-TTL | Masse commune |

La console `idf.py monitor` reste sur le port USB natif de la carte, distinct du lien GPIO16/17.

## ESP32 #2 — Véhicule

| GPIO | Direction | Vers | Usage firmware |
| --- | --- | --- | --- |
| GPIO21 | Sortie | CTX / TXD du SN65HVD230 #2 | TWAI TX |
| GPIO22 | Entrée | CRX / RXD du SN65HVD230 #2 | TWAI RX |
| GPIO25 | Sortie | SIG du GT1168 | PWM moteur (LEDC), unidirectionnel |
| GPIO32 | Entrée | A de l'encodeur | PCNT canal A, pull-up interne |
| GPIO33 | Entrée | B de l'encodeur | PCNT canal B (quadrature), pull-up interne |
| 3V3 | Alim | VCC SN65HVD230 #2, VCC logique GT1168, V encodeur | Encodeur en 3,3 V : sorties A/B compatibles ESP32 (à confirmer sur la fiche DFRobot) |
| GND | Masse | SN65HVD230 #2, GND logique GT1168, G encodeur | Masse commune |

GPIO34 à 39 sont à éviter pour l'encodeur : entrée seule, sans pull-up interne.

## Bus CAN et chemin de puissance

Bus CAN : CANH ↔ CANH et CANL ↔ CANL entre les deux SN65HVD230 (fils torsadés). Terminaison 120 Ω activée sur les deux modules.

Chemin de puissance, entièrement hors breadboard (bornes à vis) :

| De | Vers |
| --- | --- |
| Bloc 6 V (centre +), cordon jack 48079, fil + | VIN du GT1168 |
| Cordon jack 48079, fil − | GND puissance du GT1168 |
| M1 du moteur | V+ du GT1168 |
| M2 du moteur | V− du GT1168 |
| Diode 1N4007 | En parallèle sur V+ / V−, cathode (bague) côté V+ |

Libellés VIN / GND / V+ / V− typiques des modules IRF520 : à confirmer sur la sérigraphie du GT1168.

## Contraintes firmware

1. Centraliser les broches dans des `#define` (un header par carte), avec les noms de ce document : `TWAI_TX_PIN 21`, `TWAI_RX_PIN 22`, `FAILSAFE_GPIO_PIN 4`, `FAILSAFE_LED_PIN 25`, `UART_PROTO_TX_PIN 17`, `UART_PROTO_RX_PIN 16` (ECU, `main/board_pins.h`) ; `MOTOR_PWM_PIN 25`, `ENC_A_PIN 32`, `ENC_B_PIN 33` (véhicule).
2. Failsafe GPIO4 en contact normalement fermé (NC) : pull-up interne, interruption sur front montant (ouverture). Un fil coupé déclenche le failsafe au lieu de le désactiver. Pas de matériel anti-rebond : déclencher sur le premier front, émettre l'ALARM uniquement sur la transition vers l'état failsafe, ignorer les fronts suivants. Niveau relu au boot et à chaque polling (100 ms) pour détecter une ligne déjà ouverte. Tant que la ligne est ouverte, MODE_SET(AUTO) est refusé ; LED GPIO25 allumée tant que le failsafe est actif.
3. PWM moteur : forcé à 0 dès l'initialisation, avant toute autre tâche. Séquence d'alimentation : ESP32 d'abord, bloc 6 V ensuite.
4. Encodeur : PCNT en quadrature sur GPIO32/33, pull-ups internes activés. Conversion en vitesse via le PPR et le rapport de réduction du FIT0522 (fiche DFRobot).
5. Point ouvert : l'IRF520N du GT1168 n'est pas un vrai MOSFET logic-level. À PWM 100 %, vérifier une tension moteur proche de 6 V et l'absence d'échauffement.

