# Risque matériel : broche PWM flottante au démarrage

## Problème

Pendant le boot et le reset de l'ESP32, toutes les GPIO sont en haute impédance
jusqu'à ce que le pilote GPIO soit initialisé. `MOTOR_PWM_PIN` (GPIO27) est donc
flottant **avant** que `hal_pwm_init()` soit appelé.

Le transistor GT1168 (IRF520 ou équivalent) est un MOSFET canal N : sa grille
est sensible aux perturbations. Une tension positive transitoire sur la grille
(couplage capacitif, boucle induite par le câblage) peut entraîner une brève
ouverture du canal et une mise en rotation involontaire du moteur.

## Mitigation obligatoire

Placer une résistance **10 kΩ entre la grille du GT1168 et GND**.

```
ESP32 GPIO27 ──[100 Ω]──┬── Grille GT1168
                         │
                        [10 kΩ]
                         │
                        GND
```

- La résistance série (100 Ω) limite le courant de charge de la capacité de grille
  et amortit les oscillations HF.
- La résistance de pull-down (10 kΩ) maintient Vgs ≈ 0 V tant que l'ESP32 ne
  pilote pas activement la broche.

## Comportement logiciel

`hal_pwm_init()` configure LEDC avec duty = 0, ce qui maintient GPIO27 à l'état
bas **après** l'initialisation. Sans le pull-down, il n'y a aucune garantie
matérielle pendant la fenêtre de boot (typiquement 50–200 ms).

`hal_pwm_enable()` requiert `HW_PROTECTION_CONFIRMED = true` : ce flag doit être
positionné uniquement après avoir vérifié physiquement la présence du pull-down,
du fusible de protection et de la diode de roue libre (1N5822).

## Chemin de sûreté résumé

| Événement | Action software | Action hardware |
|---|---|---|
| Boot / reset | LEDC duty = 0 après `hal_pwm_init()` | Pull-down 10 kΩ force Vgs = 0 |
| Failsafe GPIO | `hal_pwm_disable()` → duty = 0 | Pull-down maintient l'état |
| Silence UART (> 2 s) | `hal_pwm_disable()` → duty = 0 | Pull-down maintient l'état |
| Heartbeat figé (> 500 ms) | `hal_pwm_disable()` via `task_failsafe` | Pull-down maintient l'état |
| Garde silence ctrl (> 1500 ms) | `hal_pwm_disable()` secondaire si task_control tourne | Pull-down maintient l'état |

> **Priorités des tâches** : `task_failsafe` (5) > `task_control` (4). Un blocage de
> `task_control` ne peut pas bloquer `task_failsafe`. La garde heartbeat dans
> `task_failsafe` détecte le gel de `task_control` en ≤ 500 ms.
