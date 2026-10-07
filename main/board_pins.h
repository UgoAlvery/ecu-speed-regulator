#ifndef ECU_BOARD_PINS_H
#define ECU_BOARD_PINS_H

/* Brochage de l'ESP32 #1 (ECU) — source unique, cf. brochage_materiel_GPIO.md.
 * Aucune autre unité de compilation ne doit coder un numéro de GPIO en dur. */

/* UART protocole (UART2), via adaptateur USB-TTL. UART0 reste à la console. */
#define UART_PROTO_TX_PIN   17
#define UART_PROTO_RX_PIN   16

/* Failsafe externe (entrée, pull-up interne) et LED d'état failsafe. */
#define FAILSAFE_GPIO_PIN   4
#define FAILSAFE_LED_PIN    25

/* Bus CAN (TWAI + SN65HVD230) — réservées, non utilisées pour l'instant. */
#define TWAI_TX_PIN         21
#define TWAI_RX_PIN         22

#endif /* ECU_BOARD_PINS_H */
