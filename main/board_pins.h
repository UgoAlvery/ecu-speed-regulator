#ifndef ECU_BOARD_PINS_H
#define ECU_BOARD_PINS_H

/* GPIO map for ESP32 #1 (ECU) — NodeMCU 38-pin breakout.
 * This is the single source of truth for all pin numbers.
 * No other translation unit shall hard-code a GPIO number.
 *
 * Rules applied:
 *   - Strapping pins avoided for driven outputs: GPIO0, 2, 5, 12, 15.
 *   - Internal flash pins forbidden: GPIO6–11.
 *   - Input-only pins (GPIO34–39) used only for inputs with no pull requirement.
 *   - GPIO21/22 reserved for TWAI; I2C uses GPIO18/19 (not 21/22).
 *     NOTE: GPIO25 is the failsafe LED — do NOT use it for I2C SDA. */

/* ── UART protocol (UART2) — USB-TTL adapter on UART2.
 *    UART0 stays for ESP_LOG console (idf.py monitor). */
#define UART_PROTO_TX_PIN   17
#define UART_PROTO_RX_PIN   16

/* ── Failsafe: normally-closed contact and status LED. */
#define FAILSAFE_GPIO_PIN   4
#define FAILSAFE_LED_PIN    25   /* GPIO25 — do NOT reuse for I2C/SPI/etc. */

/* ── CAN bus (TWAI + SN65HVD230 transceiver, 500 kbit/s).
 *    Driver not yet enabled; pins claimed to prevent conflict. */
#define TWAI_TX_PIN         21
#define TWAI_RX_PIN         22

/* ── Motor PWM (LEDC channel → GT1168 gate drive).
 *    TODO(datasheet): confirm GT1168 (likely IRF520, non logic-level)
 *    conduction at Vgs = 3.3 V; if Rds(on) too high, add a gate driver. */
#define MOTOR_PWM_PIN       27

/* ── Quadrature encoder A/B (PCNT unit 0, FIT0522).
 *    Both GPIOs are bidirectional (output-capable) so internal pull-ups work.
 *    TODO(datasheet): confirm FIT0522 encoder output voltage level (3.3 V?). */
#define ENCODER_A_PIN       32
#define ENCODER_B_PIN       33

/* ── I2C bus (INA219 current sensor, address 0x40).
 *    GPIO18/19 chosen because GPIO25 is taken by FAILSAFE_LED and GPIO21/22
 *    are taken by TWAI.  Any free GPIO works for ESP32 software I2C. */
#define I2C_SDA_PIN         18
#define I2C_SCL_PIN         19

#endif /* ECU_BOARD_PINS_H */
