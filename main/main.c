#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "ecu_state.h"
#include "protocol.h"
#include "task_rx.h"
#include "task_tx.h"
#include "task_control.h"
#include "task_telemetry.h"
#include "failsafe.h"

#define QUEUE_FRAMES_DEPTH   16u
#define QUEUE_TX_DEPTH       8u
#define QUEUE_MODE_DEPTH     1u   /* xQueueOverwrite exige une longueur de 1 */
#define TASK_FAILSAFE_PRIORITY   5u
#define TASK_RX_PRIORITY         4u
#define TASK_CONTROL_PRIORITY    4u
#define TASK_TX_PRIORITY         3u
#define TASK_TELEMETRY_PRIORITY  2u

/* ESP-IDF : la profondeur de pile passée à xTaskCreate est en OCTETS (et non
 * en mots comme dans le FreeRTOS vanilla). */
#define STACK_FAILSAFE_BYTES    2048u
#define STACK_RX_BYTES          4096u
#define STACK_CONTROL_BYTES     4096u
#define STACK_TX_BYTES          4096u
#define STACK_TELEMETRY_BYTES   2048u

static const char *TAG = "main";

static QueueHandle_t     s_queue_frames = NULL;
static QueueHandle_t     s_queue_tx     = NULL;
static QueueHandle_t     s_queue_mode   = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "ECU boot — initialisation");
    ecu_state_init();
    s_queue_frames = xQueueCreate(QUEUE_FRAMES_DEPTH, sizeof(ecu_frame_t));
    configASSERT(s_queue_frames != NULL);

    s_queue_tx = xQueueCreate(QUEUE_TX_DEPTH, sizeof(tx_message_t));
    configASSERT(s_queue_tx != NULL);

    s_queue_mode = xQueueCreate(QUEUE_MODE_DEPTH, sizeof(ecu_mode_t));
    configASSERT(s_queue_mode != NULL);

    task_rx_init(s_queue_frames, s_queue_mode);
    task_tx_init(s_queue_tx);
    task_control_init(s_queue_frames, s_queue_mode, s_queue_tx);
    task_telemetry_init(s_queue_tx);
    failsafe_init(s_queue_tx);

    BaseType_t ret = xTaskCreate(task_failsafe,
                      "task_failsafe",
                      STACK_FAILSAFE_BYTES,
                      NULL,
                      TASK_FAILSAFE_PRIORITY,
                      NULL);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(task_rx,
                      "task_rx",
                      STACK_RX_BYTES,
                      NULL,
                      TASK_RX_PRIORITY,
                      NULL);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(task_control,
                      "task_control",
                      STACK_CONTROL_BYTES,
                      NULL,
                      TASK_CONTROL_PRIORITY,
                      NULL);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(task_tx,
                      "task_tx",
                      STACK_TX_BYTES,
                      NULL,
                      TASK_TX_PRIORITY,
                      NULL);
    configASSERT(ret == pdPASS);

    ret = xTaskCreate(task_telemetry,
                      "task_telemetry",
                      STACK_TELEMETRY_BYTES,
                      NULL,
                      TASK_TELEMETRY_PRIORITY,
                      NULL);
    configASSERT(ret == pdPASS);

    ESP_LOGI(TAG, "Toutes les taches creees — scheduler actif");
}