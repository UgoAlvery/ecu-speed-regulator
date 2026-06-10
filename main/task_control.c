#include "task_control.h"

#include <string.h>

#include "ecu_state.h"
#include "pid.h"
#include "protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"


#define TAG               "CTRL"
#define CONTROL_PERIOD_MS 100u
#define TX_SEND_TIMEOUT   pdMS_TO_TICKS(10)

static QueueHandle_t s_rx_queue = NULL;
static QueueHandle_t s_tx_queue = NULL;

static void dispatch_pending_frames(void);
static void dispatch_frame(const ecu_frame_t *frame);
static void run_pid_and_emit(pid_t *pid);
static bool enqueue_output(float output);


void task_control_init(const QueueHandle_t rx_queue, const QueueHandle_t tx_queue)
{
    configASSERT(rx_queue != NULL);
    configASSERT(tx_queue != NULL);
    s_rx_queue = rx_queue;
    s_tx_queue = tx_queue;
}

void task_control(void *pvParameters)
{
    (void)pvParameters;

    configASSERT(s_rx_queue != NULL);
    configASSERT(s_tx_queue != NULL);

    pid_t pid;
    pid_init(&pid,
             PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
             PID_DEFAULT_DT, PID_DEFAULT_OUT_MIN, PID_DEFAULT_OUT_MAX);

    ecu_mode_t prev_mode = ecu_state_get_mode();

    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "task_control démarrée, période %u ms", CONTROL_PERIOD_MS);

    for (;;) {
        dispatch_pending_frames();

        const ecu_mode_t mode = ecu_state_get_mode();
        if (mode != prev_mode) {
            if (mode != ECU_MODE_AUTO) {
                ecu_state_set_output(0.0f);
                pid_reset(&pid);
                ESP_LOGI(TAG, "Transition mode %d→%d : output=0, PID reset",
                         prev_mode, mode);
            } else {
                pid_reset(&pid);
                ESP_LOGI(TAG, "Entrée en mode AUTO : PID reset");
            }
            prev_mode = mode;
        }

        if (mode == ECU_MODE_AUTO) {
            run_pid_and_emit(&pid);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Fonctions internes
 * ══════════════════════════════════════════════════════════════════════════ */

static void dispatch_pending_frames(void)
{
    ecu_frame_t frame;
    while (xQueueReceive(s_rx_queue, &frame, 0) == pdTRUE) {
        dispatch_frame(&frame);
    }
}


static void dispatch_frame(const ecu_frame_t *frame)
{
    switch (frame->type) {

    case MSG_SETPOINT: {
        if (frame->payload_len < sizeof(float)) {
            ESP_LOGW(TAG, "SETPOINT : payload trop court (%u B)", frame->payload_len);
            break;
        }
        float val;
        memcpy(&val, frame->payload, sizeof(float));
        ecu_state_set_setpoint(val);
        ESP_LOGD(TAG, "SETPOINT = %.2f", val);
        break;
    }

    case MSG_SPEED: {
        if (frame->payload_len < sizeof(float)) {
            ESP_LOGW(TAG, "SPEED : payload trop court (%u B)", frame->payload_len);
            break;
        }
        float val;
        memcpy(&val, frame->payload, sizeof(float));
        ecu_state_set_speed(val);
        ESP_LOGD(TAG, "SPEED = %.2f", val);
        break;
    }

    case MSG_MODE_SET: {
        if (frame->payload_len < sizeof(uint8_t)) {
            ESP_LOGW(TAG, "MODE_SET : payload trop court (%u B)", frame->payload_len);
            break;
        }
        const uint8_t raw_mode = frame->payload[0];
        if (raw_mode > ECU_MODE_AUTO) {
            ESP_LOGW(TAG, "MODE_SET : valeur inconnue 0x%02X, ignorée", raw_mode);
            break;
        }
        ecu_state_set_mode((ecu_mode_t)raw_mode);
        ESP_LOGI(TAG, "MODE_SET → %u", raw_mode);
        break;
    }

    default:
        ESP_LOGD(TAG, "Type 0x%02X ignoré en dispatch", frame->type);
        break;
    }
}


static void run_pid_and_emit(pid_t *pid)
{
    const float setpoint = ecu_state_get_setpoint();
    const float speed    = ecu_state_get_speed();
    const float output   = pid_compute(pid, setpoint, speed);

    ecu_state_set_output(output);

    if (!enqueue_output(output)) {
        ESP_LOGW(TAG, "queue_tx pleine — OUTPUT non émis");
    }
}


static bool enqueue_output(const float output)
{
    static uint8_t frame_buf[PROTOCOL_MAX_FRAME_SIZE];

    uint8_t payload[sizeof(float)];
    memcpy(payload, &output, sizeof(float));

    const size_t frame_len = protocol_encode(frame_buf, MSG_OUTPUT,
                                       payload, (uint16_t)sizeof(float));
    if (frame_len == 0) {
        ESP_LOGE(TAG, "protocol_encode a échoué pour OUTPUT");
        return false;
    }

    typedef struct {
        uint8_t data[PROTOCOL_MAX_FRAME_SIZE];
        size_t  len;
    } tx_message_t;

    tx_message_t msg;
    memcpy(msg.data, frame_buf, frame_len);
    msg.len = frame_len;

    return xQueueSend(s_tx_queue, &msg, TX_SEND_TIMEOUT) == pdTRUE;
}