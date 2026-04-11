#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "can_node.h"
#include "can_esc.h"

// ============================================================
//  RTOS handles — private to main.c
// ============================================================

static TaskHandle_t      s_rx_task_handle;
static SemaphoreHandle_t s_canard_mutex;
static SemaphoreHandle_t s_tx_sem;   // counting, signals TX task

// ============================================================
//  Notify hooks — these are what the library calls
// ============================================================

// Called from ISR context by can_node_rx_isr
static void notify_rx_task_from_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
    vTaskNotifyGiveFromISR(s_rx_task_handle, pxHigherPriorityTaskWoken);
}

// Called from task context by can_node_1hz_tasks / can_node_poll_dna
static void signal_tx_task(void)
{
    xSemaphoreGive(s_tx_sem);
}

// ============================================================
//  HAL ISR override — HAL calls this automatically
// ============================================================

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    BaseType_t woken = can_node_rx_isr(hfdcan);
    portYIELD_FROM_ISR(woken);
}

// ============================================================
//  Tasks
// ============================================================

static void rx_task(void *arg)
{
    (void)arg;
    for (;;) {
        // Block until ISR notifies us
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(s_canard_mutex, portMAX_DELAY);
        while (can_node_dequeue_and_process()) {}  // drain all queued frames
        can_node_poll_dna();
        xSemaphoreGive(s_canard_mutex);

        xSemaphoreGive(s_tx_sem);  // flush any response frames
    }
}

static void tx_task(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_tx_sem, portMAX_DELAY);

        xSemaphoreTake(s_canard_mutex, portMAX_DELAY);
        can_node_flush_tx();
        xSemaphoreGive(s_canard_mutex);
    }
}

static void node_1hz_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));

        xSemaphoreTake(s_canard_mutex, portMAX_DELAY);
        can_node_1hz_tasks();   // internally calls s_tx_ready_fn → signal_tx_task
        xSemaphoreGive(s_canard_mutex);
    }
}

static void esc_status_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));  // 10 Hz

        xSemaphoreTake(s_canard_mutex, portMAX_DELAY);
        send_esc_status();
        can_node_flush_tx();    // flush immediately — no need to wake separate task
        xSemaphoreGive(s_canard_mutex);
    }
}

// ============================================================
//  Startup
// ============================================================

void app_init(FDCAN_HandleTypeDef *hfdcan)
{
    s_canard_mutex = xSemaphoreCreateMutex();
    s_tx_sem       = xSemaphoreCreateCounting(32, 0);
    configASSERT(s_canard_mutex && s_tx_sem);

    // Wire notify hooks before starting the peripheral
    can_node_set_rx_notify(notify_rx_task_from_isr);
    can_node_set_tx_ready(signal_tx_task);

    can_node_init(hfdcan);
    set_all_to_zero();

    xTaskCreate(rx_task,         "CAN_RX",   512, NULL, 5, &s_rx_task_handle);
    xTaskCreate(tx_task,         "CAN_TX",   256, NULL, 4, NULL);
    xTaskCreate(node_1hz_task,   "CAN_1HZ",  256, NULL, 3, NULL);
    xTaskCreate(esc_status_task, "ESC_STAT", 256, NULL, 3, NULL);

    // Note: s_rx_task_handle is valid immediately after xTaskCreate returns.
    // The ISR won't fire until HAL_FDCAN_Start inside can_node_init,
    // so there is no race here.
}