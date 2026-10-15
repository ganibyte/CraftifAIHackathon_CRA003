#include "board.h"
#include <stdbool.h>
#include "sdkconfig.h"
#include "app_events.h"
#include "wt_config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"

static const char *TAG = "board";
#define LED_GPIO ((gpio_num_t)CONFIG_WT_PIN_LED)
#define BTN_GPIO ((gpio_num_t)CONFIG_WT_PIN_BTN)

volatile int64_t g_evt_btn_us;

static esp_timer_handle_t s_hb, s_hb_off, s_blink;
static TaskHandle_t s_notify, s_btn_task;
static int s_led_level;

static void led(int on) { s_led_level = on; gpio_set_level(LED_GPIO, on); }

/* Heartbeat: 20 ms pulse every 5 s (FR-19). Timers fire from the esp_timer task, no busy-wait. */
static void hb_cb(void *a) { led(1); esp_timer_start_once(s_hb_off, 20 * 1000); }
static void hb_off_cb(void *a) { led(0); }
static void blink_cb(void *a) { led(!s_led_level); }     /* toggle every 100 ms = 5 Hz */

void board_led_set_mode(led_mode_t m)
{
    esp_timer_stop(s_hb); esp_timer_stop(s_hb_off); esp_timer_stop(s_blink);
    switch (m) {
    case LED_MODE_NORMAL:
        led(0);
        esp_timer_start_periodic(s_hb, 5ULL * 1000 * 1000);
        hb_cb(NULL);
        break;
    case LED_MODE_ALERT:
        led(1);                                             /* solid on */
        break;
    case LED_MODE_FAULT:
        led(1);
        esp_timer_start_periodic(s_blink, 100 * 1000);      /* 5 Hz blink */
        break;
    }
}

static void IRAM_ATTR btn_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(s_btn_task, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void btn_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);             /* any edge */
        vTaskDelay(pdMS_TO_TICKS(20));                       /* debounce */
        if (gpio_get_level(BTN_GPIO) != 0) continue;         /* not pressed */
        int64_t t0 = esp_timer_get_time() - 20000;
        bool long_sent = false;
        while (gpio_get_level(BTN_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            if (!long_sent && (esp_timer_get_time() - t0) / 1000 > RECALIB_HOLD_MS) {
                long_sent = true;
                g_evt_btn_us = esp_timer_get_time();
                xTaskNotify(s_notify, EVT_BTN_LONG, eSetBits);
            }
        }
        int64_t held_ms = (esp_timer_get_time() - t0) / 1000;
        if (!long_sent && held_ms < BTN_SHORT_MAX_MS) {
            g_evt_btn_us = esp_timer_get_time();
            xTaskNotify(s_notify, EVT_BTN_SHORT, eSetBits);
        }
        ulTaskNotifyTake(pdTRUE, 0);                         /* drop release-edge notification */
    }
}

void board_init(TaskHandle_t notify_task)
{
    s_notify = notify_task;
    gpio_config_t o = { .pin_bit_mask = 1ULL << LED_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&o);
    const esp_timer_create_args_t a1 = { .callback = hb_cb, .name = "led_hb" };
    const esp_timer_create_args_t a2 = { .callback = hb_off_cb, .name = "led_hb_off" };
    const esp_timer_create_args_t a3 = { .callback = blink_cb, .name = "led_blink" };
    esp_timer_create(&a1, &s_hb); esp_timer_create(&a2, &s_hb_off); esp_timer_create(&a3, &s_blink);

    xTaskCreate(btn_task, "btn", 2560, NULL, 6, &s_btn_task);
    gpio_config_t b = { .pin_bit_mask = 1ULL << BTN_GPIO, .mode = GPIO_MODE_INPUT,
                        .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE };
    gpio_config(&b);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_GPIO, btn_isr, NULL);
    board_led_set_mode(LED_MODE_NORMAL);
    ESP_LOGI(TAG, "LED GPIO%d, BOOT button GPIO%d", CONFIG_WT_PIN_LED, CONFIG_WT_PIN_BTN);
}
