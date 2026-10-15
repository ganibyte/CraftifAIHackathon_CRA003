/* LED patterns (FR-19) and BOOT-button handling (FR-3, FR-9). */
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum { LED_MODE_NORMAL, LED_MODE_ALERT, LED_MODE_FAULT } led_mode_t;

/* notify_task receives EVT_BTN_SHORT / EVT_BTN_LONG; g_evt_btn_us holds the event time. */
void board_init(TaskHandle_t notify_task);
void board_led_set_mode(led_mode_t mode);
