/* Notification bits for the sensor task and shared event timestamps. */
#pragma once
#include <stdint.h>

#define EVT_TICK       (1u << 0)   /* drain timer (200 ms normal / 50 ms fall window) */
#define EVT_FF_IRQ     (1u << 1)   /* IMU free-fall interrupt                         */
#define EVT_BTN_SHORT  (1u << 2)   /* BOOT short press released (<1 s)                */
#define EVT_BTN_LONG   (1u << 3)   /* BOOT held > 3 s                                 */
#define EVT_REFRESH    (1u << 4)   /* 1 s payload refresh                             */

/* esp_timer timestamps (us) captured at the instant the event occurred. */
extern volatile int64_t g_evt_ff_us;
extern volatile int64_t g_evt_btn_us;
