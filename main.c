/*
 * WearTrack - wearable activity / fall / posture beacon (ESP32 + MPU6050, ESP-IDF v5.5, NimBLE).
 *
 * Threads:
 *   sensor_task  - single owner of all algorithm state; woken by notifications
 *   NimBLE host  - advertising
 *   btn task     - BOOT button gestures (board.c)
 * Wakeups (all event-driven, no polling loops):
 *   EVT_TICK     esp_timer, 200 ms (normal tier) / 50 ms (fall window)   -> drain FIFO + process
 *   EVT_FF_IRQ   MPU6050 free-fall INT (GPIO23)                           -> 200 Hz tier
 *   EVT_REFRESH  esp_timer, 1 s                                           -> payload refresh
 *   EVT_BTN_*    BOOT short / long press
 */
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_events.h"
#include "board.h"
#include "ble_beacon.h"
#include "imu_mpu6050.h"
#include "wt_config.h"
#include "wt_signal.h"
#include "wt_calib.h"
#include "wt_step.h"
#include "wt_fall.h"
#include "wt_posture.h"
#include "wt_activity.h"
#include "wt_payload.h"

static const char *TAG = "wt";

#ifdef CONFIG_FREERTOS_USE_TICKLESS_IDLE
#define WT_TICKLESS 1
#else
#define WT_TICKLESS 0
#endif
#ifdef CONFIG_BT_CTRL_MODEM_SLEEP
#define WT_MODEM_SLEEP 1
#else
#define WT_MODEM_SLEEP 0
#endif

#if CONFIG_WT_POLLING_BASELINE
#define NORMAL_RATE_HZ   100
#define NORMAL_DRAIN_MS  10
#define TIERING          0
#else
#define NORMAL_RATE_HZ   ODR_NORMAL_HZ
#define NORMAL_DRAIN_MS  DRAIN_NORMAL_MS
#define TIERING          1
#endif

volatile int64_t g_evt_ff_us;

static struct {
    TaskHandle_t task;
    esp_timer_handle_t tick, refresh;

    wt_calib_t    calib;
    wt_step_t     step;
    wt_fall_t     fall;
    wt_posture_t  posture;
    wt_activity_t act;
    wt_vec3_t     ref;

    bool     fault;
    bool     deadline_seen;
    uint8_t  violations;
    uint8_t  seq;
    uint32_t cycles;
    uint32_t refresh_n;
    uint8_t  fault_retry;
    int      read_fail;

    bool     fast_tier;
    uint32_t fast_until_ms;
    int      rate_hz;
    int      decim;
    uint32_t decim_ctr;
    int64_t  last_tick_us;

    bool     adv_fast;
    bool     need_publish;
    led_mode_t led_mode;

    /* evidence bookkeeping (TR-5..TR-8) */
    bool     pend_step;   uint32_t pend_step_ms;
    bool     pend_fall;   int64_t  fall_confirm_us; uint32_t fall_impact_ms;
    bool     pend_cancel; int64_t  cancel_us;

    wt_vec3_t buf[IMU_MAX_BATCH];
} A;

static inline uint32_t ms_now(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline long long us_now(void) { return (long long)esp_timer_get_time(); }

static const char *state_name(wt_state_t s)
{
    switch (s) { case WT_STATE_IDLE: return "IDLE"; case WT_STATE_WALKING: return "WALKING";
                 case WT_STATE_ALERT: return "ALERT"; default: return "CALIBRATING"; }
}

/* ------------------------------------------------------------------ payload publishing */

static void build_status(wt_status_t *s)
{
    s->flags = wt_flags_build(A.act.fall_flag, A.posture.bad, A.act.walking, A.fault, A.deadline_seen);
    s->state = (uint8_t)A.act.state;
    s->steps = A.step.steps;
    float tilt = (A.act.state == WT_STATE_CALIBRATING) ? 0.0f : A.posture.tilt_deg;
    s->tilt_deg = (uint8_t)(tilt < 0 ? 0 : (tilt > 180 ? 180 : tilt + 0.5f));
    s->falls = A.act.falls;
    s->uptime_min = (uint16_t)((uint64_t)esp_timer_get_time() / 60000000ULL);
    s->violations = A.violations;
}

static void update_led(void)
{
    led_mode_t m = A.fault ? LED_MODE_FAULT : (A.act.fall_flag ? LED_MODE_ALERT : LED_MODE_NORMAL);
    if (m != A.led_mode) { A.led_mode = m; board_led_set_mode(m); }
}

static void publish(void)
{
    uint32_t now = ms_now();
    bool fast = wt_act_fast_alert(&A.act, now);
    wt_status_t st;
    uint8_t p[WT_PAYLOAD_LEN];
    build_status(&st);
    wt_payload_encode(&st, A.seq++, p);
    esp_err_t e = beacon_update(p, fast);
    int64_t t_us = esp_timer_get_time();
    A.adv_fast = fast;
    update_led();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) ESP_LOGW(TAG, "beacon_update: %s", esp_err_to_name(e));
    if (e != ESP_OK) return;       /* BLE not synced yet: evidence stays pending */

    int64_t t_ms = (int64_t)(t_us / 1000);
    if (A.pend_fall) {
        long long c2a = (long long)(t_us - A.fall_confirm_us);
        long long i2a = (long long)(t_ms - (int64_t)A.fall_impact_ms);
        printf("TR6 confirm_to_adv_us=%lld impact_to_adv_ms=%lld ok=%d\n", c2a, i2a,
               (c2a <= 300000 && i2a <= 2500));
        A.pend_fall = false;
    }
    if (A.pend_step) {
        long long d = (long long)(t_ms - (int64_t)A.pend_step_ms);
        printf("TR7 step_to_adv_ms=%lld ok=%d\n", d, d <= 1500);
        A.pend_step = false;
    }
    if (A.pend_cancel) {
        long long d = (long long)(t_us - A.cancel_us);
        printf("TR8 cancel_to_adv_us=%lld ok=%d\n", d, d <= 200000);
        A.pend_cancel = false;
    }
}

/* ------------------------------------------------------------------ calibration / fault */

static void start_calibration(void)
{
    wt_act_begin_calibration(&A.act);
    wt_calib_start(&A.calib);
    wt_fall_reset(&A.fall);
    wt_posture_reset(&A.posture);
    wt_step_reset_sequence(&A.step);
    A.pend_fall = false;
    printf("CALIB start - keep the device still for %d s t=%lld\n", CALIB_DURATION_MS / 1000, us_now());
}

static void enter_fault(const char *why)
{
    printf("ERROR: sensor fault (%s). LED 5 Hz, BLE keeps advertising. t=%lld\n", why, us_now());
    A.fault = true;
    A.read_fail = 0;
    if (A.tick) esp_timer_stop(A.tick);
    if (A.act.state == WT_STATE_CALIBRATING) A.act.state = WT_STATE_IDLE;
    A.need_publish = true;
}

static esp_err_t bring_up_imu(void)
{
    esp_err_t e = imu_init();
    if (e == ESP_OK && NORMAL_RATE_HZ != ODR_NORMAL_HZ) e = imu_set_rate_hz(NORMAL_RATE_HZ);
    if (e == ESP_OK) {
        A.rate_hz = NORMAL_RATE_HZ; A.fast_tier = false; A.decim = 1; A.decim_ctr = 0; A.last_tick_us = 0;
    }
    return e;
}

/* ------------------------------------------------------------------ per-sample processing */

static void process_sample(const wt_vec3_t *v, uint32_t t_ms)
{
    if (A.act.state == WT_STATE_CALIBRATING) {
        wt_calib_status_t st = wt_calib_feed(&A.calib, v, t_ms);
        if (st == WT_CALIB_RESTARTED) {
            printf("CALIB restart (movement > %.2f g) restarts=%u t=%lld\n",
                   (double)CALIB_DEVIATION_G, (unsigned)A.calib.restarts, us_now());
        } else if (st == WT_CALIB_DONE) {
            A.ref = A.calib.ref;
            wt_act_calibration_done(&A.act);
            wt_fall_reset(&A.fall);
            wt_posture_reset(&A.posture);
            wt_step_reset_sequence(&A.step);
            printf("CALIB done upright=(%.3f, %.3f, %.3f) |g|=%.3f restarts=%u t=%lld\n",
                   (double)A.ref.x, (double)A.ref.y, (double)A.ref.z, (double)wt_vec_norm(&A.ref),
                   (unsigned)A.calib.restarts, us_now());
            A.need_publish = true;
        }
        return;
    }

    /* fall detection runs on every sample at the current ODR; latched while an alert is open */
    if (!A.act.fall_flag) {
        uint8_t ev = wt_fall_feed(&A.fall, v, t_ms, &A.ref);
        if (ev & WT_FALL_EV_STAGE1) printf("FALL s1 t=%lld\n", us_now());
        if (ev & WT_FALL_EV_STAGE2) printf("FALL s2 t=%lld\n", us_now());
        if (ev & WT_FALL_EV_STAGE3) {
            printf("FALL s3 tilt=%.1f t=%lld\n", (double)A.fall.last_tilt, us_now());
            printf("TR5 inactivity_ms=%u ok=%d\n", (unsigned)A.fall.last_inactivity_ms,
                   (A.fall.last_inactivity_ms >= INACTIVITY_MS - 100 && A.fall.last_inactivity_ms <= INACTIVITY_MS + 100));
            wt_act_on_fall(&A.act, ms_now());
            A.pend_fall = true; A.fall_confirm_us = esp_timer_get_time(); A.fall_impact_ms = A.fall.t_impact;
            printf("FALL ALERT t=%lld falls=%u\n", us_now(), (unsigned)A.act.falls);
            A.need_publish = true;
        }
        if (ev & WT_FALL_EV_DISCARDED)
            printf("FALL discard reason=%d t=%lld\n", (int)A.fall.last_discard, us_now());
    }

    /* step / posture algorithms are tuned for the normal ODR: decimate in the 200 Hz tier */
    if ((A.decim_ctr++ % (uint32_t)A.decim) != 0) return;

    float mag = wt_vec_norm(v);
    int c = wt_step_feed(&A.step, mag, t_ms);
    if (c > 0) {
        uint32_t first = A.step.steps - (uint32_t)c + 1;
        for (int k = 0; k < c; k++) printf("STEP %u t=%lld\n", (unsigned)(first + (uint32_t)k), us_now());
        wt_act_on_step(&A.act, t_ms);
        if (!A.pend_step) { A.pend_step = true; A.pend_step_ms = t_ms; }
    }

    int pe = wt_posture_feed(&A.posture, v, t_ms, &A.ref);
    if (pe == WT_POSTURE_EV_ON)  { printf("POSTURE on t=%lld\n", us_now());  A.need_publish = true; }
    if (pe == WT_POSTURE_EV_OFF) { printf("POSTURE off t=%lld\n", us_now()); A.need_publish = true; }
}

/* Reads the FIFO and runs every sample through the algorithms. Never changes tiers. */
static int drain_and_process(void)
{
    int n = imu_read_fifo(A.buf, IMU_MAX_BATCH);
    if (n < 0) {
        if (++A.read_fail >= 10) enter_fault("I2C/FIFO read failures");
        return n;
    }
    A.read_fail = 0;
    uint32_t now = ms_now();
    for (int i = 0; i < n; i++) {
        uint32_t age = (uint32_t)((n - 1 - i) * 1000 / A.rate_hz);
        process_sample(&A.buf[i], now - age);
    }
    return n;
}

/* ------------------------------------------------------------------ sampling tiers (FR-11) */

static void set_tier(bool fast)
{
    if (!TIERING || fast == A.fast_tier) return;
    drain_and_process();                                  /* flush samples still at the old rate */
    int hz = fast ? ODR_FAST_HZ : NORMAL_RATE_HZ;
    if (imu_set_rate_hz(hz) != ESP_OK) { ESP_LOGW(TAG, "rate change failed"); return; }
    A.rate_hz = hz;
    A.fast_tier = fast;
    A.decim = fast ? (ODR_FAST_HZ / NORMAL_RATE_HZ) : 1;
    A.decim_ctr = 0;
    A.last_tick_us = 0;
    esp_timer_stop(A.tick);
    esp_timer_start_periodic(A.tick, (uint64_t)(fast ? DRAIN_FAST_MS : NORMAL_DRAIN_MS) * 1000ULL);
    printf("TIER %s rate=%dHz drain=%dms t=%lld\n", fast ? "FAST" : "NORMAL", hz,
           fast ? DRAIN_FAST_MS : NORMAL_DRAIN_MS, us_now());
}

static void housekeeping(uint32_t now)
{
    wt_act_tick(&A.act, now);
    if (wt_act_fast_alert(&A.act, now) != A.adv_fast) A.need_publish = true;   /* 30 s alert interval ended */

    if (TIERING && !A.fault) {
        bool cand = wt_fall_active(&A.fall);
        if (!A.fast_tier && cand) {                       /* software trigger in case the INT was missed */
            A.fast_until_ms = now + FAST_TIER_MS;
            set_tier(true);
        } else if (A.fast_tier && !cand && (int32_t)(now - A.fast_until_ms) >= 0) {
            set_tier(false);                              /* back to 50 Hz / 200 ms */
        }
    }
}

/* ------------------------------------------------------------------ event handlers */

static void on_tick(void)
{
    int64_t t0 = esp_timer_get_time();
    long long period = A.last_tick_us ? (long long)(t0 - A.last_tick_us) : 0;
    A.last_tick_us = t0;

    drain_and_process();
    housekeeping(ms_now());
    if (A.need_publish) { A.need_publish = false; publish(); }

    long long cyc = (long long)(esp_timer_get_time() - t0);       /* FR-18 */
    bool viol = cyc > DEADLINE_US;
    A.cycles++;
    if (viol) { if (A.violations < 255) A.violations++; A.deadline_seen = true; }
    if (viol || (A.cycles % TIMING_LOG_EVERY_N) == 0)
        printf("TIMING cycle_us=%lld violation=%d period_us=%lld tier=%s t=%lld\n",
               cyc, viol ? 1 : 0, period, A.fast_tier ? "fast" : "normal", us_now());
}

static void on_ff_irq(void)
{
    if (!TIERING || A.fault || A.act.state == WT_STATE_CALIBRATING) return;
    int64_t t_irq = g_evt_ff_us;
    A.fast_until_ms = ms_now() + FAST_TIER_MS;
    if (!A.fast_tier) {
        set_tier(true);
        long long d = (long long)(esp_timer_get_time() - t_irq);
        printf("TR3 ff_to_fast_us=%lld ok=%d\n", d, d <= 20000);
    }
}

static void on_btn_short(void)
{
    if (wt_act_cancel(&A.act, ms_now())) {
        wt_fall_reset(&A.fall);
        printf("CANCEL t=%lld\n", us_now());
        A.pend_cancel = true; A.cancel_us = g_evt_btn_us;
        publish();
    } else {
        printf("BTN short press ignored (no open alert within %d s) t=%lld\n", CANCEL_WINDOW_MS / 1000, us_now());
    }
}

static void on_btn_long(void)
{
    printf("RECALIB requested (BOOT held > %d s) t=%lld\n", RECALIB_HOLD_MS / 1000, us_now());
    if (A.fault) return;
    start_calibration();
    publish();
}

static void print_summary(void)
{
    wt_status_t st; build_status(&st);
    printf("SUMMARY state=%s steps=%u tilt=%u falls=%u violations=%u posture=%s uptime_min=%u t=%lld\n",
           state_name(A.act.state), (unsigned)st.steps, st.tilt_deg, st.falls, st.violations,
           A.posture.bad ? "BAD" : "ok", st.uptime_min, us_now());
}

static void on_refresh(void)
{
    if (A.fault && ++A.fault_retry >= 5) {                /* try to recover every ~5 s */
        A.fault_retry = 0;
        if (bring_up_imu() == ESP_OK) {
            A.fault = false;
            printf("IMU recovered\n");
            start_calibration();
            esp_timer_start_periodic(A.tick, (uint64_t)NORMAL_DRAIN_MS * 1000ULL);
        }
    }
    publish();
    if ((++A.refresh_n % SUMMARY_PERIOD_S) == 0) print_summary();
}

/* ------------------------------------------------------------------ plumbing */

static void tick_cb(void *arg)    { xTaskNotify(A.task, EVT_TICK, eSetBits); }
static void refresh_cb(void *arg) { xTaskNotify(A.task, EVT_REFRESH, eSetBits); }

#if TIERING
static void IRAM_ATTR ff_isr(void *arg)
{
    g_evt_ff_us = esp_timer_get_time();
    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(A.task, EVT_FF_IRQ, eSetBits, &hp);
    if (hp) portYIELD_FROM_ISR();
}
#endif

static void sensor_task(void *arg)
{
    A.task = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);              /* wait for app_main to finish board/BLE setup */

    const esp_timer_create_args_t t1 = { .callback = tick_cb, .name = "drain" };
    const esp_timer_create_args_t t2 = { .callback = refresh_cb, .name = "refresh" };
    esp_timer_create(&t1, &A.tick);
    esp_timer_create(&t2, &A.refresh);
    esp_timer_start_periodic(A.refresh, (uint64_t)PAYLOAD_REFRESH_MS * 1000ULL);

    esp_err_t e = bring_up_imu();                         /* FR-1 */
    if (e != ESP_OK) {
        enter_fault(esp_err_to_name(e));
        publish();
    } else {
#if TIERING
        gpio_config_t io = { .pin_bit_mask = 1ULL << CONFIG_WT_PIN_INT, .mode = GPIO_MODE_INPUT,
                             .intr_type = GPIO_INTR_POSEDGE };       /* active-high push-pull INT */
        gpio_config(&io);
        gpio_isr_handler_add((gpio_num_t)CONFIG_WT_PIN_INT, ff_isr, NULL);
#endif
        start_calibration();
        publish();
        esp_timer_start_periodic(A.tick, (uint64_t)NORMAL_DRAIN_MS * 1000ULL);
    }

    for (;;) {
        uint32_t bits = 0;
        xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);
        if (bits & EVT_FF_IRQ)    on_ff_irq();
        if (bits & EVT_BTN_SHORT) on_btn_short();
        if (bits & EVT_BTN_LONG)  on_btn_long();
        if (bits & EVT_TICK)      on_tick();
        if (bits & EVT_REFRESH)   on_refresh();
        if (A.need_publish && !(bits & EVT_TICK)) { A.need_publish = false; publish(); }
    }
}

static void power_config(void)
{
#if CONFIG_WT_ENABLE_PM && !CONFIG_WT_POLLING_BASELINE
    esp_pm_config_t pm = { .max_freq_mhz = 160, .min_freq_mhz = CONFIG_WT_PM_MIN_MHZ, .light_sleep_enable = true };
    esp_err_t e = esp_pm_configure(&pm);
    gpio_wakeup_enable((gpio_num_t)CONFIG_WT_PIN_BTN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)CONFIG_WT_PIN_INT, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    printf("PWR config: pm=%s max=160MHz min=%dMHz auto_light_sleep=%d tickless_idle=%d bt_modem_sleep=%d\n",
           e == ESP_OK ? "on" : "FAILED", CONFIG_WT_PM_MIN_MHZ, 1, WT_TICKLESS, WT_MODEM_SLEEP);
    if (e != ESP_OK) printf("PWR esp_pm_configure: %s\n", esp_err_to_name(e));
#else
    printf("PWR config: pm=off cpu=%dMHz (%s)\n", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
           TIERING ? "PM disabled in menuconfig" : "100 Hz polling baseline");
#endif
}

void app_main(void)
{
    esp_err_t r = nvs_flash_init();                       /* FR-14 */
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_BT));
    char name[6];
    wt_name_from_mac_byte(mac[5], name);
    printf("\nWearTrack v1.0 boot t=%lld\n", us_now());
    printf("BT MAC %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("BLE name %s\n", name);

    memset(&A, 0, sizeof A);
    wt_act_init(&A.act);
    wt_step_init(&A.step, (float)NORMAL_RATE_HZ);
    wt_posture_init(&A.posture, (float)NORMAL_RATE_HZ);
    wt_fall_init(&A.fall);
    wt_calib_start(&A.calib);
    A.ref = (wt_vec3_t){ 0, 0, 1 };
    A.rate_hz = NORMAL_RATE_HZ;
    A.decim = 1;
    A.led_mode = LED_MODE_ALERT;                          /* forces the first update_led() to apply NORMAL */

    TaskHandle_t th = NULL;
    xTaskCreate(sensor_task, "sensor", 6144, NULL, 10, &th);
    board_init(th);
    power_config();

    ESP_ERROR_CHECK(beacon_init(name));                   /* advertising starts on host sync, state CALIBRATING */
    A.task = th;
    publish();                                            /* cache the initial CALIBRATING payload */
    xTaskNotifyGive(th);                                  /* release the sensor task */
    ESP_LOGI(TAG, "init done");
}
