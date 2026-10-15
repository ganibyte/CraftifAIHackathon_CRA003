#include "ble_beacon.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "wt_config.h"

static const char *TAG = "ble";

static char s_name[16];
static uint8_t s_payload[WT_PAYLOAD_LEN];
static bool s_fast;
static volatile bool s_synced;
static volatile bool s_adv;
static uint8_t s_own_addr_type;
static SemaphoreHandle_t s_lock;

static esp_err_t adv_apply_locked(void)
{
    uint8_t adv[31];
    size_t n = wt_adv_build(s_name, s_payload, adv, sizeof adv);
    if (n == 0) return ESP_ERR_INVALID_SIZE;

    if (ble_gap_adv_active()) ble_gap_adv_stop();
    int rc = ble_gap_adv_set_data(adv, (int)n);
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_data rc=%d", rc); return ESP_FAIL; }

    struct ble_gap_adv_params p;
    memset(&p, 0, sizeof p);
    p.conn_mode = BLE_GAP_CONN_MODE_NON;                 /* non-connectable beacon */
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    uint32_t ms = s_fast ? ADV_ALERT_MS : ADV_NORMAL_MS;
    p.itvl_min = p.itvl_max = BLE_GAP_ADV_ITVL_MS(ms);
    p.channel_map = 0;                                   /* all three advertising channels */

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &p, NULL, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "adv_start rc=%d", rc); s_adv = false; return ESP_FAIL; }
    s_adv = true;
    return ESP_OK;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "address setup rc=%d", rc); return; }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_synced = true;
    esp_err_t e = adv_apply_locked();
    xSemaphoreGive(s_lock);
    if (e == ESP_OK) printf("BLE advertising started name=%s itvl=%ums t=%lld\n",
                            s_name, (unsigned)(s_fast ? ADV_ALERT_MS : ADV_NORMAL_MS),
                            (long long)esp_timer_get_time());
}

static void on_reset(int reason) { s_synced = false; s_adv = false; ESP_LOGW(TAG, "host reset, reason=%d", reason); }

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t beacon_init(const char *name)
{
    strncpy(s_name, name, sizeof s_name - 1);
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    esp_err_t e = nimble_port_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "nimble_port_init: %s", esp_err_to_name(e)); return e; }
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

esp_err_t beacon_update(const uint8_t payload[WT_PAYLOAD_LEN], bool fast)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(s_payload, payload, WT_PAYLOAD_LEN);
    s_fast = fast;
    esp_err_t e = ESP_ERR_INVALID_STATE;          /* cached; applied by on_sync() */
    if (s_synced) e = adv_apply_locked();
    xSemaphoreGive(s_lock);
    return e;
}

bool beacon_is_advertising(void) { return s_adv; }
