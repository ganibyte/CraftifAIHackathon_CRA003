#include "imu_mpu6050.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu";

#define MPU_ADDR        0x68
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_FF_THR      0x1D
#define REG_FF_DUR      0x1E
#define REG_FIFO_EN     0x23
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE  0x38
#define REG_INT_STATUS  0x3A
#define REG_USER_CTRL   0x6A
#define REG_PWR_MGMT_1  0x6B
#define REG_PWR_MGMT_2  0x6C
#define REG_FIFO_CNT_H  0x72
#define REG_FIFO_RW     0x74
#define REG_WHO_AM_I    0x75

#define ACCEL_LSB_PER_G 4096.0f      /* +/-8 g */
#define I2C_TIMEOUT_MS  50

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_ready;
static uint32_t s_overflows;

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, I2C_TIMEOUT_MS);
}
static esp_err_t rd(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t bus_up(void)
{
    if (s_dev) return ESP_OK;
    if (!s_bus) {
        i2c_master_bus_config_t cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = CONFIG_WT_PIN_SDA,
            .scl_io_num = CONFIG_WT_PIN_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t e = i2c_new_master_bus(&cfg, &s_bus);
        if (e != ESP_OK) return e;
    }
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 400000,                       /* FR-1: 400 kHz */
    };
    return i2c_master_bus_add_device(s_bus, &dc, &s_dev);
}

esp_err_t imu_set_rate_hz(int hz)
{
    if (hz != 50 && hz != 100 && hz != 200) return ESP_ERR_INVALID_ARG;
    uint8_t div = (uint8_t)(1000 / hz - 1);           /* 1 kHz base with DLPF on */
    esp_err_t e;
    if ((e = wr(REG_USER_CTRL, 0x00)) != ESP_OK) return e;   /* FIFO off */
    if ((e = wr(REG_FIFO_EN, 0x00)) != ESP_OK) return e;
    if ((e = wr(REG_SMPLRT_DIV, div)) != ESP_OK) return e;
    if ((e = wr(REG_USER_CTRL, 0x04)) != ESP_OK) return e;   /* FIFO reset */
    if ((e = wr(REG_FIFO_EN, 0x08)) != ESP_OK) return e;     /* ACCEL_FIFO_EN */
    return wr(REG_USER_CTRL, 0x40);                          /* FIFO enable */
}

esp_err_t imu_init(void)
{
    s_ready = false;
    esp_err_t e = bus_up();
    if (e != ESP_OK) { ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(e)); return e; }

    uint8_t id = 0;
    e = rd(REG_WHO_AM_I, &id, 1);
    if (e != ESP_OK) { ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(e)); return e; }
    if (id != (uint8_t)CONFIG_WT_IMU_WHOAMI) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X, expected 0x%02X", id, (unsigned)CONFIG_WT_IMU_WHOAMI);
        return ESP_ERR_NOT_FOUND;
    }

    if ((e = wr(REG_PWR_MGMT_1, 0x80)) != ESP_OK) return e;  /* device reset */
    vTaskDelay(pdMS_TO_TICKS(100));
#if CONFIG_WT_IMU_GYRO_STANDBY
    if ((e = wr(REG_PWR_MGMT_1, 0x00)) != ESP_OK) return e;  /* internal oscillator */
    if ((e = wr(REG_PWR_MGMT_2, 0x07)) != ESP_OK) return e;  /* gyro axes standby */
#else
    if ((e = wr(REG_PWR_MGMT_1, 0x01)) != ESP_OK) return e;  /* PLL, gyro X reference */
    if ((e = wr(REG_PWR_MGMT_2, 0x00)) != ESP_OK) return e;
#endif
    vTaskDelay(pdMS_TO_TICKS(20));
    if ((e = wr(REG_CONFIG, 0x03)) != ESP_OK) return e;      /* DLPF ~44 Hz, 1 kHz base rate */
    if ((e = wr(REG_GYRO_CFG, 0x00)) != ESP_OK) return e;
    if ((e = wr(REG_ACCEL_CFG, 0x10)) != ESP_OK) return e;   /* AFS_SEL=2: +/-8 g (FR-2) */

    /* Free-fall interrupt: all axes below ~0.5 g for 10 ms, active-high push-pull pulse. */
    if ((e = wr(REG_FF_THR, 16)) != ESP_OK) return e;        /* 32 mg/LSB -> 512 mg */
    if ((e = wr(REG_FF_DUR, 10)) != ESP_OK) return e;        /* 1 ms/LSB */
    if ((e = wr(REG_INT_PIN_CFG, 0x00)) != ESP_OK) return e; /* active high, push-pull, 50 us pulse */
    if ((e = wr(REG_INT_ENABLE, 0x80)) != ESP_OK) return e;  /* FF_EN */

    if ((e = imu_set_rate_hz(50)) != ESP_OK) return e;       /* FR-2: 50 Hz + FIFO */
    uint8_t dummy; rd(REG_INT_STATUS, &dummy, 1);
    s_ready = true;
    ESP_LOGI(TAG, "MPU6050 ready (WHO_AM_I=0x%02X, +/-8 g, 50 Hz, FIFO)", id);
    return ESP_OK;
}

int imu_read_fifo(wt_vec3_t *out, int max_samples)
{
    if (!s_ready) return -ESP_ERR_INVALID_STATE;
    uint8_t cb[2];
    esp_err_t e = rd(REG_FIFO_CNT_H, cb, 2);
    if (e != ESP_OK) return -(int)e;
    int count = (cb[0] << 8) | cb[1];
    if (count >= 1008) {                                     /* overflow: realign FIFO */
        s_overflows++;
        wr(REG_USER_CTRL, 0x44);                             /* FIFO_EN | FIFO_RESET */
        return 0;
    }
    int n = count / 6;
    if (n > max_samples) n = max_samples;
    int done = 0;
    uint8_t raw[6 * 24];
    while (done < n) {
        int chunk = n - done;
        if (chunk > 24) chunk = 24;
        e = rd(REG_FIFO_RW, raw, (size_t)chunk * 6);
        if (e != ESP_OK) return -(int)e;
        for (int i = 0; i < chunk; i++) {
            const uint8_t *p = &raw[i * 6];
            out[done + i].x = (int16_t)((p[0] << 8) | p[1]) / ACCEL_LSB_PER_G;
            out[done + i].y = (int16_t)((p[2] << 8) | p[3]) / ACCEL_LSB_PER_G;
            out[done + i].z = (int16_t)((p[4] << 8) | p[5]) / ACCEL_LSB_PER_G;
        }
        done += chunk;
    }
    return done;
}

uint32_t imu_overflow_count(void) { return s_overflows; }
