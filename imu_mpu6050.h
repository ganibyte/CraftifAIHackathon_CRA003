/* MPU6050 driver: accel-only FIFO @ 50/100/200 Hz, +/-8 g, free-fall interrupt (FR-1, FR-2, FR-11). */
#pragma once
#include "esp_err.h"
#include "wt_types.h"

#define IMU_MAX_BATCH 170      /* 1024-byte FIFO / 6 bytes per sample */

esp_err_t imu_init(void);                       /* idempotent; probes WHO_AM_I, configures 50 Hz */
esp_err_t imu_set_rate_hz(int hz);              /* 50 / 100 / 200; resets the FIFO */
/* Drain whole samples from the FIFO. Returns count (>=0) or a negative esp_err on I2C failure.
 * A FIFO overflow resets the FIFO and returns 0 (counted in imu_overflow_count()). */
int       imu_read_fifo(wt_vec3_t *out, int max_samples);
uint32_t  imu_overflow_count(void);
