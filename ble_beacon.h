/* Non-connectable NimBLE status beacon (FR-14, FR-15, FR-16). */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "wt_payload.h"

/* Start the NimBLE host. Advertising begins as soon as the host syncs, using the
 * most recent payload passed to beacon_update() (or an empty one). */
esp_err_t beacon_init(const char *name);

/* Replace the payload and restart advertising with it. fast=true uses the 100 ms alert
 * interval, otherwise 1000 ms. If the host is not synced yet the payload is cached.
 * Returns ESP_OK once advertising is (re)started with the new data. */
esp_err_t beacon_update(const uint8_t payload[WT_PAYLOAD_LEN], bool fast);

bool beacon_is_advertising(void);
