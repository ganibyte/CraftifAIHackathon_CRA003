/* Activity state machine: IDLE / WALKING / ALERT / CALIBRATING (FR-6, FR-8, FR-9). */
#pragma once
#include "wt_types.h"

typedef struct {
    wt_state_t state;
    bool walking;              /* valid step within IDLE_TIMEOUT_MS */
    bool fall_flag;
    uint8_t falls;             /* 8-bit, wraps */
    uint32_t last_step_ms;
    uint32_t alert_ms;
} wt_activity_t;

void wt_act_init(wt_activity_t *a);
void wt_act_begin_calibration(wt_activity_t *a);       /* clears fall flag (FR-8) */
void wt_act_calibration_done(wt_activity_t *a);
void wt_act_on_step(wt_activity_t *a, uint32_t now_ms);
bool wt_act_tick(wt_activity_t *a, uint32_t now_ms);   /* true if state/walking changed */
void wt_act_on_fall(wt_activity_t *a, uint32_t now_ms);
bool wt_act_cancel(wt_activity_t *a, uint32_t now_ms); /* true if cancelled */
bool wt_act_fast_alert(const wt_activity_t *a, uint32_t now_ms);
