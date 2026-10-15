#include "wt_activity.h"
#include "wt_config.h"
#include <string.h>

void wt_act_init(wt_activity_t *a) { memset(a, 0, sizeof(*a)); a->state = WT_STATE_CALIBRATING; }

void wt_act_begin_calibration(wt_activity_t *a)
{
    a->state = WT_STATE_CALIBRATING; a->walking = false; a->fall_flag = false;
}
void wt_act_calibration_done(wt_activity_t *a) { a->state = WT_STATE_IDLE; }

void wt_act_on_step(wt_activity_t *a, uint32_t now)
{
    if (a->state == WT_STATE_CALIBRATING) return;
    a->last_step_ms = now; a->walking = true;
    if (a->state == WT_STATE_IDLE) a->state = WT_STATE_WALKING;
}

bool wt_act_tick(wt_activity_t *a, uint32_t now)
{
    if (a->walking && now - a->last_step_ms >= IDLE_TIMEOUT_MS) {
        a->walking = false;
        if (a->state == WT_STATE_WALKING) a->state = WT_STATE_IDLE;
        return true;
    }
    return false;
}

void wt_act_on_fall(wt_activity_t *a, uint32_t now)
{
    a->fall_flag = true; a->falls++; a->state = WT_STATE_ALERT; a->alert_ms = now;
}

bool wt_act_cancel(wt_activity_t *a, uint32_t now)
{
    if (!a->fall_flag || now - a->alert_ms > CANCEL_WINDOW_MS) return false;
    a->fall_flag = false; a->state = WT_STATE_IDLE;
    return true;
}

bool wt_act_fast_alert(const wt_activity_t *a, uint32_t now)
{
    return a->fall_flag && now - a->alert_ms < ALERT_FAST_ADV_MS;
}
