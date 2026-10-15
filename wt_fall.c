#include "wt_fall.h"
#include "wt_config.h"
#include "wt_signal.h"
#include <math.h>
#include <string.h>

void wt_fall_reset(wt_fall_t *f)
{
    f->phase = WT_FALL_IDLE; f->inact_valid = false; f->inact_n = 0;
    f->inact_sum = (wt_vec3_t){0, 0, 0};
}
void wt_fall_init(wt_fall_t *f) { memset(f, 0, sizeof(*f)); wt_fall_reset(f); }
bool wt_fall_active(const wt_fall_t *f) { return f->phase != WT_FALL_IDLE; }

static uint8_t discard(wt_fall_t *f, wt_fall_discard_t why)
{
    f->last_discard = why;
    wt_fall_reset(f);
    return WT_FALL_EV_DISCARDED;
}

uint8_t wt_fall_feed(wt_fall_t *f, const wt_vec3_t *s, uint32_t now, const wt_vec3_t *ref)
{
    uint8_t ev = 0;
    float mag = wt_vec_norm(s);

    if (f->phase == WT_FALL_IDLE) {
        if (mag < FREEFALL_G) { f->phase = WT_FALL_FF_CAND; f->t_onset = now; f->t_last_low = now; }
        return 0;
    }

    if (f->phase == WT_FALL_FF_CAND) {                   /* stage 1: free-fall */
        if (mag < FREEFALL_G) {
            f->t_last_low = now;
            if (now - f->t_onset >= FREEFALL_MS) { ev |= WT_FALL_EV_STAGE1; f->phase = WT_FALL_WAIT_IMPACT; }
            else return 0;
        } else {
            if (f->t_last_low - f->t_onset >= FREEFALL_MS) { ev |= WT_FALL_EV_STAGE1; f->phase = WT_FALL_WAIT_IMPACT; }
            else { f->phase = WT_FALL_IDLE; return 0; }
        }
    }

    if (f->phase == WT_FALL_WAIT_IMPACT) {               /* stage 2: impact */
        if (now - f->t_onset > IMPACT_WINDOW_MS) return ev | discard(f, WT_FALL_DISCARD_IMPACT_TIMEOUT);
        if (mag > IMPACT_G) {
            f->phase = WT_FALL_POST_IMPACT; f->t_impact = now; f->inact_valid = false;
            ev |= WT_FALL_EV_STAGE2;
        }
        return ev;
    }

    if (f->phase == WT_FALL_POST_IMPACT) {               /* stage 3: inactivity + tilt */
        if (fabsf(mag - 1.0f) <= INACTIVITY_G) {
            if (!f->inact_valid) {
                f->inact_valid = true; f->inact_start = now; f->inact_n = 0;
                f->inact_sum = (wt_vec3_t){0, 0, 0};
            }
            f->inact_sum.x += s->x; f->inact_sum.y += s->y; f->inact_sum.z += s->z; f->inact_n++;
            if (now - f->inact_start >= INACTIVITY_MS) {
                wt_vec3_t m = { f->inact_sum.x / f->inact_n, f->inact_sum.y / f->inact_n, f->inact_sum.z / f->inact_n };
                f->last_tilt = wt_tilt_deg(&m, ref);
                f->last_inactivity_ms = now - f->inact_start;
                if (f->last_tilt >= TILT_FALL_DEG) { wt_fall_reset(f); return ev | WT_FALL_EV_STAGE3; }
                return ev | discard(f, WT_FALL_DISCARD_TILT);
            }
        } else if (now - f->t_impact < POST_IMPACT_SETTLE_MS) {
            f->inact_valid = false;                       /* still ringing: restart window */
        } else {
            return ev | discard(f, WT_FALL_DISCARD_MOVED);/* user moved (FR-10) */
        }
    }
    return ev;
}
