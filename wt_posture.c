#include "wt_posture.h"
#include "wt_config.h"
#include <string.h>

void wt_posture_init(wt_posture_t *p, float fs_hz)
{
    memset(p, 0, sizeof(*p));
    p->fs = fs_hz;
    wt_lpf_init_tau(&p->lx, POSTURE_LPF_TAU_S, fs_hz);
    wt_lpf_init_tau(&p->ly, POSTURE_LPF_TAU_S, fs_hz);
    wt_lpf_init_tau(&p->lz, POSTURE_LPF_TAU_S, fs_hz);
}

void wt_posture_reset(wt_posture_t *p)
{
    wt_lpf_reset(&p->lx); wt_lpf_reset(&p->ly); wt_lpf_reset(&p->lz);
    p->tilt_deg = 0; p->bad = false; p->above = p->below = false;
}

int wt_posture_feed(wt_posture_t *p, const wt_vec3_t *s, uint32_t now, const wt_vec3_t *ref)
{
    wt_vec3_t g = { wt_lpf_step(&p->lx, s->x), wt_lpf_step(&p->ly, s->y), wt_lpf_step(&p->lz, s->z) };
    p->tilt_deg = wt_tilt_deg(&g, ref);
    int ev = WT_POSTURE_EV_NONE;

    if (p->tilt_deg >= POSTURE_DEG) { if (!p->above) { p->above = true; p->above_since = now; } }
    else p->above = false;

    if (p->tilt_deg < POSTURE_CLEAR_DEG) { if (!p->below) { p->below = true; p->below_since = now; } }
    else p->below = false;

    if (!p->bad && p->above && now - p->above_since >= (uint32_t)POSTURE_HOLD_S * 1000u) {
        p->bad = true; ev = WT_POSTURE_EV_ON;
    } else if (p->bad && p->below && now - p->below_since >= (uint32_t)POSTURE_CLEAR_S * 1000u) {
        p->bad = false; ev = WT_POSTURE_EV_OFF;
    }
    return ev;
}
