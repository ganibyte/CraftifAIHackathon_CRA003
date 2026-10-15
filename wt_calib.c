#include "wt_calib.h"
#include "wt_config.h"
#include "wt_signal.h"

void wt_calib_start(wt_calib_t *c)
{
    c->sx = c->sy = c->sz = 0; c->n = 0; c->start_ms = 0; c->restarts = 0;
    c->done = false; c->ref = (wt_vec3_t){0, 0, 1};
}

static void add(wt_calib_t *c, const wt_vec3_t *s, uint32_t now)
{
    if (c->n == 0) c->start_ms = now;
    c->sx += s->x; c->sy += s->y; c->sz += s->z; c->n++;
}

wt_calib_status_t wt_calib_feed(wt_calib_t *c, const wt_vec3_t *s, uint32_t now_ms)
{
    if (c->done) return WT_CALIB_DONE;
    if (c->n >= CALIB_MIN_SAMPLES) {
        wt_vec3_t m = { c->sx / c->n, c->sy / c->n, c->sz / c->n };
        wt_vec3_t d = { s->x - m.x, s->y - m.y, s->z - m.z };
        if (wt_vec_norm(&d) > CALIB_DEVIATION_G) {      /* moved: restart (FR-3) */
            c->sx = c->sy = c->sz = 0; c->n = 0; c->restarts++;
            add(c, s, now_ms);
            return WT_CALIB_RESTARTED;
        }
    }
    add(c, s, now_ms);
    if (now_ms - c->start_ms >= CALIB_DURATION_MS && c->n >= CALIB_MIN_SAMPLES) {
        c->ref = (wt_vec3_t){ c->sx / c->n, c->sy / c->n, c->sz / c->n };
        c->done = true;
        return WT_CALIB_DONE;
    }
    return WT_CALIB_RUNNING;
}
