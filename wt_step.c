#include "wt_step.h"
#include "wt_config.h"
#include <math.h>
#include <string.h>

void wt_step_init(wt_step_t *s, float fs_hz)
{
    memset(s, 0, sizeof(*s));
    s->fs = fs_hz;
    wt_lpf_init_fc(&s->lpf, STEP_LPF_HZ, fs_hz);
    uint32_t w = (uint32_t)(fs_hz * STEP_THRESH_WINDOW_MS / 1000.0f);
    if (w > WT_STEP_BUF_MAX) w = WT_STEP_BUF_MAX;
    s->win = (uint16_t)w;
}

void wt_step_reset_sequence(wt_step_t *s)
{
    wt_lpf_reset(&s->lpf);
    s->head = s->count = 0; s->have = 0;
    s->have_peak = false; s->seq_count = 0; s->walking = false; s->prev_dt = 0; s->rejected = 0;
}

static float threshold(const wt_step_t *s)
{
    float sum = 0, sq = 0;
    for (uint16_t i = 0; i < s->count; i++) { sum += s->buf[i]; sq += s->buf[i] * s->buf[i]; }
    float mean = sum / s->count;
    float var = sq / s->count - mean * mean;
    if (var < 0) var = 0;
    float d = STEP_THRESH_K * sqrtf(var);
    if (d < STEP_THRESH_MIN_DELTA_G) d = STEP_THRESH_MIN_DELTA_G;
    return mean + d;
}

static int on_peak(wt_step_t *s, uint32_t t)
{
    if (!s->have_peak) { s->have_peak = true; s->last_peak_ms = t; s->seq_count = 1; s->prev_dt = 0; s->rejected = 0; return 0; }
    uint32_t dt = t - s->last_peak_ms;
    if (dt < STEP_MIN_INTERVAL_MS) {                         /* too close: ignore, but remember it as noise */
        if (s->rejected < 255) s->rejected++;
        return 0;
    }
    uint8_t rej = s->rejected;
    s->rejected = 0;
    s->last_peak_ms = t;
    if (dt > STEP_MAX_INTERVAL_MS) {                         /* gap: start a new sequence */
        s->seq_count = 1; s->walking = false; s->prev_dt = 0; return 0;
    }
    if (s->walking && rej > STEP_NOISE_PEAKS_WALK) {          /* clutter: not a clean gait signal */
        s->walking = false; s->seq_count = 1; s->prev_dt = 0; return 0;
    }
    if (!s->walking && rej > STEP_NOISE_PEAKS) {
        s->seq_count = 1; s->prev_dt = 0; return 0;
    }
    if (s->walking) {
        uint32_t hi = dt > s->prev_dt ? dt : s->prev_dt;
        uint32_t lo = dt > s->prev_dt ? s->prev_dt : dt;
        if (s->prev_dt && (float)(hi - lo) > STEP_WALK_REGULARITY * (float)hi) {
            s->walking = false; s->seq_count = 2; s->prev_dt = dt;   /* rhythm lost: re-confirm */
            return 0;
        }
        s->steps++; s->prev_dt = dt; return 1;
    }
    if (s->seq_count >= 2) {                                 /* rhythm check: random shaking is irregular */
        uint32_t hi = dt > s->prev_dt ? dt : s->prev_dt;
        uint32_t lo = dt > s->prev_dt ? s->prev_dt : dt;
        if ((float)(hi - lo) > STEP_REGULARITY * (float)hi) {
            s->seq_count = 2; s->prev_dt = dt; return 0;     /* restart with this interval */
        }
    }
    s->prev_dt = dt;
    s->seq_count++;
    if (s->seq_count >= STEP_CONFIRM_COUNT) {                /* confirmed: credit retroactively */
        s->walking = true; s->steps += STEP_CONFIRM_COUNT; return STEP_CONFIRM_COUNT;
    }
    return 0;
}

int wt_step_feed(wt_step_t *s, float mag_g, uint32_t t_ms)
{
    float y = wt_lpf_step(&s->lpf, mag_g);
    if (s->count < s->win) s->count++;
    s->buf[s->head] = y;
    s->head = (uint16_t)((s->head + 1) % s->win);

    int credited = 0;
    if (s->have >= 2 && s->count >= s->win / 4) {
        if (s->y1 > s->y2 && s->y1 >= y && s->y1 > threshold(s))
            credited = on_peak(s, s->t1);
    }
    s->y2 = s->y1; s->y1 = y; s->t1 = t_ms;
    if (s->have < 2) s->have++;
    return credited;
}
