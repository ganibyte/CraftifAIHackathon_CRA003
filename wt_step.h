/* Step detection: low-pass + adaptive-threshold peak detect + 3-step confirmation (FR-4, FR-5). */
#pragma once
#include "wt_types.h"
#include "wt_signal.h"

#define WT_STEP_BUF_MAX 256

typedef struct {
    float fs;
    wt_lpf_t lpf;
    float buf[WT_STEP_BUF_MAX];
    uint16_t win, head, count;
    float y1, y2; uint32_t t1; uint8_t have;
    uint32_t last_peak_ms; bool have_peak;
    uint8_t seq_count;
    uint32_t prev_dt;
    uint8_t  rejected;         /* peaks rejected (too close) since last accepted step */
    bool walking;
    uint32_t steps;            /* 32-bit counter (FR-5) */
} wt_step_t;

void wt_step_init(wt_step_t *s, float fs_hz);
void wt_step_reset_sequence(wt_step_t *s);   /* keeps the 32-bit counter */
/* Feed one |a| sample (g). Returns steps credited by this sample: 0, 1, or 3 (retroactive). */
int  wt_step_feed(wt_step_t *s, float mag_g, uint32_t t_ms);
