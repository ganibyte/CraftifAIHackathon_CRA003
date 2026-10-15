/* Signal features: magnitude, tilt, first-order low-pass. No ESP-IDF calls (FR-20). */
#pragma once
#include "wt_types.h"

float wt_vec_norm(const wt_vec3_t *v);
float wt_vec_dot(const wt_vec3_t *a, const wt_vec3_t *b);
/* Angle between two vectors in degrees, 0..180. Returns 0 if either is ~zero. */
float wt_tilt_deg(const wt_vec3_t *a, const wt_vec3_t *b);

typedef struct { float alpha; float y; bool primed; } wt_lpf_t;
void  wt_lpf_init_fc(wt_lpf_t *f, float fc_hz, float fs_hz);
void  wt_lpf_init_tau(wt_lpf_t *f, float tau_s, float fs_hz);
void  wt_lpf_reset(wt_lpf_t *f);
float wt_lpf_step(wt_lpf_t *f, float x);
