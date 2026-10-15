#include "wt_signal.h"
#include <math.h>

#define RAD2DEG 57.29577951f

float wt_vec_norm(const wt_vec3_t *v) { return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z); }
float wt_vec_dot(const wt_vec3_t *a, const wt_vec3_t *b) { return a->x * b->x + a->y * b->y + a->z * b->z; }

float wt_tilt_deg(const wt_vec3_t *a, const wt_vec3_t *b)
{
    float na = wt_vec_norm(a), nb = wt_vec_norm(b);
    if (na < 1e-6f || nb < 1e-6f) return 0.0f;
    float c = wt_vec_dot(a, b) / (na * nb);
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c) * RAD2DEG;
}

void wt_lpf_init_fc(wt_lpf_t *f, float fc_hz, float fs_hz)
{
    float rc = 1.0f / (2.0f * 3.14159265f * fc_hz);
    float dt = 1.0f / fs_hz;
    f->alpha = dt / (rc + dt);
    f->y = 0; f->primed = false;
}

void wt_lpf_init_tau(wt_lpf_t *f, float tau_s, float fs_hz)
{
    float dt = 1.0f / fs_hz;
    f->alpha = dt / (tau_s + dt);
    f->y = 0; f->primed = false;
}

void wt_lpf_reset(wt_lpf_t *f) { f->y = 0; f->primed = false; }

float wt_lpf_step(wt_lpf_t *f, float x)
{
    if (!f->primed) { f->y = x; f->primed = true; }
    else f->y += f->alpha * (x - f->y);
    return f->y;
}
