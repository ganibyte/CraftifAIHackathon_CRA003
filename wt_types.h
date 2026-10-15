#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct { float x, y, z; } wt_vec3_t;

typedef enum {
    WT_STATE_IDLE = 0,
    WT_STATE_WALKING = 1,
    WT_STATE_ALERT = 2,
    WT_STATE_CALIBRATING = 3,
} wt_state_t;
