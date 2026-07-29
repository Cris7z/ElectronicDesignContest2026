#include "encoder.h"

#define PI_F 3.14159265f

void encoder_init(encoder_t *e, float counts_per_rev, float wheel_diameter_m)
{
    e->counts_per_rev = counts_per_rev;
    e->wheel_diameter_m = wheel_diameter_m;
    e->last_count = 0;
    e->speed_mps = e->speed_raw = 0.0f;
    e->dist_m = 0.0f;
    e->lpf_alpha = 0.5f;
    e->first = 1;
}

void encoder_update(encoder_t *e, int32_t count, float dt)
{
    if (e->first) { e->last_count = count; e->first = 0; return; }
    int32_t diff = count - e->last_count;
    e->last_count = count;

    float rev = (float)diff / e->counts_per_rev;
    float dist = rev * PI_F * e->wheel_diameter_m;
    e->dist_m += dist;
    e->speed_raw = (dt > 1e-6f) ? dist / dt : 0.0f;
    e->speed_mps += e->lpf_alpha * (e->speed_raw - e->speed_mps);
}
