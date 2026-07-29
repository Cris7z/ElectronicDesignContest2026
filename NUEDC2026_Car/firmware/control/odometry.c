#include "odometry.h"
#include <math.h>

void odom_init(odom_t *o) { odom_reset(o); }

void odom_reset(odom_t *o)
{
    o->x_m = o->y_m = 0.0f;
    o->theta_rad = 0.0f;
    o->dist_m = 0.0f;
    o->_last_dl = o->_last_dr = 0.0f;
}

void odom_update(odom_t *o, float dl, float dr, float track_width_m,
                 float yaw_rad, bool yaw_ok)
{
    float ddl = dl - o->_last_dl;
    float ddr = dr - o->_last_dr;
    o->_last_dl = dl; o->_last_dr = dr;

    float ds = 0.5f * (ddl + ddr);
    o->dist_m += fabsf(ds);

    if (yaw_ok) {
        o->theta_rad = yaw_rad;              /* 陀螺仪直接给航向 */
    } else {
        o->theta_rad += (ddr - ddl) / track_width_m;
    }
    /* 中点弧线近似 */
    o->x_m += ds * cosf(o->theta_rad);
    o->y_m += ds * sinf(o->theta_rad);
}
