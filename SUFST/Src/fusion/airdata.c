/**
 * @file    airdata.c
 * @brief   Pitot airspeed, tare, and wind / scale-factor EKF (see airdata.h).
 */

#include "fusion/airdata.h"

#include <math.h>
#include <string.h>

#define AIRDATA_R_AIR          287.058f /* J/(kg K), dry air */
#define AIRDATA_RHO_STANDARD   1.225f

/* Tare: the car counts as stationary below this fused ground speed. The
 * first second of a stop is skipped (the car may still be rolling the
 * last few cm, and air dragged along with it takes a moment to settle);
 * after that the zero follows q with this time constant, and the tare is
 * trusted once the car has been still this long in total. */
#define AIRDATA_STATIONARY_MPS 0.3f
#define AIRDATA_TARE_SETTLE_S  1.0f
#define AIRDATA_TARE_TAU_S     2.0f
#define AIRDATA_TARE_MIN_S     3.0f

/* Wind/scale EKF. Fitted only above this ground speed: q grows with v^2,
 * so at low speed the 1 Pa pressure resolution and turbulence swamp the
 * airspeed. */
#define AIRDATA_FIT_MIN_SPEED_MPS 8.0f
/* Airspeed measurement noise: gusts, turbulence off the car, and the
 * sideslip a pitot sees in corners - not sensor noise, which is far
 * smaller at these speeds. */
#define AIRDATA_MEAS_SIGMA_MPS    0.7f
/* Random-walk process noise per second: wind drifts over minutes; k only
 * changes if the pitot is knocked. */
#define AIRDATA_Q_SCALE           (0.001f * 0.001f)
#define AIRDATA_Q_WIND            (0.05f * 0.05f)
/* Prior: k = 1 +- 0.15, wind = 0 +- 6 m/s. */
#define AIRDATA_P0_SCALE          (0.15f * 0.15f)
#define AIRDATA_P0_WIND           (6.0f * 6.0f)
/* Converged: 1-sigma wind within 0.7 m/s and k within 0.03. */
#define AIRDATA_VALID_VAR_WIND    (0.7f * 0.7f)
#define AIRDATA_VALID_VAR_SCALE   (0.03f * 0.03f)
/* Innovation gate (squared sigmas): rejects a gust, or another car's
 * wake, rather than letting it drag the wind estimate. */
#define AIRDATA_GATE_SIGMA2       9.0f
#define AIRDATA_SCALE_MIN         0.5f
#define AIRDATA_SCALE_MAX         1.5f

float
airdata_density(float p_static_mbar, float temp_c)
{
    float t_k = temp_c + 273.15f;
    /* A pressure that can't be ambient air (sensor absent or broken) or a
     * nonsense temperature: fall back to standard air rather than divide
     * by garbage. */
    if (p_static_mbar < 500.0f || p_static_mbar > 1200.0f || t_k < 200.0f) {
        return AIRDATA_RHO_STANDARD;
    }
    return (p_static_mbar * 100.0f) / (AIRDATA_R_AIR * t_k);
}

float
airdata_mean_channels(const float ch_mbar[4], uint8_t mask)
{
    float sum = 0.0f;
    uint32_t n = 0U;
    for (uint32_t i = 0U; i < 4U; i++) {
        if ((mask & (1U << i)) != 0U) {
            sum += ch_mbar[i];
            n++;
        }
    }
    return (n > 0U) ? (sum / (float) n) : 0.0f;
}

void
airdata_init(airdata_t *a)
{
    memset(a, 0, sizeof(*a));
    a->x[0] = 1.0f;
    a->p[0][0] = AIRDATA_P0_SCALE;
    a->p[1][1] = AIRDATA_P0_WIND;
    a->p[2][2] = AIRDATA_P0_WIND;
    a->density = AIRDATA_RHO_STANDARD;
}

static void
update_tare(airdata_t *a, float q_raw_pa, const airdata_input_t *in)
{
    float ground_speed = sqrtf(in->ve_mps * in->ve_mps +
                               in->vn_mps * in->vn_mps);

    if (!in->ground_valid || ground_speed >= AIRDATA_STATIONARY_MPS) {
        a->stationary_s = 0.0f;
        return;
    }

    float before = a->stationary_s;
    a->stationary_s += in->dt_s;
    if (a->stationary_s < AIRDATA_TARE_SETTLE_S) {
        return;
    }

    if (before < AIRDATA_TARE_SETTLE_S && !a->tared) {
        a->q_offset_pa = q_raw_pa; /* first stop: start from here */
    } else {
        float alpha = in->dt_s / AIRDATA_TARE_TAU_S;
        if (alpha > 1.0f) {
            alpha = 1.0f;
        }
        a->q_offset_pa += alpha * (q_raw_pa - a->q_offset_pa);
    }

    if (a->stationary_s >= AIRDATA_TARE_MIN_S) {
        a->tared = true;
    }
}

/* One EKF step on x = [k, we, wn] with z = k * |v_ground - w|. */
static void
update_wind(airdata_t *a, const airdata_input_t *in)
{
    for (uint32_t i = 0U; i < 3U; i++) {
        a->p[i][i] += in->dt_s * ((i == 0U) ? AIRDATA_Q_SCALE : AIRDATA_Q_WIND);
    }

    float ground_speed = sqrtf(in->ve_mps * in->ve_mps +
                               in->vn_mps * in->vn_mps);
    if (!in->ground_valid || !a->tared ||
        ground_speed < AIRDATA_FIT_MIN_SPEED_MPS) {
        return;
    }

    float k = a->x[0];
    float rel_e = in->ve_mps - a->x[1];
    float rel_n = in->vn_mps - a->x[2];
    float m = sqrtf(rel_e * rel_e + rel_n * rel_n);
    if (m < 1.0f) {
        return; /* only if the wind estimate is wildly off; skip */
    }

    float h[3] = {m, -k * rel_e / m, -k * rel_n / m};
    float y = a->indicated_mps - k * m;

    float ph[3];
    for (uint32_t i = 0U; i < 3U; i++) {
        ph[i] = a->p[i][0] * h[0] + a->p[i][1] * h[1] + a->p[i][2] * h[2];
    }
    float s = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2] +
              AIRDATA_MEAS_SIGMA_MPS * AIRDATA_MEAS_SIGMA_MPS;

    if (y * y > AIRDATA_GATE_SIGMA2 * s) {
        return;
    }

    float gain[3];
    for (uint32_t i = 0U; i < 3U; i++) {
        gain[i] = ph[i] / s;
        a->x[i] += gain[i] * y;
    }
    for (uint32_t i = 0U; i < 3U; i++) {
        for (uint32_t j = 0U; j < 3U; j++) {
            a->p[i][j] -= gain[i] * ph[j];
        }
    }
    /* Keep P symmetric against float round-off. */
    for (uint32_t i = 0U; i < 3U; i++) {
        for (uint32_t j = i + 1U; j < 3U; j++) {
            float avg = 0.5f * (a->p[i][j] + a->p[j][i]);
            a->p[i][j] = avg;
            a->p[j][i] = avg;
        }
    }

    if (a->x[0] < AIRDATA_SCALE_MIN) {
        a->x[0] = AIRDATA_SCALE_MIN;
    } else if (a->x[0] > AIRDATA_SCALE_MAX) {
        a->x[0] = AIRDATA_SCALE_MAX;
    }
}

void
airdata_update(airdata_t *a, const airdata_input_t *in)
{
    a->temp_used = in->temp_valid && in->temp_c > -40.0f &&
                   in->temp_c < 80.0f;
    a->density = airdata_density(in->p_static_mbar,
                                 a->temp_used ? in->temp_c
                                              : AIRDATA_DEFAULT_TEMP_C);

    float q_raw_pa = (in->p_total_mbar - in->p_static_mbar) * 100.0f;
    update_tare(a, q_raw_pa, in);

    a->q_pa = q_raw_pa - a->q_offset_pa;
    a->indicated_mps =
        (a->q_pa > 0.0f) ? sqrtf(2.0f * a->q_pa / a->density) : 0.0f;

    update_wind(a, in);
}

float
airdata_airspeed_mps(const airdata_t *a)
{
    if (!a->tared) {
        return 0.0f;
    }
    return a->indicated_mps / a->x[0];
}

void
airdata_wind(const airdata_t *a, float *wind_e_mps, float *wind_n_mps)
{
    *wind_e_mps = a->x[1];
    *wind_n_mps = a->x[2];
}

float
airdata_scale(const airdata_t *a)
{
    return a->x[0];
}

bool
airdata_wind_valid(const airdata_t *a)
{
    return a->tared && a->p[0][0] < AIRDATA_VALID_VAR_SCALE &&
           a->p[1][1] < AIRDATA_VALID_VAR_WIND &&
           a->p[2][2] < AIRDATA_VALID_VAR_WIND;
}
