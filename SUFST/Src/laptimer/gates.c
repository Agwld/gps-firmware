/**
 * @file    gates.c
 * @brief   Gate storage and crossing detection - see gates.h.
 */

#include "laptimer/gates.h"

#include <stdint.h>

#include "board/board_config.h"

typedef struct {
    float east_m;
    float north_m;
    float heading_rad;
    bool valid;
} gate_t;

static gate_t s_gates[LAP_MAX_GATES];

/* Heading is a runtime float (not a compile-time constant), so the only
 * way to get sin/cos of it without linking libm is a real approximation
 * - the host test toolchain doesn't link libm, and the target's
 * -Os flash budget would rather not pull it in either. Ninth/tenth-order
 * Taylor series after a fold into [0, pi/2] is accurate to ~1e-6, far
 * more than the geometry here needs. */
#define GATES_PI     3.14159265358979323846f
#define GATES_TWO_PI (2.0f * GATES_PI)

static float gates_wrap_pi(float x)
{
    /* Reduce to (-pi, pi]. Truncating cast to int is a single compiler
     * instruction, not a libm call. */
    int32_t k = (int32_t) (x * (1.0f / GATES_TWO_PI));
    float r = x - (float) k * GATES_TWO_PI;

    if (r > GATES_PI) {
        r -= GATES_TWO_PI;
    } else if (r <= -GATES_PI) {
        r += GATES_TWO_PI;
    }
    return r;
}

static void gates_sincosf(float x, float *sin_out, float *cos_out)
{
    float r = gates_wrap_pi(x);
    float sign = 1.0f;
    float ar = r;

    if (ar < 0.0f) {
        ar = -ar;
        sign = -1.0f;
    }

    float cos_sign = 1.0f;
    if (ar > GATES_PI * 0.5f) {
        ar = GATES_PI - ar;
        cos_sign = -1.0f;
    }

    float ar2 = ar * ar;

    float s = ar
              * (1.0f
                 + ar2
                       * (-1.0f / 6.0f
                          + ar2
                                * (1.0f / 120.0f
                                   + ar2
                                         * (-1.0f / 5040.0f
                                            + ar2
                                                  * (1.0f / 362880.0f
                                                     + ar2
                                                           * (-1.0f
                                                              / 39916800.0f))))));

    float c = 1.0f
              + ar2
                    * (-1.0f / 2.0f
                       + ar2
                             * (1.0f / 24.0f
                                + ar2
                                      * (-1.0f / 720.0f
                                         + ar2
                                               * (1.0f / 40320.0f
                                                  + ar2
                                                        * (-1.0f
                                                           / 3628800.0f)))));

    *sin_out = sign * s;
    *cos_out = cos_sign * c;
}

void gates_init(void)
{
    gates_clear_all();
}

void gates_clear_all(void)
{
    for (uint8_t i = 0U; i < LAP_MAX_GATES; i++) {
        s_gates[i].valid = false;
    }
}

status_t gates_set(uint8_t index, float east_m, float north_m,
                    float heading_rad)
{
    if (index >= LAP_MAX_GATES) {
        return STATUS_INVALID_ARG;
    }

    s_gates[index].east_m = east_m;
    s_gates[index].north_m = north_m;
    s_gates[index].heading_rad = heading_rad;
    s_gates[index].valid = true;

    /* A new start/finish line invalidates the previous sector split: the
     * old sector gates were positioned relative to the old lap geometry
     * and would otherwise silently mismatch it. */
    if (index == 0U) {
        for (uint8_t i = 1U; i < LAP_MAX_GATES; i++) {
            s_gates[i].valid = false;
        }
    }

    return STATUS_OK;
}

status_t gates_clear(uint8_t index)
{
    if (index >= LAP_MAX_GATES) {
        return STATUS_INVALID_ARG;
    }

    s_gates[index].valid = false;
    return STATUS_OK;
}

status_t gates_get(uint8_t index, float *east_m, float *north_m,
                    float *heading_rad)
{
    if (index >= LAP_MAX_GATES) {
        return STATUS_INVALID_ARG;
    }
    if (!s_gates[index].valid) {
        return STATUS_NOT_READY;
    }

    *east_m = s_gates[index].east_m;
    *north_m = s_gates[index].north_m;
    *heading_rad = s_gates[index].heading_rad;

    return STATUS_OK;
}

bool gates_check_crossing(uint8_t index, float prev_east, float prev_north,
                           float cur_east, float cur_north, float *frac_out)
{
    if (index >= LAP_MAX_GATES || !s_gates[index].valid) {
        return false;
    }

    const gate_t *g = &s_gates[index];

    /* Forward axis: along the stored heading, i.e. the normal to the
     * gate line (the line itself runs perpendicular to heading). Tangent
     * axis: along the gate line, used for the finite half-width check. */
    float hy, hx;
    gates_sincosf(g->heading_rad, &hy, &hx);
    const float tx = -hy;
    const float ty = hx;

    /* Signed distance of prev/cur from the gate point, along the
     * forward axis. A crossing of the infinite line is a sign change
     * from non-positive to positive on this axis. */
    const float s_prev = (prev_east - g->east_m) * hx
                          + (prev_north - g->north_m) * hy;
    const float s_cur = (cur_east - g->east_m) * hx
                         + (cur_north - g->north_m) * hy;

    /* Direction check: reject backwards (or stationary) crossings so
     * reversing over the line, or line-hugging jitter with no net
     * forward motion, never registers a lap/sector. */
    const float forward_disp = (cur_east - prev_east) * hx
                                + (cur_north - prev_north) * hy;
    if (forward_disp <= 0.0f) {
        return false;
    }

    if (!(s_prev <= 0.0f && s_cur > 0.0f)) {
        return false;
    }

    /* s_cur > 0.0f >= s_prev guarantees a strictly negative denominator
     * here, so this division is always safe. */
    float frac = s_prev / (s_prev - s_cur);
    if (frac < 0.0f) {
        frac = 0.0f;
    } else if (frac > 1.0f) {
        frac = 1.0f;
    }

    const float cross_east = prev_east + frac * (cur_east - prev_east);
    const float cross_north = prev_north + frac * (cur_north - prev_north);
    const float tangent_offset = (cross_east - g->east_m) * tx
                                  + (cross_north - g->north_m) * ty;

    const float abs_offset
        = (tangent_offset < 0.0f) ? -tangent_offset : tangent_offset;
    if (abs_offset > LAP_GATE_HALF_WIDTH_M) {
        return false;
    }

    *frac_out = frac;
    return true;
}
