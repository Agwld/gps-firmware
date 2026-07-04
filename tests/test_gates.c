/**
 * @file    test_gates.c
 * @brief   Host unit tests for laptimer/gates.c crossing detection.
 */

#include <assert.h>
#include <stdio.h>

#include "board/board_config.h"
#include "laptimer/gates.h"

/* No <math.h>/libm here on purpose: the host test toolchain doesn't link
 * libm, so trig/abs stay as either compile-time constants or tiny local
 * helpers - see gates.c for why gates_check_crossing() itself is libm-
 * free too. */
#define PI_F 3.14159265358979323846f
#define SQRT2_OVER_2_F 0.70710678118654752440f /* sin/cos(pi/4) */

static float test_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void test_square_crossing(void)
{
    /* Gate at the origin, facing east (+x). A straight prev->cur segment
     * passing squarely through it dead-centre should be detected with
     * the crossing at the segment midpoint. */
    assert(gates_set(0U, 0.0f, 0.0f, 0.0f) == STATUS_OK);

    float frac = -1.0f;
    bool hit = gates_check_crossing(0U, -1.0f, 0.0f, 1.0f, 0.0f, &frac);

    assert(hit);
    assert(test_absf(frac - 0.5f) < 1e-4f);
}

static void test_outside_half_width_not_detected(void)
{
    /* Same gate, but the segment runs parallel to the heading axis
     * offset well beyond the half-width on the tangent axis: it still
     * crosses the *infinite* line (sign change on the forward axis) but
     * lies outside the finite gate segment, so it must not register. */
    assert(gates_set(0U, 0.0f, 0.0f, 0.0f) == STATUS_OK);

    float offset = LAP_GATE_HALF_WIDTH_M + 5.0f;
    float frac = -1.0f;
    bool hit = gates_check_crossing(0U, -1.0f, offset, 1.0f, offset, &frac);

    assert(!hit);
}

static void test_backwards_crossing_not_detected(void)
{
    /* Same straight-through path as test_square_crossing, but traversed
     * in reverse: the direction check (forward displacement dot heading
     * must be positive) must reject it. */
    assert(gates_set(0U, 0.0f, 0.0f, 0.0f) == STATUS_OK);

    float frac = -1.0f;
    bool hit = gates_check_crossing(0U, 1.0f, 0.0f, -1.0f, 0.0f, &frac);

    assert(!hit);
}

static void test_unset_gate_not_detected(void)
{
    gates_clear_all();

    float frac = -1.0f;
    bool hit = gates_check_crossing(1U, -1.0f, 0.0f, 1.0f, 0.0f, &frac);

    assert(!hit);
}

static void test_diagonal_heading_crossing(void)
{
    /* Sanity check with a non-axis-aligned heading: gate facing 45 deg
     * (north-east), crossed squarely along that same heading. */
    gates_clear_all();
    float heading = PI_F / 4.0f;
    assert(gates_set(0U, 10.0f, 20.0f, heading) == STATUS_OK);

    float hx = SQRT2_OVER_2_F;
    float hy = SQRT2_OVER_2_F;
    float prev_e = 10.0f - 2.0f * hx;
    float prev_n = 20.0f - 2.0f * hy;
    float cur_e = 10.0f + 2.0f * hx;
    float cur_n = 20.0f + 2.0f * hy;

    float frac = -1.0f;
    bool hit = gates_check_crossing(0U, prev_e, prev_n, cur_e, cur_n, &frac);

    assert(hit);
    assert(test_absf(frac - 0.5f) < 1e-3f);
}

static void test_set_index0_clears_others(void)
{
    gates_clear_all();
    assert(gates_set(1U, 5.0f, 5.0f, 0.0f) == STATUS_OK);

    float e, n, h;
    assert(gates_get(1U, &e, &n, &h) == STATUS_OK);

    assert(gates_set(0U, 0.0f, 0.0f, 0.0f) == STATUS_OK);
    assert(gates_get(1U, &e, &n, &h) == STATUS_NOT_READY);
}

static void test_get_bounds(void)
{
    gates_clear_all();
    float e, n, h;

    assert(gates_get(0U, &e, &n, &h) == STATUS_NOT_READY);
    assert(gates_get(LAP_MAX_GATES, &e, &n, &h) == STATUS_INVALID_ARG);
    assert(gates_set(LAP_MAX_GATES, 0.0f, 0.0f, 0.0f) == STATUS_INVALID_ARG);
}

static void test_clear_single_gate(void)
{
    gates_clear_all();
    float e, n, h;

    assert(gates_set(1U, 5.0f, 5.0f, 0.0f) == STATUS_OK);
    assert(gates_set(2U, 10.0f, 10.0f, 0.0f) == STATUS_OK);

    assert(gates_clear(1U) == STATUS_OK);

    assert(gates_get(1U, &e, &n, &h) == STATUS_NOT_READY);
    assert(gates_get(2U, &e, &n, &h) == STATUS_OK); /* untouched */

    assert(gates_clear(LAP_MAX_GATES) == STATUS_INVALID_ARG);
}

int main(void)
{
    gates_init();

    test_square_crossing();
    test_outside_half_width_not_detected();
    test_backwards_crossing_not_detected();
    test_unset_gate_not_detected();
    test_diagonal_heading_crossing();
    test_set_index0_clears_others();
    test_get_bounds();
    test_clear_single_gate();

    printf("test_gates: all tests passed\n");
    return 0;
}
