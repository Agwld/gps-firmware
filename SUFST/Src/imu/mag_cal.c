/**
 * @file    mag_cal.c
 * @brief   Magnetometer hard-iron/soft-iron calibration - see mag_cal.h.
 */

#include "imu/mag_cal.h"

#include <math.h>

#define MAG_CAL_MIN_SAMPLES    100UL
#define MAG_CAL_MIN_RANGE      1.0f /* uT; below this, axis wasn't rotated */

/* Continuous-calibration tuning. A window must sweep most of the heading
 * circle before it's trusted, so a car driving a few corners qualifies
 * but a straight-line crawl does not. */
#define MAG_CAL_CONT_MIN_SAMPLES 200UL
#define MAG_CAL_CONT_MIN_BINS    9 /* of MAG_CAL_HEADING_BINS (12) => 270 deg */

/* Course cross-check: how many moving-sample observations to gather
 * before judging, the resultant-length (0..1) above which the fused
 * heading is deemed to track course, and the cap at which the circular
 * accumulator forgets old blocks to stay adaptive. */
#define MAG_CAL_VAL_MIN_OBS   50UL
#define MAG_CAL_VAL_GOOD_R    0.90f
#define MAG_CAL_VAL_DROP_R    0.70f /* below this, demote VALIDATED->GOOD */
#define MAG_CAL_VAL_CAP       400UL

void
mag_cal_start(mag_cal_accumulator_t *acc)
{
    for (int i = 0; i < 3; i++) {
        acc->min[i] = 1e9f;
        acc->max[i] = -1e9f;
    }
    acc->sample_count = 0UL;
}

void
mag_cal_feed(mag_cal_accumulator_t *acc, float mx, float my, float mz)
{
    float v[3] = {mx, my, mz};
    for (int i = 0; i < 3; i++) {
        if (v[i] < acc->min[i]) {
            acc->min[i] = v[i];
        }
        if (v[i] > acc->max[i]) {
            acc->max[i] = v[i];
        }
    }
    acc->sample_count++;
}

status_t
mag_cal_finish(const mag_cal_accumulator_t *acc, mag_cal_result_t *out)
{
    if (acc->sample_count < MAG_CAL_MIN_SAMPLES) {
        return STATUS_NOT_READY;
    }

    float range[3];
    for (int i = 0; i < 3; i++) {
        range[i] = acc->max[i] - acc->min[i];
        if (range[i] < MAG_CAL_MIN_RANGE) {
            return STATUS_NOT_READY;
        }
    }

    float avg_range = (range[0] + range[1] + range[2]) / 3.0f;

    for (int i = 0; i < 3; i++) {
        out->bias[i] = 0.5f * (acc->max[i] + acc->min[i]);
        out->scale[i] = avg_range / range[i];
    }

    return STATUS_OK;
}

void
mag_cal_apply(const mag_cal_result_t *cal, float mx, float my, float mz,
              float *out_x, float *out_y, float *out_z)
{
    *out_x = (mx - cal->bias[0]) * cal->scale[0];
    *out_y = (my - cal->bias[1]) * cal->scale[1];
    *out_z = (mz - cal->bias[2]) * cal->scale[2];
}

void
mag_cal_identity(mag_cal_result_t *out)
{
    for (int i = 0; i < 3; i++) {
        out->bias[i] = 0.0f;
        out->scale[i] = 1.0f;
    }
}

/* ------------------------------------------------------------------ */
/* Continuous background calibration                                   */
/* ------------------------------------------------------------------ */

#define MAG_CAL_PI 3.14159265358979323846f

static bool
result_is_identity(const mag_cal_result_t *r)
{
    for (int i = 0; i < 3; i++) {
        if (r->bias[i] != 0.0f || r->scale[i] != 1.0f) {
            return false;
        }
    }
    return true;
}

/* A calibration change worth persisting / re-validating (as opposed to
 * the sub-noise jitter between two windows of the same field). */
static bool
result_differs(const mag_cal_result_t *a, const mag_cal_result_t *b)
{
    for (int i = 0; i < 3; i++) {
        float db = a->bias[i] - b->bias[i];
        if (db < 0.0f) {
            db = -db;
        }
        float ds = a->scale[i] - b->scale[i];
        if (ds < 0.0f) {
            ds = -ds;
        }
        if (db > 0.5f || ds > 0.02f) {
            return true;
        }
    }
    return false;
}

void
mag_cal_cont_init(mag_cal_cont_t *c, const mag_cal_result_t *initial)
{
    c->result = *initial;
    c->quality =
        result_is_identity(initial) ? MAG_CAL_UNCALIBRATED : MAG_CAL_GOOD;
    c->dirty = false;
    mag_cal_start(&c->window);
    c->bins_seen = 0U;
    c->off_sin = 0.0f;
    c->off_cos = 0.0f;
    c->val_count = 0UL;
}

static void
reset_validation(mag_cal_cont_t *c)
{
    c->off_sin = 0.0f;
    c->off_cos = 0.0f;
    c->val_count = 0UL;
}

void
mag_cal_cont_feed(mag_cal_cont_t *c, float mx, float my, float mz)
{
    mag_cal_feed(&c->window, mx, my, mz);

    /* Coverage: which sector of the heading circle this sample lands in,
     * using the window's running midpoint as a bias estimate. The
     * horizontal plane is vehicle x/y (z is up after mounting), which is
     * exactly the plane heading lives in - so a car sweeping corners
     * fills the bins even though it never tilts enough for a full sphere. */
    float bx = 0.5f * (c->window.max[0] + c->window.min[0]);
    float by = 0.5f * (c->window.max[1] + c->window.min[1]);
    float ang = atan2f(my - by, mx - bx); /* -pi..pi */
    int bin = (int) ((ang + MAG_CAL_PI) *
                     ((float) MAG_CAL_HEADING_BINS / (2.0f * MAG_CAL_PI)));
    if (bin < 0) {
        bin = 0;
    }
    if (bin >= MAG_CAL_HEADING_BINS) {
        bin = MAG_CAL_HEADING_BINS - 1;
    }
    c->bins_seen |= (uint16_t) (1U << bin);

    if (c->quality == MAG_CAL_UNCALIBRATED) {
        c->quality = MAG_CAL_COLLECTING;
    }

    if (c->window.sample_count < MAG_CAL_CONT_MIN_SAMPLES) {
        return;
    }

    int bins = 0;
    for (int i = 0; i < MAG_CAL_HEADING_BINS; i++) {
        if (c->bins_seen & (uint16_t) (1U << i)) {
            bins++;
        }
    }
    if (bins < MAG_CAL_CONT_MIN_BINS) {
        return; /* keep sweeping this window */
    }

    /* Enough of the circle covered: finalise the window - but only the
     * horizontal axes (x,y). A car stays level, so the vertical axis sees
     * a near-constant field and can't be calibrated from driving; its
     * bias/scale are carried over from whatever was loaded (identity, or a
     * prior full-sphere manual pass). Horizontal is all heading needs:
     * heading is atan2 of the two horizontal components, and equalising
     * their scale is what stops the field circle reading as an ellipse. */
    float rx = c->window.max[0] - c->window.min[0];
    float ry = c->window.max[1] - c->window.min[1];
    if (rx >= MAG_CAL_MIN_RANGE && ry >= MAG_CAL_MIN_RANGE) {
        mag_cal_result_t cand = c->result; /* keep vertical-axis terms */
        cand.bias[0] = 0.5f * (c->window.max[0] + c->window.min[0]);
        cand.bias[1] = 0.5f * (c->window.max[1] + c->window.min[1]);
        float avg = 0.5f * (rx + ry);
        cand.scale[0] = avg / rx;
        cand.scale[1] = avg / ry;

        bool changed = result_differs(&c->result, &cand);
        c->result = cand;
        if (c->quality < MAG_CAL_GOOD) {
            c->quality = MAG_CAL_GOOD;
        }
        if (changed) {
            c->dirty = true;
            /* A materially new calibration must re-prove itself against
             * course before it can claim VALIDATED again. */
            if (c->quality == MAG_CAL_VALIDATED) {
                c->quality = MAG_CAL_GOOD;
            }
            reset_validation(c);
        }
    }

    /* Start a fresh sweep regardless, so stale extremes can't accumulate
     * and a magnetic transient taints at most this one window. */
    mag_cal_start(&c->window);
    c->bins_seen = 0U;
}

void
mag_cal_cont_observe_heading(mag_cal_cont_t *c, float fused_yaw_rad,
                             float course_rad)
{
    if (c->quality < MAG_CAL_GOOD) {
        return; /* no calibration worth validating yet */
    }

    /* Circular-average the (course - yaw) difference. A tight cluster
     * (resultant length near 1) means the two headings differ by a
     * constant offset - declination plus mounting yaw - and otherwise
     * track together; a spread-out one means they don't. */
    float d = course_rad - fused_yaw_rad;
    c->off_sin += sinf(d);
    c->off_cos += cosf(d);
    c->val_count++;

    if (c->val_count >= MAG_CAL_VAL_CAP) {
        c->off_sin *= 0.5f;
        c->off_cos *= 0.5f;
        c->val_count /= 2UL;
    }

    if (c->val_count < MAG_CAL_VAL_MIN_OBS) {
        return;
    }

    float r = sqrtf(c->off_sin * c->off_sin + c->off_cos * c->off_cos) /
              (float) c->val_count;

    if (r >= MAG_CAL_VAL_GOOD_R) {
        c->quality = MAG_CAL_VALIDATED;
    } else if (r < MAG_CAL_VAL_DROP_R && c->quality == MAG_CAL_VALIDATED) {
        c->quality = MAG_CAL_GOOD;
    }
}

const mag_cal_result_t *
mag_cal_cont_result(const mag_cal_cont_t *c)
{
    return &c->result;
}

mag_cal_quality_t
mag_cal_cont_quality(const mag_cal_cont_t *c)
{
    return c->quality;
}

bool
mag_cal_cont_take_dirty(mag_cal_cont_t *c)
{
    bool d = c->dirty;
    c->dirty = false;
    return d;
}
