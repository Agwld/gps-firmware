/**
 * @file    mag_cal.c
 * @brief   Magnetometer hard-iron/soft-iron calibration - see mag_cal.h.
 */

#include "imu/mag_cal.h"

#define MAG_CAL_MIN_SAMPLES    100UL
#define MAG_CAL_MIN_RANGE      1.0f /* uT; below this, axis wasn't rotated */

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
