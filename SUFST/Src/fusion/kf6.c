/**
 * @file    kf6.c
 * @brief   6-state ENU position/velocity KF with delayed-state rewind.
 */

#include "fusion/kf6.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define KF6_N     6U /* state dimension: pe,pn,pu,ve,vn,vu */
#define KF6_M_MAX 3U /* largest measurement dimension used (pos/vel) */

#define KF6_GRAVITY_MPS2 9.80665f

/* Accelerometer noise driving process noise Q (see propagate()) - a
 * generic MEMS IMU figure, not yet tuned against the LSM6DSO32 datasheet
 * or bench data. */
#define KF6_ACCEL_NOISE_MPS2 0.3f

/* Initial covariance: deliberately uninformative (large) so the first
 * correction snaps the filter to it rather than fighting a confident
 * wrong prior. */
#define KF6_INIT_POS_VAR (200.0f * 200.0f)
#define KF6_INIT_VEL_VAR (100.0f * 100.0f)

typedef float kf6_vec_t[KF6_N];
typedef float kf6_mat_t[KF6_N][KF6_N];

typedef struct {
    uint32_t tick;
    float a_enu[3]; /* gravity-removed accel used for this predict step */
    float dt;
    kf6_vec_t x;
    kf6_mat_t p;
} kf6_hist_entry_t;

static kf6_vec_t s_x;
static kf6_mat_t s_p;

static kf6_hist_entry_t s_hist[KF6_HISTORY_LEN];
static uint8_t s_hist_count;
static uint8_t s_hist_head; /* index of the most recently written entry */

static void
mat_mult(const kf6_mat_t a, const kf6_mat_t b, kf6_mat_t out)
{
    for (uint32_t i = 0; i < KF6_N; i++) {
        for (uint32_t j = 0; j < KF6_N; j++) {
            float sum = 0.0f;
            for (uint32_t k = 0; k < KF6_N; k++) {
                sum += a[i][k] * b[k][j];
            }
            out[i][j] = sum;
        }
    }
}

static void
mat_transpose(const kf6_mat_t a, kf6_mat_t out)
{
    for (uint32_t i = 0; i < KF6_N; i++) {
        for (uint32_t j = 0; j < KF6_N; j++) {
            out[j][i] = a[i][j];
        }
    }
}

/* Rotate vector v by quaternion q: v' = q * v * q^-1 (ahrs.h's body -> ENU
 * convention: q is the attitude, so this maps a body-frame vector into
 * ENU). Reimplemented locally rather than shared with ahrs.c - a generic
 * rotation formula, not any of that module's internal correction state. */
static void
quat_rotate(quat_t q, float vx, float vy, float vz, float *rx, float *ry,
            float *rz)
{
    float tx = 2.0f * (q.y * vz - q.z * vy);
    float ty = 2.0f * (q.z * vx - q.x * vz);
    float tz = 2.0f * (q.x * vy - q.y * vx);

    *rx = vx + q.w * tx + (q.y * tz - q.z * ty);
    *ry = vy + q.w * ty + (q.z * tx - q.x * tz);
    *rz = vz + q.w * tz + (q.x * ty - q.y * tx);
}

/* Propagate (x, p) forward by one predict step in place: this is the
 * shared core between the live predict path and delayed-state replay,
 * so both use exactly the same transition. */
static void
propagate(kf6_vec_t x, kf6_mat_t p, const float a_enu[3], float dt)
{
    for (uint32_t i = 0; i < 3U; i++) {
        x[i] += x[i + 3U] * dt + 0.5f * a_enu[i] * dt * dt;
    }
    for (uint32_t i = 0; i < 3U; i++) {
        x[i + 3U] += a_enu[i] * dt;
    }

    kf6_mat_t f = {{0}};
    for (uint32_t i = 0; i < KF6_N; i++) {
        f[i][i] = 1.0f;
    }
    for (uint32_t i = 0; i < 3U; i++) {
        f[i][i + 3U] = dt;
    }

    kf6_mat_t fp, ft;
    mat_mult(f, p, fp);
    mat_transpose(f, ft);
    mat_mult(fp, ft, p);

    /* Q: discrete white-noise-acceleration model per axis, from treating
     * the accel input itself as noisy (variance sigma_a^2) rather than
     * exact - see any INS/GPS integration reference for this standard
     * derivation. Cross-axis terms are zero (axis noise assumed
     * independent). */
    float sigma_a2 = KF6_ACCEL_NOISE_MPS2 * KF6_ACCEL_NOISE_MPS2;
    float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;
    for (uint32_t i = 0; i < 3U; i++) {
        p[i][i] += sigma_a2 * 0.25f * dt4;
        p[i][i + 3U] += sigma_a2 * 0.5f * dt3;
        p[i + 3U][i] += sigma_a2 * 0.5f * dt3;
        p[i + 3U][i + 3U] += sigma_a2 * dt2;
    }
}

/* In-place inverse of the top-left n x n block of a KF6_M_MAX x KF6_M_MAX
 * matrix via Gauss-Jordan elimination, n <= KF6_M_MAX. No pivoting: this
 * filter only ever inverts an innovation covariance S = H P H^T + R,
 * which stays well-conditioned (R > 0, P positive semi-definite) for any
 * physically sane measurement noise, so a pivot search buys nothing here. */
static void
mat_inverse(const float a_in[KF6_M_MAX][KF6_M_MAX],
            float inv[KF6_M_MAX][KF6_M_MAX], uint32_t n)
{
    float a[KF6_M_MAX][2U * KF6_M_MAX];

    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            a[i][j] = a_in[i][j];
        }
        for (uint32_t j = 0; j < n; j++) {
            a[i][n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (uint32_t col = 0; col < n; col++) {
        float recip = 1.0f / a[col][col];
        for (uint32_t j = 0; j < 2U * n; j++) {
            a[col][j] *= recip;
        }
        for (uint32_t row = 0; row < n; row++) {
            if (row == col) {
                continue;
            }
            float factor = a[row][col];
            for (uint32_t j = 0; j < 2U * n; j++) {
                a[row][j] -= factor * a[col][j];
            }
        }
    }

    for (uint32_t i = 0; i < n; i++) {
        for (uint32_t j = 0; j < n; j++) {
            inv[i][j] = a[i][n + j];
        }
    }
}

/* Chi-squared 99.9th-percentile critical values, indexed [m-1] by
 * measurement dimension. A correction whose normalized innovation
 * squared (Mahalanobis distance y^T S^-1 y) exceeds this is statistically
 * inconsistent with the filter's own uncertainty and is rejected outright
 * rather than fused - e.g. a multipath GPS fix hundreds of metres off
 * with only a moderately elevated hAcc, which a plain KF has no defence
 * against otherwise (see kf6_correct_pos()/kf6_correct_vel()). */
static const float k_chi2_999[KF6_M_MAX] = {10.828f, 13.816f, 16.266f};

static bool
innovation_is_reasonable(const float y[KF6_M_MAX],
                          const float sinv[KF6_M_MAX][KF6_M_MAX], uint32_t m)
{
    float d2 = 0.0f;
    for (uint32_t i = 0; i < m; i++) {
        for (uint32_t j = 0; j < m; j++) {
            d2 += y[i] * sinv[i][j] * y[j];
        }
    }
    return d2 <= k_chi2_999[m - 1U];
}

/* Joseph-form linear KF measurement update on an arbitrary (x, p) pair -
 * used both for the live state and, during a rewind, for the historical
 * state being corrected. h/r are laid out as KF6_M_MAX-wide arrays with
 * only the first m rows/columns meaningful, so callers can pass fixed-size
 * locals without VLAs. `gate` enables the chi-squared outlier rejection
 * above - only for the absolute GPS position/velocity corrections, whose
 * declared sigma comes from the receiver's own (occasionally optimistic)
 * hAcc/sAcc; the wheelspeed-derived speed correction runs ungated since
 * it converges continuously at a fixed, already-conservative sigma rather
 * than arriving as a discrete, potentially-wrong fix. */
static void
kf_update(kf6_vec_t x, kf6_mat_t p, const float h[KF6_M_MAX][KF6_N],
          const float z[KF6_M_MAX], const float r[KF6_M_MAX][KF6_M_MAX],
          uint32_t m, bool gate)
{
    float y[KF6_M_MAX];
    for (uint32_t i = 0; i < m; i++) {
        float hx = 0.0f;
        for (uint32_t k = 0; k < KF6_N; k++) {
            hx += h[i][k] * x[k];
        }
        y[i] = z[i] - hx;
    }

    float pht[KF6_N][KF6_M_MAX];
    for (uint32_t i = 0; i < KF6_N; i++) {
        for (uint32_t j = 0; j < m; j++) {
            float sum = 0.0f;
            for (uint32_t k = 0; k < KF6_N; k++) {
                sum += p[i][k] * h[j][k];
            }
            pht[i][j] = sum;
        }
    }

    float s[KF6_M_MAX][KF6_M_MAX];
    for (uint32_t i = 0; i < m; i++) {
        for (uint32_t j = 0; j < m; j++) {
            float sum = 0.0f;
            for (uint32_t k = 0; k < KF6_N; k++) {
                sum += h[i][k] * pht[k][j];
            }
            s[i][j] = sum + r[i][j];
        }
    }

    float sinv[KF6_M_MAX][KF6_M_MAX];
    mat_inverse(s, sinv, m);

    if (gate && !innovation_is_reasonable(y, sinv, m)) {
        return; /* reject: leave (x, p) exactly as they were */
    }

    float k[KF6_N][KF6_M_MAX];
    for (uint32_t i = 0; i < KF6_N; i++) {
        for (uint32_t j = 0; j < m; j++) {
            float sum = 0.0f;
            for (uint32_t l = 0; l < m; l++) {
                sum += pht[i][l] * sinv[l][j];
            }
            k[i][j] = sum;
        }
    }

    for (uint32_t i = 0; i < KF6_N; i++) {
        float sum = 0.0f;
        for (uint32_t j = 0; j < m; j++) {
            sum += k[i][j] * y[j];
        }
        x[i] += sum;
    }

    /* P = (I - K H) P (I - K H)^T + K R K^T */
    kf6_mat_t ikh;
    for (uint32_t i = 0; i < KF6_N; i++) {
        for (uint32_t j = 0; j < KF6_N; j++) {
            float khij = 0.0f;
            for (uint32_t l = 0; l < m; l++) {
                khij += k[i][l] * h[l][j];
            }
            ikh[i][j] = ((i == j) ? 1.0f : 0.0f) - khij;
        }
    }

    kf6_mat_t tmp, ikht;
    mat_mult(ikh, p, tmp);
    mat_transpose(ikh, ikht);
    mat_mult(tmp, ikht, p);

    float kr[KF6_N][KF6_M_MAX];
    for (uint32_t i = 0; i < KF6_N; i++) {
        for (uint32_t j = 0; j < m; j++) {
            float sum = 0.0f;
            for (uint32_t l = 0; l < m; l++) {
                sum += k[i][l] * r[l][j];
            }
            kr[i][j] = sum;
        }
    }
    for (uint32_t i = 0; i < KF6_N; i++) {
        for (uint32_t j = 0; j < KF6_N; j++) {
            float sum = 0.0f;
            for (uint32_t l = 0; l < m; l++) {
                sum += kr[i][l] * k[j][l];
            }
            p[i][j] += sum;
        }
    }
}

void
kf6_init(void)
{
    memset(s_x, 0, sizeof(s_x));
    memset(s_p, 0, sizeof(s_p));
    for (uint32_t i = 0; i < 3U; i++) {
        s_p[i][i] = KF6_INIT_POS_VAR;
        s_p[i + 3U][i + 3U] = KF6_INIT_VEL_VAR;
    }

    s_hist_count = 0U;
    s_hist_head = 0U;
}

void
kf6_predict(uint32_t tick, float ax, float ay, float az, quat_t q, float dt)
{
    float a_enu[3];
    quat_rotate(q, ax, ay, az, &a_enu[0], &a_enu[1], &a_enu[2]);
    a_enu[2] -= KF6_GRAVITY_MPS2;

    propagate(s_x, s_p, a_enu, dt);

    s_hist_head = (uint8_t) ((s_hist_head + 1U) % KF6_HISTORY_LEN);
    kf6_hist_entry_t *h = &s_hist[s_hist_head];
    h->tick = tick;
    h->a_enu[0] = a_enu[0];
    h->a_enu[1] = a_enu[1];
    h->a_enu[2] = a_enu[2];
    h->dt = dt;
    memcpy(h->x, s_x, sizeof(s_x));
    memcpy(h->p, s_p, sizeof(s_p));

    if (s_hist_count < KF6_HISTORY_LEN) {
        s_hist_count++;
    }
}

static void
correct_with_rewind(uint32_t fix_tick, const float h[KF6_M_MAX][KF6_N],
                     const float z[KF6_M_MAX],
                     const float r[KF6_M_MAX][KF6_M_MAX], uint32_t m,
                     bool gate)
{
    if (s_hist_count == 0U) {
        kf_update(s_x, s_p, h, z, r, m, gate);
        return;
    }

    uint32_t oldest_idx =
        (s_hist_head + KF6_HISTORY_LEN + 1U - s_hist_count) %
        KF6_HISTORY_LEN;

    /* Find the newest entry at or before fix_tick (signed-difference
     * comparison so this stays correct across the tick counter's own
     * wraparound, same idiom as timebase.c). Falls back to the oldest
     * retained entry if the fix predates the whole window. */
    uint32_t target_slot = 0U;
    for (uint32_t slot = s_hist_count; slot-- > 0U;) {
        uint32_t idx = (oldest_idx + slot) % KF6_HISTORY_LEN;
        if ((int32_t) (s_hist[idx].tick - fix_tick) <= 0) {
            target_slot = slot;
            break;
        }
    }

    uint32_t target_idx = (oldest_idx + target_slot) % KF6_HISTORY_LEN;

    kf6_vec_t x_work;
    kf6_mat_t p_work;
    memcpy(x_work, s_hist[target_idx].x, sizeof(x_work));
    memcpy(p_work, s_hist[target_idx].p, sizeof(p_work));

    kf_update(x_work, p_work, h, z, r, m, gate);

    memcpy(s_hist[target_idx].x, x_work, sizeof(x_work));
    memcpy(s_hist[target_idx].p, p_work, sizeof(p_work));

    for (uint32_t slot = target_slot + 1U; slot < s_hist_count; slot++) {
        uint32_t idx = (oldest_idx + slot) % KF6_HISTORY_LEN;
        propagate(x_work, p_work, s_hist[idx].a_enu, s_hist[idx].dt);
        memcpy(s_hist[idx].x, x_work, sizeof(x_work));
        memcpy(s_hist[idx].p, p_work, sizeof(p_work));
    }

    memcpy(s_x, x_work, sizeof(s_x));
    memcpy(s_p, p_work, sizeof(s_p));
}

void
kf6_correct_pos(uint32_t fix_tick, float e_m, float n_m, float u_m,
                float sigma_pos_m)
{
    float h[KF6_M_MAX][KF6_N] = {{0}};
    h[0][0] = 1.0f;
    h[1][1] = 1.0f;
    h[2][2] = 1.0f;

    float z[KF6_M_MAX] = {e_m, n_m, u_m};

    float r[KF6_M_MAX][KF6_M_MAX] = {{0}};
    float var = sigma_pos_m * sigma_pos_m;
    r[0][0] = var;
    r[1][1] = var;
    r[2][2] = var;

    correct_with_rewind(fix_tick, h, z, r, 3U, true);
}

void
kf6_correct_vel(uint32_t fix_tick, float ve_mps, float vn_mps, float vu_mps,
                float sigma_vel_mps)
{
    float h[KF6_M_MAX][KF6_N] = {{0}};
    h[0][3] = 1.0f;
    h[1][4] = 1.0f;
    h[2][5] = 1.0f;

    float z[KF6_M_MAX] = {ve_mps, vn_mps, vu_mps};

    float r[KF6_M_MAX][KF6_M_MAX] = {{0}};
    float var = sigma_vel_mps * sigma_vel_mps;
    r[0][0] = var;
    r[1][1] = var;
    r[2][2] = var;

    correct_with_rewind(fix_tick, h, z, r, 3U, true);
}

void
kf6_correct_speed(uint32_t fix_tick, float speed_mps, float heading_rad,
                   float sigma_speed_mps)
{
    float h[KF6_M_MAX][KF6_N] = {{0}};
    h[0][3] = cosf(heading_rad);
    h[0][4] = sinf(heading_rad);

    float z[KF6_M_MAX] = {speed_mps, 0.0f, 0.0f};

    float r[KF6_M_MAX][KF6_M_MAX] = {{0}};
    r[0][0] = sigma_speed_mps * sigma_speed_mps;

    /* Ungated (see kf_update()'s doc comment): this runs continuously off
     * wheelspeed at a fixed, already-conservative sigma rather than as a
     * discrete fix that can itself be wrong. */
    correct_with_rewind(fix_tick, h, z, r, 1U, false);
}

void
kf6_get_state(float *e_m, float *n_m, float *u_m, float *ve_mps,
              float *vn_mps, float *vu_mps)
{
    if (e_m) {
        *e_m = s_x[0];
    }
    if (n_m) {
        *n_m = s_x[1];
    }
    if (u_m) {
        *u_m = s_x[2];
    }
    if (ve_mps) {
        *ve_mps = s_x[3];
    }
    if (vn_mps) {
        *vn_mps = s_x[4];
    }
    if (vu_mps) {
        *vu_mps = s_x[5];
    }
}
