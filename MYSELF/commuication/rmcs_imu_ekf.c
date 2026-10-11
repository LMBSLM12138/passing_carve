/**
 * @file rmcs_imu_ekf.c
 */

#include "rmcs_imu_ekf.h"

#include <math.h>
#include <string.h>

#define EKF_EPSILON 1.0e-9f
#define EKF_PI      3.14159265358979323846f

static void mat4_zero(float *m)
{
    memset(m, 0, sizeof(float) * 16U);
}

static void mat4_identity(float *m)
{
    uint8_t i;
    mat4_zero(m);
    for (i = 0U; i < 4U; ++i) {
        m[(uint16_t)i * 4U + i] = 1.0f;
    }
}

static void mat4_multiply(const float *a, const float *b, float *out)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            float sum = 0.0f;
            for (k = 0U; k < 4U; ++k) {
                sum += a[(uint16_t)i * 4U + k] * b[(uint16_t)k * 4U + j];
            }
            out[(uint16_t)i * 4U + j] = sum;
        }
    }
}

static void mat4_multiply_vector(const float *a, const float x[4], float out[4])
{
    uint8_t i;
    uint8_t k;

    for (i = 0U; i < 4U; ++i) {
        float sum = 0.0f;
        for (k = 0U; k < 4U; ++k) {
            sum += a[(uint16_t)i * 4U + k] * x[k];
        }
        out[i] = sum;
    }
}

static void mat4_transpose(const float *a, float *out)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            out[(uint16_t)i * 4U + j] = a[(uint16_t)j * 4U + i];
        }
    }
}

static void mat4_add_into(float *out, const float *a)
{
    uint8_t i;
    for (i = 0U; i < 16U; ++i) {
        out[i] += a[i];
    }
}

static int mat4_all_finite(const float *m)
{
    uint8_t i;
    for (i = 0U; i < 16U; ++i) {
        if (!isfinite(m[i])) {
            return 0;
        }
    }
    return 1;
}

static void mat4_symmetrize(float *m)
{
    uint8_t i;
    uint8_t j;
    for (i = 0U; i < 4U; ++i) {
        for (j = (uint8_t)(i + 1U); j < 4U; ++j) {
            const float value = 0.5f * (m[(uint16_t)i * 4U + j] + m[(uint16_t)j * 4U + i]);
            m[(uint16_t)i * 4U + j] = value;
            m[(uint16_t)j * 4U + i] = value;
        }
    }
}

static void normalize_vector3(float v[3])
{
    const float norm2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (norm2 <= EKF_EPSILON) {
        v[0] = 0.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
        return;
    }

    {
        const float inv_norm = 1.0f / sqrtf(norm2);
        v[0] *= inv_norm;
        v[1] *= inv_norm;
        v[2] *= inv_norm;
    }
}

static void normalize_quaternion(float q[4])
{
    const float norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (norm2 <= EKF_EPSILON) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    {
        const float inv_norm = 1.0f / sqrtf(norm2);
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

static void omega_matrix(const float gyro[3], float omega[16])
{
    mat4_zero(omega);
    omega[0] = 0.0f;
    omega[1] = -gyro[0];
    omega[2] = -gyro[1];
    omega[3] = -gyro[2];

    omega[4] = gyro[0];
    omega[5] = 0.0f;
    omega[6] = gyro[2];
    omega[7] = -gyro[1];

    omega[8] = gyro[1];
    omega[9] = -gyro[2];
    omega[10] = 0.0f;
    omega[11] = gyro[0];

    omega[12] = gyro[2];
    omega[13] = gyro[1];
    omega[14] = -gyro[0];
    omega[15] = 0.0f;
}

static void orthogonalization_jacobian(const float q[4], float jacobian[16])
{
    const float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    const float inv_norm = (isfinite(norm) && norm > EKF_EPSILON) ? (1.0f / norm) : 1.0f;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            jacobian[(uint16_t)i * 4U + j] =
                (i == j ? 1.0f : 0.0f)
                - inv_norm * inv_norm * q[i] * q[j];
        }
    }

    for (i = 0U; i < 16U; ++i) {
        jacobian[i] *= inv_norm;
    }
}

static void process_noise_jacobian(const float q[4], const float jacobian[16], float fw[12])
{
    /* 4x3 matrix from RMCS ImuEkf::process_noise_jacobian(). */
    fw[0] = -q[1];
    fw[1] = -q[2];
    fw[2] = -q[3];

    fw[3] = q[0];
    fw[4] = -q[3];
    fw[5] = q[2];

    fw[6] = q[3];
    fw[7] = q[0];
    fw[8] = -q[1];

    fw[9] = -q[2];
    fw[10] = q[1];
    fw[11] = q[0];

    /* fw = J_norm * (0.5 * Q_matrix). */
    {
        uint8_t i;
        uint8_t j;
        for (i = 0U; i < 4U; ++i) {
            for (j = 0U; j < 3U; ++j) {
                fw[(uint16_t)i * 3U + j] *= 0.5f;
            }
        }
    }

    {
        float temp[12];
        uint8_t i;
        uint8_t j;
        uint8_t k;

        for (i = 0U; i < 4U; ++i) {
            for (j = 0U; j < 3U; ++j) {
                float sum = 0.0f;
                for (k = 0U; k < 4U; ++k) {
                    sum += jacobian[(uint16_t)i * 4U + k] * fw[(uint16_t)k * 3U + j];
                }
                temp[(uint16_t)i * 3U + j] = sum;
            }
        }
        memcpy(fw, temp, sizeof(temp));
    }
}

static void measurement_model(const float q[4], float h[3])
{
    h[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    h[1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    h[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

static void measurement_jacobian(const float q[4], float h[12])
{
    h[0] = -2.0f * q[2];
    h[1] = 2.0f * q[3];
    h[2] = -2.0f * q[0];
    h[3] = 2.0f * q[1];

    h[4] = 2.0f * q[1];
    h[5] = 2.0f * q[0];
    h[6] = 2.0f * q[3];
    h[7] = 2.0f * q[2];

    h[8] = 2.0f * q[0];
    h[9] = -2.0f * q[1];
    h[10] = -2.0f * q[2];
    h[11] = 2.0f * q[3];
}

static int solve3(const float a[9], const float b[3], float x[3])
{
    float l00;
    float l10;
    float l11;
    float l20;
    float l21;
    float l22;
    float y0;
    float y1;
    float y2;

    if ((a[0] <= EKF_EPSILON) || !isfinite(a[0])) {
        return 0;
    }

    l00 = sqrtf(a[0]);
    l10 = a[3] / l00;
    l20 = a[6] / l00;

    {
        const float value = a[4] - l10 * l10;
        if (value <= EKF_EPSILON) {
            return 0;
        }
        l11 = sqrtf(value);
    }

    l21 = (a[7] - l20 * l10) / l11;

    {
        const float value = a[8] - l20 * l20 - l21 * l21;
        if (value <= EKF_EPSILON) {
            return 0;
        }
        l22 = sqrtf(value);
    }

    y0 = b[0] / l00;
    y1 = (b[1] - l10 * y0) / l11;
    y2 = (b[2] - l20 * y0 - l21 * y1) / l22;

    x[2] = y2 / l22;
    x[1] = (y1 - l21 * x[2]) / l11;
    x[0] = (y0 - l10 * x[1] - l20 * x[2]) / l00;

    return isfinite(x[0]) && isfinite(x[1]) && isfinite(x[2]);
}

static void projected_covariance(
    const float q[4],
    float initial_covariance,
    float covariance[16])
{
    float jacobian[16];
    uint8_t i;
    uint8_t j;
    uint8_t k;

    orthogonalization_jacobian(q, jacobian);

    /* raw covariance = initial_covariance * I4 */
    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            float sum = 0.0f;
            for (k = 0U; k < 4U; ++k) {
                sum += jacobian[(uint16_t)i * 4U + k]
                     * jacobian[(uint16_t)j * 4U + k];
            }
            covariance[(uint16_t)i * 4U + j] = initial_covariance * sum;
        }
    }
}

void RmcsImuEkf_Init(RmcsImuEkf_t *ekf)
{
    if (ekf == NULL) {
        return;
    }

    RmcsImuEkf_Configure(
        ekf,
        1.22e-3f,   /* RMCS ImuEkf default process_noise */
        50.0f,      /* RMCS ImuEkf default measurement_noise */
        0.02f,      /* RMCS ImuEkf default gate_noise */
        0.15f);     /* RMCS ImuEkf default initial_covariance */
}

void RmcsImuEkf_Configure(
    RmcsImuEkf_t *ekf,
    float process_noise,
    float measurement_noise,
    float gate_noise,
    float initial_covariance)
{
    if (ekf == NULL) {
        return;
    }

    ekf->process_noise = process_noise;
    ekf->measurement_noise = measurement_noise;
    ekf->gate_noise = gate_noise;
    ekf->initial_covariance = initial_covariance;
    RmcsImuEkf_Reset(ekf);
}

void RmcsImuEkf_Reset(RmcsImuEkf_t *ekf)
{
    if (ekf == NULL) {
        return;
    }

    ekf->q[0] = 1.0f;
    ekf->q[1] = 0.0f;
    ekf->q[2] = 0.0f;
    ekf->q[3] = 0.0f;

    projected_covariance(ekf->q, ekf->initial_covariance, ekf->P);
    ekf->last_chi_square = 0.0f;
    ekf->revision++;
}

int RmcsImuEkf_ResetFromAccel(
    RmcsImuEkf_t *ekf,
    float ax_g,
    float ay_g,
    float az_g,
    float initial_yaw_rad)
{
    float accel[3];
    float gx;
    float gy;
    float gz;
    float q0;
    float q1;
    float q2;
    float q3;
    float cy;
    float sy;
    float raw_q[4];

    if (ekf == NULL || !isfinite(ax_g) || !isfinite(ay_g) || !isfinite(az_g)
        || !isfinite(initial_yaw_rad)) {
        return 0;
    }

    accel[0] = ax_g;
    accel[1] = ay_g;
    accel[2] = az_g;
    normalize_vector3(accel);
    if ((accel[0] == 0.0f) && (accel[1] == 0.0f) && (accel[2] == 0.0f)) {
        return 0;
    }

    gx = accel[0];
    gy = accel[1];
    gz = accel[2];
    if (gz > 1.0f) {
        gz = 1.0f;
    } else if (gz < -1.0f) {
        gz = -1.0f;
    }

    if (gz >= 0.0f) {
        const float s = sqrtf(0.5f * (1.0f + gz));
        const float inv = 0.5f / s;
        q0 = s;
        q1 = gy * inv;
        q2 = -gx * inv;
        q3 = 0.0f;
    } else {
        const float s = sqrtf(0.5f * (1.0f - gz));
        const float inv = 0.5f / s;
        q0 = gy * inv;
        q1 = s;
        q2 = 0.0f;
        q3 = gx * inv;
    }

    cy = cosf(initial_yaw_rad * 0.5f);
    sy = sinf(initial_yaw_rad * 0.5f);

    raw_q[0] = cy * q0 - sy * q3;
    raw_q[1] = cy * q1 - sy * q2;
    raw_q[2] = cy * q2 + sy * q1;
    raw_q[3] = cy * q3 + sy * q0;

    normalize_quaternion(raw_q);
    memcpy(ekf->q, raw_q, sizeof(ekf->q));

    projected_covariance(ekf->q, ekf->initial_covariance, ekf->P);
    ekf->revision++;
    return 1;
}

int RmcsImuEkf_Predict(
    RmcsImuEkf_t *ekf,
    float gyro_x_rad_s,
    float gyro_y_rad_s,
    float gyro_z_rad_s,
    float dt_seconds)
{
    float gyro[3];
    float omega[16];
    float q_tmp[4];
    float q_pred[4];
    float jacobian[16];
    float f[16];
    float ft[16];
    float fw[12];
    float fp[16];
    float temp[16];
    float q_term[16];
    float p_pred[16];
    float half_dt;
    uint8_t i;
    uint8_t j;
    uint8_t k;

    if (ekf == NULL || !isfinite(gyro_x_rad_s) || !isfinite(gyro_y_rad_s)
        || !isfinite(gyro_z_rad_s) || !isfinite(dt_seconds)
        || (dt_seconds < 0.0f)) {
        return 0;
    }

    gyro[0] = gyro_x_rad_s;
    gyro[1] = gyro_y_rad_s;
    gyro[2] = gyro_z_rad_s;
    half_dt = 0.5f * dt_seconds;

    omega_matrix(gyro, omega);
    mat4_multiply_vector(omega, ekf->q, q_tmp);
    for (i = 0U; i < 4U; ++i) {
        q_tmp[i] = ekf->q[i] + half_dt * q_tmp[i];
    }

    orthogonalization_jacobian(q_tmp, jacobian);
    memcpy(q_pred, q_tmp, sizeof(q_pred));
    normalize_quaternion(q_pred);

    /* F = J_norm * (I + 0.5 * dt * Omega). */
    mat4_identity(f);
    for (i = 0U; i < 16U; ++i) {
        f[i] += half_dt * omega[i];
    }
    mat4_multiply(jacobian, f, temp);
    memcpy(f, temp, sizeof(f));

    /* Fw = J_norm * (0.5 * Q_matrix). */
    process_noise_jacobian(q_pred, jacobian, fw);

    mat4_transpose(f, ft);
    mat4_multiply(f, ekf->P, fp);
    mat4_multiply(fp, ft, p_pred);

    /* P = F P F^T + dt * Fw * Q * Fw^T. */
    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            float sum = 0.0f;
            for (k = 0U; k < 3U; ++k) {
                sum += fw[(uint16_t)i * 3U + k] * fw[(uint16_t)j * 3U + k];
            }
            q_term[(uint16_t)i * 4U + j] = dt_seconds * ekf->process_noise * sum;
        }
    }

    mat4_add_into(p_pred, q_term);
    mat4_symmetrize(p_pred);

    if (!mat4_all_finite(p_pred) || !mat4_all_finite((float *)ekf->P)) {
        return 0;
    }

    memcpy(ekf->q, q_pred, sizeof(ekf->q));
    memcpy(ekf->P, p_pred, sizeof(ekf->P));
    ekf->revision++;
    return 1;
}

int RmcsImuEkf_Correct(
    RmcsImuEkf_t *ekf,
    float ax_g,
    float ay_g,
    float az_g,
    float gate_threshold)
{
    float accel[3];
    float normalized_accel[3];
    float predicted_accel[3];
    float gate_innovation[3];
    float update_innovation[3];
    float h[12];
    float hp[12];
    float projected_cov[9];
    float gate_cov[9];
    float update_cov[9];
    float gate_solution[3];
    float update_solution[12];
    float effective_gain[12];
    float correction[4];
    float q_raw[4];
    float temp44[16];
    float temp_effective[16];
    float temp_p[16];
    float corrected_cov[16];
    float joseph[16];
    float jacobian[16];
    float chi_square;
    uint8_t i;
    uint8_t j;
    uint8_t k;

    if (ekf == NULL || !isfinite(ax_g) || !isfinite(ay_g) || !isfinite(az_g)) {
        return 0;
    }

    accel[0] = ax_g;
    accel[1] = ay_g;
    accel[2] = az_g;
    normalize_vector3(accel);
    if ((accel[0] == 0.0f) && (accel[1] == 0.0f) && (accel[2] == 0.0f)) {
        return 0;
    }

    normalized_accel[0] = accel[0];
    normalized_accel[1] = accel[1];
    normalized_accel[2] = accel[2];
    measurement_model(ekf->q, predicted_accel);

    gate_innovation[0] = ax_g - predicted_accel[0];
    gate_innovation[1] = ay_g - predicted_accel[1];
    gate_innovation[2] = az_g - predicted_accel[2];

    update_innovation[0] = normalized_accel[0] - predicted_accel[0];
    update_innovation[1] = normalized_accel[1] - predicted_accel[1];
    update_innovation[2] = normalized_accel[2] - predicted_accel[2];

    measurement_jacobian(ekf->q, h);

    /* H * P，3x4。 */
    for (i = 0U; i < 3U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            float sum = 0.0f;
            for (k = 0U; k < 4U; ++k) {
                sum += h[(uint16_t)i * 4U + k] * ekf->P[(uint16_t)k * 4U + j];
            }
            hp[(uint16_t)i * 4U + j] = sum;
        }
    }

    /* projected_cov = H * P * H^T，3x3。 */
    for (i = 0U; i < 3U; ++i) {
        for (j = 0U; j < 3U; ++j) {
            float sum = 0.0f;
            for (k = 0U; k < 4U; ++k) {
                sum += hp[(uint16_t)i * 4U + k] * h[(uint16_t)j * 4U + k];
            }
            projected_cov[(uint16_t)i * 3U + j] = sum;
        }
    }

    for (i = 0U; i < 9U; ++i) {
        gate_cov[i] = projected_cov[i];
        update_cov[i] = projected_cov[i];
    }
    gate_cov[0] += ekf->gate_noise;
    gate_cov[4] += ekf->gate_noise;
    gate_cov[8] += ekf->gate_noise;

    update_cov[0] += ekf->measurement_noise;
    update_cov[4] += ekf->measurement_noise;
    update_cov[8] += ekf->measurement_noise;

    if (!solve3(gate_cov, gate_innovation, gate_solution)) {
        return 0;
    }

    chi_square = gate_innovation[0] * gate_solution[0]
               + gate_innovation[1] * gate_solution[1]
               + gate_innovation[2] * gate_solution[2];
    ekf->last_chi_square = chi_square;

    if (!isfinite(chi_square) || (chi_square >= gate_threshold)) {
        return 0;
    }

    /* H * P 已计算为 hp。求解 (HP)^T = (H P)^T * S^{-1}。 */
    for (j = 0U; j < 4U; ++j) {
        float rhs[3];
        float solution[3];
        rhs[0] = hp[0 * 4U + j];
        rhs[1] = hp[1 * 4U + j];
        rhs[2] = hp[2 * 4U + j];
        if (!solve3(update_cov, rhs, solution)) {
            return 0;
        }
        update_solution[0 * 4U + j] = solution[0];
        update_solution[1 * 4U + j] = solution[1];
        update_solution[2 * 4U + j] = solution[2];
    }

    /* K = (S^{-1} * H * P)^T，4x3；最后一行由 state_update_mask 置零。 */
    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 3U; ++j) {
            effective_gain[(uint16_t)i * 3U + j] = update_solution[(uint16_t)j * 4U + i];
        }
    }
    effective_gain[3 * 3U + 0U] = 0.0f;
    effective_gain[3 * 3U + 1U] = 0.0f;
    effective_gain[3 * 3U + 2U] = 0.0f;

    for (i = 0U; i < 4U; ++i) {
        correction[i] = effective_gain[(uint16_t)i * 3U + 0U] * update_innovation[0]
                      + effective_gain[(uint16_t)i * 3U + 1U] * update_innovation[1]
                      + effective_gain[(uint16_t)i * 3U + 2U] * update_innovation[2];
        q_raw[i] = ekf->q[i] + correction[i];
    }
    normalize_quaternion(q_raw);

    /* (I - K_eff H)。 */
    mat4_identity(temp_effective);
    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            float sum = 0.0f;
            for (k = 0U; k < 3U; ++k) {
                sum += effective_gain[(uint16_t)i * 3U + k] * h[(uint16_t)k * 4U + j];
            }
            temp_effective[(uint16_t)i * 4U + j] -= sum;
        }
    }

    mat4_transpose(temp_effective, temp44);
    mat4_multiply(temp_effective, ekf->P, temp_p);
    mat4_multiply(temp_p, temp44, corrected_cov);

    /* K_eff * R * K_eff^T，R 是 measurement_noise * I3。 */
    for (i = 0U; i < 4U; ++i) {
        for (j = 0U; j < 4U; ++j) {
            float sum = 0.0f;
            for (k = 0U; k < 3U; ++k) {
                sum += effective_gain[(uint16_t)i * 3U + k]
                     * effective_gain[(uint16_t)j * 3U + k];
            }
            joseph[(uint16_t)i * 4U + j] = ekf->measurement_noise * sum;
        }
    }

    mat4_add_into(corrected_cov, joseph);
    mat4_symmetrize(corrected_cov);

    orthogonalization_jacobian(q_raw, jacobian);
    mat4_multiply(jacobian, corrected_cov, temp44);
    mat4_transpose(jacobian, temp_p);
    mat4_multiply(temp44, temp_p, ekf->P);

    if (!mat4_all_finite(ekf->P)) {
        return 0;
    }

    memcpy(ekf->q, q_raw, sizeof(ekf->q));
    ekf->revision++;
    return 1;
}

void RmcsImuEkf_InflateAttitudeUncertainty(RmcsImuEkf_t *ekf)
{
    if (ekf == NULL) {
        return;
    }

    projected_covariance(ekf->q, ekf->initial_covariance, ekf->P);
    ekf->revision++;
}


