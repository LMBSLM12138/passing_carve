/**
 * @file rmcs_imu_ekf.h
 * @brief RMCS ImuEkf 的 C99 固定维移植。
 *
 * 该文件对应：
 *   RMCS-main/rmcs_ws/src/rmcs_core/src/filter/imu_ekf.hpp
 *
 * 状态量：
 *   x = [q0, q1, q2, q3]^T = [qw, qx, qy, qz]^T
 *
 * 坐标和单位：
 *   - 加速度输入单位必须是 g，不是 m/s^2。
 *   - 加速度原始转换按 RMCS Bmi088Ekf：
 *         accel_g = raw / 32767.0f * 6.0f
 *   - 陀螺仪输入单位是 rad/s。
 *
 * 默认参数与 RMCS ImuEkf::Config 一致：
 *   process_noise     = 1.22e-3
 *   measurement_noise = 50.0
 *   gate_noise        = 0.02
 *   initial_covariance= 0.15
 *
 * 注意：
 *   IMU 的 yaw 不能由重力加速度观测得到，因此该 EKF 不会直接修正 yaw
 *   的绝对漂移。Z 轴静态零偏必须在 bmi088.c 中单独标定。
 */

#ifndef __RMCS_IMU_EKF_H__
#define __RMCS_IMU_EKF_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q[4];                 /* q0,q1,q2,q3 */
    float P[16];                /* 4x4 covariance, row-major */

    float process_noise;        /* scalar diagonal Q */
    float measurement_noise;    /* scalar diagonal R */
    float gate_noise;           /* scalar diagonal gate covariance */
    float initial_covariance;   /* scalar initial covariance */

    uint32_t revision;
    float last_chi_square;
} RmcsImuEkf_t;

/** 用 RMCS 默认参数初始化。 */
void RmcsImuEkf_Init(RmcsImuEkf_t *ekf);

/** 用指定参数初始化。 */
void RmcsImuEkf_Configure(
    RmcsImuEkf_t *ekf,
    float process_noise,
    float measurement_noise,
    float gate_noise,
    float initial_covariance);

/** 重置为单位四元数和初始协方差。 */
void RmcsImuEkf_Reset(RmcsImuEkf_t *ekf);

/**
 * @brief 用重力方向初始化 roll/pitch，yaw 固定为 initial_yaw。
 * @param ax_g,ay_g,az_g 加速度，单位 g。
 * @return 1 成功，0 失败。
 */
int RmcsImuEkf_ResetFromAccel(
    RmcsImuEkf_t *ekf,
    float ax_g,
    float ay_g,
    float az_g,
    float initial_yaw_rad);

/**
 * @brief EKF 预测。
 * @param gyro_x_rad_s 陀螺仪角速度，单位 rad/s。
 * @param dt_seconds 时间间隔，单位 s。
 * @return 1 成功，0 失败。
 */
int RmcsImuEkf_Predict(
    RmcsImuEkf_t *ekf,
    float gyro_x_rad_s,
    float gyro_y_rad_s,
    float gyro_z_rad_s,
    float dt_seconds);

/**
 * @brief 用加速度计进行 EKF 修正。
 * @param ax_g,ay_g,az_g 加速度，单位 g。
 * @param gate_threshold RMCS Bmi088Ekf 使用的卡方门限，通常为 3.0。
 * @return 1 执行了修正，0 表示卡方拒绝或数值问题。
 */
int RmcsImuEkf_Correct(
    RmcsImuEkf_t *ekf,
    float ax_g,
    float ay_g,
    float az_g,
    float gate_threshold);

/** 放大姿态协方差，对应 RMCS 的 inflate_attitude_uncertainty_to_initial()。 */
void RmcsImuEkf_InflateAttitudeUncertainty(RmcsImuEkf_t *ekf);

#ifdef __cplusplus
}
#endif

#endif /* __RMCS_IMU_EKF_H__ */
