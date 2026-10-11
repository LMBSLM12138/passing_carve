/**
 * @file bmi088.h
 * @brief 达妙 DM-MC02 / STM32H723 板载 BMI088 C99 驱动与 RMCS ImuEkf 姿态解算。
 *
 */

#ifndef __BMI088_H__
#define __BMI088_H__

#include "main.h"
#include "spi.h"
#include "stm32h7xx_hal_spi.h"
#include "rmcs_imu_ekf.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_ACCEL_RANGE_G   6.0f
#define BMI088_GYRO_RANGE_DPS  2000.0f


extern volatile float accelerometer[3];       /* 单位: g */
extern volatile float BMI088_accel_mps2[3];  /* 单位: m/s^2 */
extern volatile float gyro[3];                /* 单位: rad/s，已减零偏 */
extern volatile float quat[4];                /* q0,q1,q2,q3 = w,x,y,z */
extern volatile float INS_angle[3];           /* yaw,pitch,roll，单位: rad */
extern volatile float INS_angle_deg[3];       /* yaw,pitch,roll，单位: degree */

extern volatile float BMI088_temperature_c;
extern volatile float BMI088_gyro_bias[3];    /* rad/s，传感器坐标 */
extern RmcsImuEkf_t bmi088_ekf;
extern volatile uint8_t BMI088_ready;

/**
 * @brief 初始化 SPI2 上的 BMI088。
 * @return 1 成功，0 失败。
 */
int BMI088_Init(void);

/**
 * @brief 与原工程兼容的周期读取函数。
 *
 * 请在主循环中调用。函数内部使用 HAL_GetTick() 限制到约 1 kHz，
 * 不会比 HAL 的 1 ms 时基更快地重复读取。
 */
void BMI088_work(void);

/**
 * @brief 使用调用者提供的精确 dt 读取并更新姿态。
 * @param dt_seconds 距上次调用的时间，单位秒。推荐固定 0.001f。
 * @return 1 本帧读取成功，0 失败。
 */
int BMI088_Update(float dt_seconds);

/**
 * @brief 静止状态下估计三轴陀螺仪零偏。
 * @param sample_count 采样次数，建议 1000~3000。
 * @param delay_ms 每次采样之间的延时，建议 1 ms。
 * @return 1 成功，0 失败。
 */
int BMI088_CalibrateGyroBias(uint16_t sample_count, uint16_t delay_ms);

/** 清除当前陀螺仪零偏。 */
void BMI088_ResetGyroBias(void);

/** 读取欧拉角，顺序 yaw,pitch,roll，单位 rad。 */
void BMI088_GetAngle(float *yaw, float *pitch, float *roll);

/**
 * @brief 将 INS_angle 从弧度转换并写入 INS_angle_deg。
 * 该函数会在姿态角更新后自动调用，也可以人工调用。
 */
void BMI088_UpdateAngleDeg(void);

/** 读取欧拉角，顺序 yaw,pitch,roll，单位 degree。 */
void BMI088_GetAngleDeg(float *yaw_deg, float *pitch_deg, float *roll_deg);

/** 读取四元数，顺序 w,x,y,z。 */
void BMI088_GetQuaternion(float q[4]);

/** 读取当前陀螺仪零偏，单位 rad/s。 */
void BMI088_GetGyroBias(float bias[3]);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H__ */









