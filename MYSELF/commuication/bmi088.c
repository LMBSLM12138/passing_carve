/**
 * @file bmi088.c
 * @brief 达妙 DM-MC02 / STM32H723 板载 BMI088 C99 驱动与 RMCS ImuEkf 姿态解算。
 *
 * 对应硬件：
 *   SPI2:
 *     PB13 -> SCK
 *     PC1  -> MOSI
 *     PC2_C-> MISO
 *   PC0 -> 加速度计 CS，低有效
 *   PC3 -> 陀螺仪 CS，低有效
 *
 * 使用方法：
 *   1. 在 MX_SPI2_Init() 和 MX_GPIO_Init() 之后调用 BMI088_Init()。
 *   2. 若需要低漂移角速度，静止放置开发板后调用
 *      BMI088_CalibrateGyroBias(2000, 1)。
 *   3. 在主循环中调用 BMI088_work()，或使用精确周期调用
 *      BMI088_Update(0.001f)。
 *
 * 注意：
 *   - 本文件不控制板载加热电阻。
 *   - RMCS ImuEkf 不包含磁力计，yaw 由陀螺仪积分得到；Z 轴需要单独静态零偏标定，并会随时间和温度缓慢漂移。
 *   - 原始工程 SPI2 当前约 328.125 kbit/s。一次三轴读取约需 0.4 ms，
 *     因此 MODE 下的 1 kHz 读取是可实现的，但主循环不要长时间阻塞。
 */

#include "bmi088.h"
#include "rmcs_imu_ekf.h"

#include <math.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* BMI088 寄存器                                                              */
/* -------------------------------------------------------------------------- */

#define BMI088_ACC_CHIP_ID          0x00U
#define BMI088_ACC_DATA             0x12U
#define BMI088_ACC_TEMP             0x22U
#define BMI088_ACC_CONF             0x40U
#define BMI088_ACC_RANGE            0x41U
#define BMI088_ACC_PWR_CONF         0x7CU
#define BMI088_ACC_PWR_CTRL         0x7DU
#define BMI088_ACC_SOFTRESET        0x7EU

#define BMI088_GYRO_CHIP_ID         0x00U
#define BMI088_GYRO_DATA            0x02U
#define BMI088_GYRO_RANGE           0x0FU
#define BMI088_GYRO_BANDWIDTH       0x10U
#define BMI088_GYRO_LPM1            0x11U
#define BMI088_GYRO_SOFTRESET       0x14U

#define BMI088_ACC_CHIP_ID_VALUE    0x1EU
#define BMI088_GYRO_CHIP_ID_VALUE   0x0FU
#define BMI088_SPI_READ_MASK        0x80U
#define BMI088_SPI_TIMEOUT_MS       1000U

/* 量程 ±3g，1 LSB 对应的 g 值。 */
#define BMI088_ACCEL_6G_SEN_G       (6.0f / 32767.0f)

/* 量程 ±2000 dps，1 LSB 对应的 rad/s。 */
#define BMI088_GYRO_2000_SEN_RAD_S  ((2000.0f * 0.01745329251994329577f) / 32767.0f)

#define BMI088_GRAVITY_MPS2         9.80665f
#define BMI088_GYRO_SATURATION_RAD_S (0.98f * 2000.0f * 0.01745329251994329577f)
#define BMI088_PI                   3.14159265358979323846f
#define BMI088_RAD_TO_DEG           (180.0f / BMI088_PI)
#define BMI088_DEG_TO_RAD           (BMI088_PI / 180.0f)
#define BMI088_TWO_KP_DEFAULT       1.0f
#define BMI088_TWO_KI_DEFAULT       0.0f
#define BMI088_DT_MAX               0.1f
#define BMI088_EPSILON              1.0e-12f
/* -------------------------------------------------------------------------- */
/* 对外数据                                                                   */
/* -------------------------------------------------------------------------- */

volatile float accelerometer[3] = {0.0f, 0.0f, 0.0f};
volatile float BMI088_accel_mps2[3] = {0.0f, 0.0f, 0.0f};
volatile float gyro[3] = {0.0f, 0.0f, 0.0f};
volatile float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float INS_angle[3] = {0.0f, 0.0f, 0.0f};
volatile float INS_angle_deg[3] = {0.0f, 0.0f, 0.0f};

volatile float BMI088_temperature_c = 0.0f;
volatile float BMI088_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
volatile uint8_t BMI088_ready = 0U;
RmcsImuEkf_t bmi088_ekf;

/* -------------------------------------------------------------------------- */
/* 内部状态                                                                   */
/* -------------------------------------------------------------------------- */


static uint8_t bmi088_attitude_initialized = 0U;
static uint8_t bmi088_temperature_valid = 0U;
static uint32_t bmi088_last_temperature_ms = 0U;

static uint8_t bmi088_last_work_tick_valid = 0U;
static uint32_t bmi088_last_work_tick_ms = 0U;

/* -------------------------------------------------------------------------- */
/* 基础数学与 SPI 工具                                                        */
/* -------------------------------------------------------------------------- */

static float bmi088_clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int16_t bmi088_make_int16_le(uint8_t lsb, uint8_t msb)
{
    uint16_t value = (uint16_t)lsb | ((uint16_t)msb << 8U);
    return (int16_t)value;
}

static int bmi088_spi_transfer(
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    uint8_t *tx,
    uint8_t *rx,
    uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((cs_port == NULL) || (tx == NULL) || (rx == NULL) || (length == 0U)) {
        return 0;
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(
        &hspi2, tx, rx, length, BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK) ? 1 : 0;
}

static int bmi088_accel_read(uint8_t address, uint8_t *data, uint16_t length)
{
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};
    uint16_t i;

    if ((data == NULL) || (length == 0U) || (length > 6U)) {
        return 0;
    }

    /* 加速度计读取需要额外发送一个 dummy 字节。 */
    tx[0] = (uint8_t)(address | BMI088_SPI_READ_MASK);
    if (!bmi088_spi_transfer(GPIOC, GPIO_PIN_0, tx, rx, (uint16_t)(length + 2U))) {
        return 0;
    }

    for (i = 0U; i < length; ++i) {
        data[i] = rx[i + 2U];
    }
    return 1;
}

static int bmi088_gyro_read(uint8_t address, uint8_t *data, uint16_t length)
{
    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};
    uint16_t i;

    if ((data == NULL) || (length == 0U) || (length > 6U)) {
        return 0;
    }

    /* 陀螺仪读取没有 dummy 字节。 */
    tx[0] = (uint8_t)(address | BMI088_SPI_READ_MASK);
    if (!bmi088_spi_transfer(GPIOC, GPIO_PIN_3, tx, rx, (uint16_t)(length + 1U))) {
        return 0;
    }

    for (i = 0U; i < length; ++i) {
        data[i] = rx[i + 1U];
    }
    return 1;
}

static int bmi088_accel_write(uint8_t address, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0};

    tx[0] = (uint8_t)(address & 0x7FU);
    tx[1] = value;
    return bmi088_spi_transfer(GPIOC, GPIO_PIN_0, tx, rx, 2U);
}

static int bmi088_gyro_write(uint8_t address, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0};

    tx[0] = (uint8_t)(address & 0x7FU);
    tx[1] = value;
    return bmi088_spi_transfer(GPIOC, GPIO_PIN_3, tx, rx, 2U);
}

static int bmi088_probe_accel(void)
{
    uint8_t chip_id = 0U;
    uint32_t retry;

    for (retry = 0U; retry < 20U; ++retry) {
        if (bmi088_accel_read(
                BMI088_ACC_CHIP_ID, &chip_id, 1U)
            && (chip_id == BMI088_ACC_CHIP_ID_VALUE)) {
            return 1;
        }
        HAL_Delay(10U);
    }
    return 0;
}

static int bmi088_probe_gyro(void)
{
    uint8_t chip_id = 0U;
    uint32_t retry;

    for (retry = 0U; retry < 20U; ++retry) {
        if (bmi088_gyro_read(
                BMI088_GYRO_CHIP_ID, &chip_id, 1U)
            && (chip_id == BMI088_GYRO_CHIP_ID_VALUE)) {
            return 1;
        }
        HAL_Delay(10U);
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/* 数据读取与单位转换                                                         */
/* -------------------------------------------------------------------------- */

static int bmi088_read_accel_raw(int16_t raw[3])
{
    uint8_t data[6];

    if (!bmi088_accel_read(BMI088_ACC_DATA, data, sizeof(data))) {
        return 0;
    }

    raw[0] = bmi088_make_int16_le(data[0], data[1]);
    raw[1] = bmi088_make_int16_le(data[2], data[3]);
    raw[2] = bmi088_make_int16_le(data[4], data[5]);
    return 1;
}

static int bmi088_read_gyro_raw(int16_t raw[3])
{
    uint8_t data[6];

    if (!bmi088_gyro_read(BMI088_GYRO_DATA, data, sizeof(data))) {
        return 0;
    }

    raw[0] = bmi088_make_int16_le(data[0], data[1]);
    raw[1] = bmi088_make_int16_le(data[2], data[3]);
    raw[2] = bmi088_make_int16_le(data[4], data[5]);
    return 1;
}

static int bmi088_read_temperature(float *temperature_c)
{
    uint8_t data[2];
    int32_t raw;
    float value;

    if ((temperature_c == NULL)
        || !bmi088_accel_read(BMI088_ACC_TEMP, data, sizeof(data))) {
        return 0;
    }

    raw = ((int32_t)data[0] << 3) | ((int32_t)data[1] >> 5);
    if ((raw & 0x0400) != 0) {
        raw -= 0x0800;
    }

    value = 23.0f + (float)raw * 0.125f;
    *temperature_c = value;
    return 1;
}

static void bmi088_convert_accel(const int16_t raw[3])
{
    accelerometer[0] = (float)raw[0] * BMI088_ACCEL_6G_SEN_G;
    accelerometer[1] = (float)raw[1] * BMI088_ACCEL_6G_SEN_G;
    accelerometer[2] = (float)raw[2] * BMI088_ACCEL_6G_SEN_G;

    BMI088_accel_mps2[0] = accelerometer[0] * BMI088_GRAVITY_MPS2;
    BMI088_accel_mps2[1] = accelerometer[1] * BMI088_GRAVITY_MPS2;
    BMI088_accel_mps2[2] = accelerometer[2] * BMI088_GRAVITY_MPS2;
}

static void bmi088_convert_gyro(const int16_t raw[3])
{
    gyro[0] = (float)raw[0] * BMI088_GYRO_2000_SEN_RAD_S - BMI088_gyro_bias[0];
    gyro[1] = (float)raw[1] * BMI088_GYRO_2000_SEN_RAD_S - BMI088_gyro_bias[1];
    gyro[2] = (float)raw[2] * BMI088_GYRO_2000_SEN_RAD_S - BMI088_gyro_bias[2];
}

/* -------------------------------------------------------------------------- */
/* 姿态解算                                                                   */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* RMCS ImuEkf 姿态解算接口                                                    */
/* -------------------------------------------------------------------------- */

void BMI088_UpdateAngleDeg(void)
{
    INS_angle_deg[0] = INS_angle[0] * BMI088_RAD_TO_DEG;
    INS_angle_deg[1] = INS_angle[1] * BMI088_RAD_TO_DEG;
    INS_angle_deg[2] = INS_angle[2] * BMI088_RAD_TO_DEG;
}

static void bmi088_update_angle(void)
{
    const float q0 = bmi088_ekf.q[0];
    const float q1 = bmi088_ekf.q[1];
    const float q2 = bmi088_ekf.q[2];
    const float q3 = bmi088_ekf.q[3];
    float pitch_sin;

    quat[0] = bmi088_ekf.q[0];
    quat[1] = bmi088_ekf.q[1];
    quat[2] = bmi088_ekf.q[2];
    quat[3] = bmi088_ekf.q[3];

    INS_angle_deg[0] = atan2f(
        2.0f * (q0 * q3 + q1 * q2),
        2.0f * (q0 * q0 + q1 * q1) - 1.0f) * BMI088_RAD_TO_DEG;

    pitch_sin = -2.0f * (q1 * q3 - q0 * q2);
    INS_angle_deg[1] = asinf(bmi088_clampf(pitch_sin, -1.0f, 1.0f)) * BMI088_RAD_TO_DEG;

    INS_angle_deg[2] = atan2f(
        2.0f * (q0 * q1 + q2 * q3),
        2.0f * (q0 * q0 + q3 * q3) - 1.0f) * BMI088_RAD_TO_DEG;

    INS_angle[0] = INS_angle_deg[0] * BMI088_DEG_TO_RAD;
    INS_angle[1] = INS_angle_deg[1] * BMI088_DEG_TO_RAD;
    INS_angle[2] = INS_angle_deg[2] * BMI088_DEG_TO_RAD;
}

static int bmi088_reset_attitude_from_accel(void)
{
    if (!RmcsImuEkf_ResetFromAccel(
            &bmi088_ekf,
            accelerometer[0],
            accelerometer[1],
            accelerometer[2],
            0.0f)) {
        return 0;
    }

    bmi088_update_angle();
    return 1;
}

static void bmi088_ekf_update(
    float gx,
    float gy,
    float gz,
    float ax_g,
    float ay_g,
    float az_g,
    float dt_seconds)
{
    float max_gyro = fabsf(gx);
    if (fabsf(gy) > max_gyro) {
        max_gyro = fabsf(gy);
    }
    if (fabsf(gz) > max_gyro) {
        max_gyro = fabsf(gz);
    }
    if (max_gyro >= BMI088_GYRO_SATURATION_RAD_S) {
        RmcsImuEkf_InflateAttitudeUncertainty(&bmi088_ekf);
    }

    if (!RmcsImuEkf_Predict(&bmi088_ekf, gx, gy, gz, dt_seconds)) {
        return;
    }

    (void)RmcsImuEkf_Correct(&bmi088_ekf, ax_g, ay_g, az_g, 3.0f);
    bmi088_update_angle();
}

/* -------------------------------------------------------------------------- *//* 公开接口                                                                   */
/* -------------------------------------------------------------------------- */

int BMI088_Init(void)
{
    BMI088_ready = 0U;
    bmi088_attitude_initialized = 0U;
    bmi088_temperature_valid = 0U;
    bmi088_last_work_tick_valid = 0U;
    bmi088_last_temperature_ms = HAL_GetTick();

    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
    INS_angle[0] = 0.0f;
    INS_angle[1] = 0.0f;
    INS_angle[2] = 0.0f;
    INS_angle_deg[0] = 0.0f;
    INS_angle_deg[1] = 0.0f;
    INS_angle_deg[2] = 0.0f;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(10U);

    if (!bmi088_probe_accel() || !bmi088_probe_gyro()) {
        return 0;
    }

    /* 加速度计软复位。 */
    if (!bmi088_accel_write(BMI088_ACC_SOFTRESET, 0xB6U)) {
        return 0;
    }
    HAL_Delay(50U);
    if (!bmi088_probe_accel()) {
        return 0;
    }

    /* RMCS 配置：加速度计正常模式，1600 Hz ODR，±6g。 */
    if (!bmi088_accel_write(BMI088_ACC_PWR_CTRL, 0x04U)) {
        return 0;
    }
    HAL_Delay(5U);

    if (!bmi088_accel_write(BMI088_ACC_PWR_CONF, 0x00U)) {
        return 0;
    }
    HAL_Delay(5U);

    if (!bmi088_accel_write(BMI088_ACC_CONF, 0xACU)) {
        return 0;
    }

    if (!bmi088_accel_write(BMI088_ACC_RANGE, 0x01U)) {
        return 0;
    }

    /* 陀螺仪软复位。 */
    if (!bmi088_gyro_write(BMI088_GYRO_SOFTRESET, 0xB6U)) {
        return 0;
    }
    HAL_Delay(30U);
    if (!bmi088_probe_gyro()) {
        return 0;
    }

    /* 陀螺仪 ±2000 dps，2000 Hz ODR / 532 Hz bandwidth。 */
    if (!bmi088_gyro_write(BMI088_GYRO_RANGE, 0x00U)) {
        return 0;
    }

    if (!bmi088_gyro_write(BMI088_GYRO_BANDWIDTH, 0x01U)) {
        return 0;
    }

    if (!bmi088_gyro_write(BMI088_GYRO_LPM1, 0x00U)) {
        return 0;
    }

    BMI088_gyro_bias[0] = 0.0f;
    BMI088_gyro_bias[1] = 0.0f;
    BMI088_gyro_bias[2] = 0.0f;

    RmcsImuEkf_Init(&bmi088_ekf);

    bmi088_last_work_tick_ms = HAL_GetTick();
    bmi088_last_work_tick_valid = 1U;
    BMI088_ready = 1U;

    return 1;
}

int BMI088_Update(float dt_seconds)
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    uint32_t now_ms;

    if (BMI088_ready == 0U) {
        return 0;
    }

    if (!bmi088_read_accel_raw(accel_raw) || !bmi088_read_gyro_raw(gyro_raw)) {
        return 0;
    }

    bmi088_convert_accel(accel_raw);
    bmi088_convert_gyro(gyro_raw);

    now_ms = HAL_GetTick();
    if ((bmi088_temperature_valid == 0U)
        || ((now_ms - bmi088_last_temperature_ms) >= 100U)) {
        float temperature;
        if (bmi088_read_temperature(&temperature)) {
            BMI088_temperature_c = temperature;
            bmi088_temperature_valid = 1U;
            bmi088_last_temperature_ms = now_ms;
        }
    }

    if (bmi088_attitude_initialized == 0U) {
        if (bmi088_reset_attitude_from_accel()) {
            bmi088_attitude_initialized = 1U;
        }
    }

    if (bmi088_attitude_initialized != 0U
        && isfinite(dt_seconds)
        && (dt_seconds > 0.0f)
        && (dt_seconds <= BMI088_DT_MAX)) {
        bmi088_ekf_update(
            gyro[0], gyro[1], gyro[2],
            accelerometer[0], accelerometer[1], accelerometer[2],
            dt_seconds);
    }

    return 1;
}

void BMI088_work(void)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;
    float dt_seconds;

    if (BMI088_ready == 0U) {
        return;
    }

    now_ms = HAL_GetTick();

    if (bmi088_last_work_tick_valid == 0U) {
        bmi088_last_work_tick_ms = now_ms;
        bmi088_last_work_tick_valid = 1U;
        return;
    }

    elapsed_ms = now_ms - bmi088_last_work_tick_ms;
    if (elapsed_ms == 0U) {
        return;
    }

    bmi088_last_work_tick_ms = now_ms;
    dt_seconds = (float)elapsed_ms * 0.001f;

    /* 如果调度器停顿太久，丢掉这次积分，下一帧用加速度重新对齐姿态。 */
    if (dt_seconds > BMI088_DT_MAX) {
        bmi088_attitude_initialized = 0U;
        dt_seconds = 0.001f;
    }

    (void)BMI088_Update(dt_seconds);
}

int BMI088_CalibrateGyroBias(uint16_t sample_count, uint16_t delay_ms)
{
    int16_t raw[3];
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    uint16_t i;
    double inv_count;

    if ((BMI088_ready == 0U) || (sample_count == 0U)) {
        return 0;
    }

    for (i = 0U; i < sample_count; ++i) {
        if (!bmi088_read_gyro_raw(raw)) {
            return 0;
        }

        sum_x += (double)raw[0] * (double)BMI088_GYRO_2000_SEN_RAD_S;
        sum_y += (double)raw[1] * (double)BMI088_GYRO_2000_SEN_RAD_S;
        sum_z += (double)raw[2] * (double)BMI088_GYRO_2000_SEN_RAD_S;

        if (delay_ms != 0U) {
            HAL_Delay(delay_ms);
        }
    }

    inv_count = 1.0 / (double)sample_count;
    BMI088_gyro_bias[0] = (float)(sum_x * inv_count);
    BMI088_gyro_bias[1] = (float)(sum_y * inv_count);
    BMI088_gyro_bias[2] = (float)(sum_z * inv_count);

    return 1;
}

void BMI088_ResetGyroBias(void)
{
    BMI088_gyro_bias[0] = 0.0f;
    BMI088_gyro_bias[1] = 0.0f;
    BMI088_gyro_bias[2] = 0.0f;
}

void BMI088_GetAngle(float *yaw, float *pitch, float *roll)
{
    if (yaw != NULL) {
        *yaw = INS_angle[0];
    }
    if (pitch != NULL) {
        *pitch = INS_angle[1];
    }
    if (roll != NULL) {
        *roll = INS_angle[2];
    }
}

void BMI088_GetAngleDeg(float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    if (yaw_deg != NULL) {
        *yaw_deg = INS_angle_deg[0];
    }
    if (pitch_deg != NULL) {
        *pitch_deg = INS_angle_deg[1];
    }
    if (roll_deg != NULL) {
        *roll_deg = INS_angle_deg[2];
    }
}

void BMI088_GetQuaternion(float q[4])
{
    if (q != NULL) {
        q[0] = quat[0];
        q[1] = quat[1];
        q[2] = quat[2];
        q[3] = quat[3];
    }
}

void BMI088_GetGyroBias(float bias[3])
{
    if (bias != NULL) {
        bias[0] = BMI088_gyro_bias[0];
        bias[1] = BMI088_gyro_bias[1];
        bias[2] = BMI088_gyro_bias[2];
    }
}




















