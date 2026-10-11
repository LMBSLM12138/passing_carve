#include "DM_dri_4310.h"


void DM_dri_enable(FDCAN_HandleTypeDef *hfdcan)
{
    MY_fdcan_send(hfdcan,
                  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
                  0x101U);
}

/**
 * @brief  组装位置速度模式CAN发送数据段(8字节)
 * @param  hfdcan: FDCAN1 或 FDCAN2
 * @param  pos_rad: 目标位置，rad
 * @param  vel_rads: 目标速度，rad/s
 * @param  myid: CAN 标准帧 ID
 */
void DM_dri_po_and_sp(FDCAN_HandleTypeDef *hfdcan,
                      float pos_rad,
                      float vel_rads,
                      uint32_t myid)
{
    uint8_t data_out[8] = {0};

    /* D[0]-D[3]: 位置 p_des，小端 float。 */
    memcpy(&data_out[0], &pos_rad, 4);

    /* D[4]-D[7]: 速度 v_des，小端 float。 */
    memcpy(&data_out[4], &vel_rads, 4);

    MY_fdcan_send(hfdcan,
                  data_out[0], data_out[1], data_out[2], data_out[3],
                  data_out[4], data_out[5], data_out[6], data_out[7],
                  myid);
}

//归一化函数
static uint16_t mit_norm(float x, float min, float span, uint16_t full)
{
    float n = (x - min) / span;
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return (uint16_t)(n * (float)full);
}

void DM_dri_MIT_MODE(FDCAN_HandleTypeDef *hfdcan,float pos, float vel, float kp, float kd, float torque, uint32_t myid)
{
    uint16_t p = mit_norm(pos,    -P_MAX,   2.0f * P_MAX, 65535u);
    uint16_t v = mit_norm(vel,    -V_MAX,   2.0f * V_MAX, 4095u);
    uint16_t k = mit_norm(kp,      0.0f,    KP_MAX,       4095u);
    uint16_t d = mit_norm(kd,      0.0f,    KD_MAX,       4095u);
    uint16_t t = mit_norm(torque, -T_MAX,   2.0f * T_MAX, 4095u);
    
    uint8_t data_out[8] = {0};
    data_out[0] = (uint8_t)(p >> 8);                        /* p[15:8]            */
    data_out[1] = (uint8_t)(p & 0xFF);                      /* p[7:0]             */
    data_out[2] = (uint8_t)(v >> 4);                        /* v[11:4]            */
    data_out[3] = (uint8_t)(((v & 0x0F) << 4) | (k >> 8));  /* v[3:0] | Kp[11:8]  */
    data_out[4] = (uint8_t)(k & 0xFF);                      /* Kp[7:0]            */
    data_out[5] = (uint8_t)(d >> 4);                        /* Kd[11:4]           */
    data_out[6] = (uint8_t)(((d & 0x0F) << 4) | (t >> 8));  /* Kd[3:0] | T[11:8]  */
    data_out[7] = (uint8_t)(t & 0xFF);                      /* T[7:0]             */

    MY_fdcan_send(hfdcan,
                  data_out[0], data_out[1], data_out[2], data_out[3],
                  data_out[4], data_out[5], data_out[6], data_out[7],
                  myid);
}