#ifndef __MY_FDCAN_H
#define __MY_FDCAN_H

#include <stdint.h>
#include "fdcan.h"

/* DM feedback CAN IDs. */
#define DM_CAN1_FEEDBACK_ID  0x001U
#define DM_CAN2_FEEDBACK_ID  0x001U

/* DM-J4310 feedback mapping range. */
#define DM_POS_MIN_RAD     (-12.5f)
#define DM_POS_MAX_RAD     ( 12.5f)
#define DM_VEL_MIN_RADS    (-30.0f)
#define DM_VEL_MAX_RADS    ( 30.0f)
#define DM_TORQUE_MIN_NM   (-10.0f)
#define DM_TORQUE_MAX_NM   ( 10.0f)

#define CAN_BUS_1  0U
#define CAN_BUS_2  1U

/* Common feedback for DJI 3508/6020. */
typedef struct
{
    uint8_t  bus;
    uint8_t  motor_id;
    uint16_t ecd;
    int16_t  speed_rpm;
    int16_t  current;
    uint8_t  temperature;
} DJI_Feedback_t;

/* DM-J4310-2EC feedback. */
typedef struct
{
    uint8_t  bus;
    uint16_t master_id;
    uint8_t  motor_id;
    uint8_t  err;

    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t torque_raw;

    float position_rad;
    float velocity_rad_s;
    float torque_nm;

    uint8_t  temperature_mos;
    uint8_t  temperature_rotor;
    uint32_t timestamp_ms;
} DM_Feedback_t;

extern DJI_Feedback_t dji3508_fb[2][4];
extern DJI_Feedback_t dji6020_fb[2][4];
extern DM_Feedback_t  dm_fb[2];

void MY_fdcan_init(void);

void MY_fdcan_send(FDCAN_HandleTypeDef *hfdcan,
                   uint8_t mydata1, uint8_t mydata2, uint8_t mydata3, uint8_t mydata4,
                   uint8_t mydata5, uint8_t mydata6, uint8_t mydata7, uint8_t mydata8,
                   uint32_t myid);

void DM_Feedback_RxCallback(FDCAN_HandleTypeDef *hfdcan,
                            const DM_Feedback_t *feedback);

#endif
