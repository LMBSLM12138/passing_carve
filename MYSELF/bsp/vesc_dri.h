#ifndef __VESC_DRI_H
#define __VESC_DRI_H

#include "fdcan.h"

#define VESC_ID_MAX  8U

/* VESC CAN status packet IDs. */
typedef enum
{
    CAN_MESSAGE_STATUES1 = 9,
    CAN_MESSAGE_STATUES2 = 14,
    CAN_MESSAGE_STATUES3 = 15,
    CAN_MESSAGE_STATUES4 = 16,
    CAN_MESSAGE_STATUES5 = 27,
    CAN_MESSAGE_STATUES6 = 58
} vesc_can_receive_msg_command_e;

/* VESC CAN control packet IDs. */
typedef enum
{
    CAN_SET_DUTY = 0,
    CAN_SET_CURRENT = 1,
    CAN_SET_BRAKE_CURRENT = 2,
    CAN_SET_RPM = 3,
    CAN_SET_POSITION = 4
} vesc_can_transmit_msg_command_e;

typedef struct
{
    uint8_t id;
    uint8_t bus;
    struct
    {
        int32_t rpm;
        int16_t current;
        int16_t duty;
    } status1;
    struct
    {
        uint32_t amp_hours;
        uint32_t amp_hours_charged;
    } status2;
    struct
    {
        uint32_t watt_hours;
        uint32_t watt_hours_charged;
    } status3;
    struct
    {
        int16_t temp_fet;
        int16_t temp_motor;
        int16_t current_in;
        int16_t pid_pos_now;
    } status4;
    struct
    {
        uint32_t tacho_value;
        uint16_t v_in;
        uint16_t rec_0;
    } status5;
    struct
    {
        uint16_t adc1;
        uint16_t adc2;
        uint16_t adc3;
        uint16_t ppm;
    } status6;
} VESC_DataTypeDef;

/* vesc_data[bus][id] */
extern VESC_DataTypeDef vesc_data[2][VESC_ID_MAX];

void VESC_Init(void);
void VESC_CAN_Transmit(FDCAN_HandleTypeDef *hfdcan,
                       vesc_can_transmit_msg_command_e command,
                       uint32_t data,
                       uint8_t myid);

#endif
