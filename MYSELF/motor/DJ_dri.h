#ifndef __DJ_DRI_H
#define __DJ_DRI_H    

#include "main.h"
#include "fdcan.h"
#include "MY_fdcan.h"
#include "vesc_dri.h"

void DJ_set_current(FDCAN_HandleTypeDef *hfdcan,int16_t current_set[4],uint32_t myid);
//主要用于6020的电压模式来设置电压(-25000~25000)
void DJ_set_voltage(FDCAN_HandleTypeDef *hfdcan,int16_t voltage_set[4],uint32_t myid);
void DJ_set_snail2305(FDCAN_HandleTypeDef *hfdcan,vesc_can_transmit_msg_command_e command,float data,uint8_t myid);


#endif
