#include "DJ_dri.h"


//主要是用于3508和6020的电流模式来设置电流(-16384~16384,20A~20A,3A~3A)
void DJ_set_current(FDCAN_HandleTypeDef *hfdcan,int16_t current_set[4],uint32_t myid)
{
    uint8_t data_out[8] = {0};
    // 填充 4 个电机的电流值
    for(int i = 0; i < 4; i++) {
        data_out[i * 2] = (current_set[i] >> 8);
        data_out[i * 2 + 1] = current_set[i];
    }
    
      MY_fdcan_send(hfdcan,
                  data_out[0], data_out[1], data_out[2], data_out[3],
                  data_out[4], data_out[5], data_out[6], data_out[7],
                  myid);
}
//主要用于6020的电压模式来设置电压(-25000~25000)
void DJ_set_voltage(FDCAN_HandleTypeDef *hfdcan,int16_t voltage_set[4],uint32_t myid)
{
    uint8_t data_out[8] = {0};
        // 填充 4 个电机的电流值
        for(int i = 0; i < 4; i++) {
            data_out[i * 2] = (voltage_set[i] >> 8);
            data_out[i * 2 + 1] = voltage_set[i];
        }
        
        MY_fdcan_send(hfdcan,
                    data_out[0], data_out[1], data_out[2], data_out[3],
                    data_out[4], data_out[5], data_out[6], data_out[7],
                    myid);
}
/*
占空比设置单位是%，即20%，参数就填0.2   
设置电流单位是A
设置刹车电流单位是A
设置转速单位是rpm
设置位置单位是°，即30°，就填30
*/


void DJ_set_snail2305(FDCAN_HandleTypeDef *hfdcan,vesc_can_transmit_msg_command_e command,float data,uint8_t myid)
{   
    switch(command)
    {
        case CAN_SET_DUTY:
            data =data*100000;
            break;
        case CAN_SET_CURRENT:
            data =data*1000;
            break;
        case CAN_SET_BRAKE_CURRENT:
            data =data*1000;
            break;
        case CAN_SET_RPM:
            data =data*1;
            break;
        case CAN_SET_POSITION:
            data =data*1000000;
            break;
        default:
            break;
    }
    data = (uint32_t)data;

    VESC_CAN_Transmit(hfdcan,command,data,myid);
}
