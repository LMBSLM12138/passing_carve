/*********************************************************
* 函数名：VOFA_Send_PID
* 描  述：通过UART7发送PID数据到VOFA上位机(JustFloat协议)
* 参  数：set      设定值
*         feedback 实际反馈值
*         pid_out  PID计算输出
* 返回值：无
*********************************************************/

#include "main.h"
#include "usart.h"
#include "VOFA.h"

int16_t VOFA_TxBuf[12];  // 12 * 2 = 24 字节
uint8_t VOFA_TxBusy = 0; // 发送忙标志

void VOFA_SendPID(float target, float real, float pid_out, float current, float error)
{
    uint8_t buf[24] = {0};  // 24字节 足够
    float data[5] = {target, real, pid_out, current, error};

    // 浮点转字节（JustFloat 标准）
    for(int i=0; i<5; i++)
    {
        uint8_t *p = (uint8_t*)&data[i];
        buf[i*4+0] = p[0];
        buf[i*4+1] = p[1];
        buf[i*4+2] = p[2];
        buf[i*4+3] = p[3];
    }

    // 帧尾
    buf[20] = 0x00;
    buf[21] = 0x00;
    buf[22] = 0x80;
    buf[23] = 0x7F;

    // 关键：短超时，绝对不会卡死
    HAL_UART_Transmit(&huart7, buf, 24, 2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart7)
    {
        VOFA_TxBusy = 0;  // 发送完成，可以发下一包
    }
}