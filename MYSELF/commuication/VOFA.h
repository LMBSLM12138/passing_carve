#ifndef __VOFA_UART_H
#define __VOFA_UART_H

#include "main.h"

// 发送函数：传入PID三个值
void VOFA_SendPID(float target, float real, float pid_out, float current, float error);

extern int16_t VOFA_TxBuf[12];
extern uint8_t VOFA_TxBusy;






#endif

