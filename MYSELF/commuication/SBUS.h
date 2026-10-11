#ifndef __SBUS_H
#define __SBUS_H

#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#define RC_PACKET_SIZE      7U
#define SBUS_RX_BUF_NUM     50U
#define RC_FRAME_LENGTH     25U
#define RC_CH_VALUE_OFFSET  1024U

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

typedef struct
{
    int16_t Start;
    int16_t Ch1;
    int16_t Ch2;
    int16_t Ch3;
    int16_t Ch4;
    int16_t SA;
    int16_t SB;
    int16_t SC;
    int16_t SD;
    int16_t SE;
    int16_t SF;
    int16_t SG;
    int16_t SH;
    int16_t LD;
    int16_t RD;
    int16_t LS;
    int16_t RS;
} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;
extern int16_t speed;
extern int16_t speed1;
extern uint8_t a[7];

void SUBUS_init(void);

#endif
