/**
 * @file sys_timestamp.h
 * @brief 时间计算相关支持库 (C语言版, 仅保留USB所需功能)
 */

#ifndef SYS_TIMESTAMP_H
#define SYS_TIMESTAMP_H

/* Includes ------------------------------------------------------------------*/

#include "tim.h"
#include "stm32h7xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 系统时间戳结构体
 */
typedef struct
{
    // 绑定的定时器 (PSC后为1MHz, ARR为3600000000, 1h溢出一次)
    TIM_HandleTypeDef *TIM_Handler;

    // 定时器溢出计数, 一小时溢出一次
    uint32_t TIM_Overflow_Count;

} Struct_Timestamp;

/* Exported variables --------------------------------------------------------*/

extern Struct_Timestamp SYS_Timestamp;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化时间戳, 绑定定时器
 *
 * @param htim 定时器句柄, psc后为1MHz, arr要求3600000000(32位)
 */
void SYS_Timestamp_Init(TIM_HandleTypeDef *htim);

/**
 * @brief 获取当前时间戳, 单位微秒(64位)
 *
 * @return uint64_t 当前时间戳
 */
uint64_t SYS_Timestamp_Get_Current_Timestamp(void);

/**
 * @brief TIM定时器溢出回调, 每3600s由外部中断调用一次
 */
void SYS_Timestamp_TIM_3600s_PeriodElapsedCallback(void);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/