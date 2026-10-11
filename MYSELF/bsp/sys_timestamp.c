/**
 * @file sys_timestamp.c
 * @brief 时间计算相关支持库 (C语言版)
 *
 * @note 要求: 使能并绑定一个定时器, 开外部中断, PSC分频到1MHz, ARR为3600000000
 *       保证arr计数器1us增一次, 1h触发一次外部中断
 */

/* Includes ------------------------------------------------------------------*/

#include "sys_timestamp.h"

/* Private variables ---------------------------------------------------------*/

Struct_Timestamp SYS_Timestamp = {0};

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 计算当前时间戳, 单位微秒
 */
static uint64_t SYS_Timestamp_Calculate_Timestamp(void)
{
    // 当前硬件计数值
    uint32_t arr_counter = SYS_Timestamp.TIM_Handler->Instance->CNT;

    // 溢出次数 * 1小时 + 当前计数值
    return ((uint64_t)(SYS_Timestamp.TIM_Overflow_Count) * 3600000000ULL + (uint64_t)arr_counter);
}

/**
 * @brief 初始化时间戳
 */
void SYS_Timestamp_Init(TIM_HandleTypeDef *htim)
{
    SYS_Timestamp.TIM_Handler = htim;
}

/**
 * @brief TIM定时器回调, 每3600s调用一次
 */
void SYS_Timestamp_TIM_3600s_PeriodElapsedCallback(void)
{
    SYS_Timestamp.TIM_Overflow_Count++;
}

/**
 * @brief 获取当前时间戳, 单位微秒
 */
uint64_t SYS_Timestamp_Get_Current_Timestamp(void)
{
    return (SYS_Timestamp_Calculate_Timestamp());
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/