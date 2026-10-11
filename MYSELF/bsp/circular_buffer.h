#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ========================== 配置参数 ========================== */
#define RB_BSP_BUFFER_SIZE      256     /* 环形缓冲区总容量（字节），必须是2的幂次 */
#define RB_BSP_BUFFER_MASK      (RB_BSP_BUFFER_SIZE - 1)

/* ========================== 数据结构 ========================== */
typedef struct {
    uint8_t  buffer[RB_BSP_BUFFER_SIZE]; /* 数据存储区 */
    volatile uint32_t head;              /* 写指针（生产者） */
    volatile uint32_t tail;              /* 读指针（消费者） */
} rb_bsp_t;

/* ========================== 函数原型 ========================== */

/**
 * @brief  初始化环形缓冲区
 * @param  rb  环形缓冲区对象指针
 */
void rb_bsp_init(rb_bsp_t *rb);

/**
 * @brief  向环形缓冲区写入多个字节
 * @param  rb      环形缓冲区对象指针
 * @param  data    待写入数据指针
 * @param  len     请求写入的字节数
 * @return 实际写入的字节数（可能小于len，当剩余空间不足时）
 */
uint32_t rb_bsp_write(rb_bsp_t *rb, const uint8_t *data, uint32_t len);

/**
 * @brief  从环形缓冲区读出多个字节
 * @param  rb      环形缓冲区对象指针
 * @param  data    接收数据缓冲区指针
 * @param  len     请求读出的字节数
 * @return 实际读出的字节数（可能小于len，当可读数据不足时）
 */
uint32_t rb_bsp_read(rb_bsp_t *rb, uint8_t *data, uint32_t len);

/**
 * @brief  获取当前可读数据字节数
 * @param  rb  环形缓冲区对象指针
 * @return 可读字节数
 */
uint32_t rb_bsp_available(const rb_bsp_t *rb);

/**
 * @brief  获取当前剩余可写空间字节数
 * @param  rb  环形缓冲区对象指针
 * @return 可写字节数
 */
uint32_t rb_bsp_free(const rb_bsp_t *rb);

/**
 * @brief  清空环形缓冲区
 * @param  rb  环形缓冲区对象指针
 */
void rb_bsp_flush(rb_bsp_t *rb);


#endif
