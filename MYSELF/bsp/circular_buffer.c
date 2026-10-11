#include "circular_buffer.h"


/**
 * @brief  计算距离缓冲区末尾的连续空间大小
 * @param  pos  当前指针位置
 * @return 从pos到缓冲区末尾的字节数
 */
static inline uint32_t rb_bsp_contig_size(uint32_t pos)
{
    return RB_BSP_BUFFER_SIZE - (pos & RB_BSP_BUFFER_MASK);
}

/* ========================== 公共函数实现 ========================== */

void rb_bsp_init(rb_bsp_t *rb)
{
    if (rb == NULL) {
        return;
    }
    memset(rb->buffer, 0, sizeof(rb->buffer));
    rb->head = 0;
    rb->tail = 0;
}

uint32_t rb_bsp_write(rb_bsp_t *rb, const uint8_t *data, uint32_t len)
{
    if (rb == NULL || data == NULL || len == 0) {
        return 0;
    }

    /* 计算当前可写空间：保留一个字节用于区分满/空状态，或使用镜像索引。
       此处使用 head 和 tail 均为单调递增的 uint32_t，通过掩码访问实际位置，
       因此不需要保留空字节，但需要确保写入后 head - tail <= BUFFER_SIZE。
       为了无锁安全，我们计算空闲空间：free = BUFFER_SIZE - (head - tail)。
       由于 head 和 tail 在单生产者单消费者下不会同时修改，读取时可能略有偏差，
       但不会导致数据损坏。此处为精确控制，先读取 tail，再计算。 */
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    uint32_t used = head - tail;                /* 已使用字节数 */
    uint32_t free = RB_BSP_BUFFER_SIZE - used;  /* 剩余可写字节数 */

    uint32_t write_len = (len < free) ? len : free;
    if (write_len == 0) {
        return 0;
    }

    /* 分两段写入，处理回绕 */
    uint32_t pos = head & RB_BSP_BUFFER_MASK;
    uint32_t first_chunk = rb_bsp_contig_size(pos);
    if (first_chunk > write_len) {
        first_chunk = write_len;
    }

    memcpy(&rb->buffer[pos], data, first_chunk);

    if (write_len > first_chunk) {
        uint32_t second_chunk = write_len - first_chunk;
        memcpy(rb->buffer, data + first_chunk, second_chunk);
    }

    /* 更新写指针（使用内存屏障确保数据先写入） */
    __sync_synchronize();
    rb->head = head + write_len;

    return write_len;
}

// 读取数据
uint32_t rb_bsp_read(rb_bsp_t *rb, uint8_t *data, uint32_t len)
{
    if (rb == NULL || data == NULL || len == 0) {
        return 0;
    }

    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    uint32_t used = head - tail;                /* 可读字节数 */

    uint32_t read_len = (len < used) ? len : used;
    if (read_len == 0) {
        return 0;
    }

    /* 分两段读出，处理回绕 */
    uint32_t pos = tail & RB_BSP_BUFFER_MASK;
    uint32_t first_chunk = rb_bsp_contig_size(pos);
    if (first_chunk > read_len) {
        first_chunk = read_len;
    }

    memcpy(data, &rb->buffer[pos], first_chunk);

    if (read_len > first_chunk) {
        uint32_t second_chunk = read_len - first_chunk;
        memcpy(data + first_chunk, rb->buffer, second_chunk);
    }

    /* 更新读指针 */
    __sync_synchronize();
    rb->tail = tail + read_len;

    return read_len;
}

// 获取当前可读字节数
uint32_t rb_bsp_available(const rb_bsp_t *rb)
{
    if (rb == NULL) {
        return 0;
    }
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    return head - tail;
}

// 获取当前可写字节数
uint32_t rb_bsp_free(const rb_bsp_t *rb)
{
    if (rb == NULL) {
        return 0;
    }
    return RB_BSP_BUFFER_SIZE - rb_bsp_available(rb);
}

// 清空缓冲区
void rb_bsp_flush(rb_bsp_t *rb)
{
    if (rb == NULL) {
        return;
    }
    /* 在单生产者单消费者场景下，直接设置 tail = head 即可清空。
       但为了安全，先读取 head，再更新 tail。 */
    uint32_t head = rb->head;
    __sync_synchronize();
    rb->tail = head;
}