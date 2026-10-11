#include "SBUS.h"

RC_ctrl_t rc_ctrl = {0};
int16_t speed = 0;
int16_t speed1 = 0;
uint8_t a[7] = {0};

static enum
{
    SBUS_WAIT_START = 0,
    SBUS_IN_FRAME
} sbus_state = SBUS_WAIT_START;

static uint8_t frame[RC_FRAME_LENGTH];
static uint8_t frame_idx = 0U;
static uint8_t * const sbus_rx_buf = (uint8_t *)0x24000000U;  /* AXI SRAM */

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static void sbus_parse_stream(const uint8_t *src, uint16_t len);

void SUBUS_init(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5,
                                     sbus_rx_buf,
                                     SBUS_RX_BUF_NUM) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * SBUS 的 16 个通道连续占用 D[1]~D[22]。
 * 每个通道 11 位，从 bit0 开始依次排列。
 */
static uint16_t sbus_get_channel(const uint8_t *sbus_buf, uint8_t channel)
{
    uint16_t bit_index = (uint16_t)channel * 11U;
    uint8_t byte_index = (uint8_t)(1U + (bit_index >> 3));
    uint8_t bit_offset = (uint8_t)(bit_index & 0x07U);

    uint32_t value = ((uint32_t)sbus_buf[byte_index]) |
                     ((uint32_t)sbus_buf[byte_index + 1U] << 8) |
                     ((uint32_t)sbus_buf[byte_index + 2U] << 16);

    return (uint16_t)((value >> bit_offset) & 0x07FFU);
}
//处理函数
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf[0] != 0x0FU)
    {
        return;
    }

    rc_ctrl->Start = sbus_buf[0];

    rc_ctrl->Ch1 = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 0U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch2 = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 1U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch3 = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 2U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->Ch4 = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 3U) - (int16_t)RC_CH_VALUE_OFFSET;

    rc_ctrl->SA = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 4U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->SB = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 5U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->SC = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 6U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->SD = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 7U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->SE = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 8U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->SF = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 9U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->SG = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 10U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->SH = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 11U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->LD = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 12U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->RD = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 13U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->LS = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 14U) - (int16_t)RC_CH_VALUE_OFFSET;
    rc_ctrl->RS = (int16_t)sbus_get_channel((const uint8_t *)sbus_buf, 15U) - (int16_t)RC_CH_VALUE_OFFSET;
}

//接收函数
static void sbus_parse_stream(const uint8_t *src, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        uint8_t b = src[i];

        switch (sbus_state)
        {
            case SBUS_WAIT_START:
                if (b == 0x0FU)
                {
                    frame[0] = b;
                    frame_idx = 1U;
                    sbus_state = SBUS_IN_FRAME;
                }
                break;

            case SBUS_IN_FRAME:
                frame[frame_idx++] = b;
                if (frame_idx == RC_FRAME_LENGTH)
                {
                    sbus_to_rc(frame, &rc_ctrl);
                    frame_idx = 0U;
                    sbus_state = SBUS_WAIT_START;

                    speed = rc_ctrl.Ch3 * (-4);
                    speed1 = rc_ctrl.Ch4 * 4;
                }
                break;

            default:
                sbus_state = SBUS_WAIT_START;
                break;
        }
    }
}

//SBUS进中断接收数据
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {

			if (Size > 0U)
			{
					sbus_parse_stream(sbus_rx_buf, Size);
			}

			(void)HAL_UARTEx_ReceiveToIdle_DMA(huart,
																				 sbus_rx_buf,
																				 SBUS_RX_BUF_NUM);
	}
}
