#include "vesc_dri.h"
#include "MY_fdcan.h"

VESC_DataTypeDef vesc_data[2][VESC_ID_MAX];

void VESC_Init(void)
{
    MY_fdcan_init();
}

void VESC_CAN_Transmit(FDCAN_HandleTypeDef *hfdcan,
                       vesc_can_transmit_msg_command_e command,
                       uint32_t data,
                       uint8_t myid)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t tx_data[4];

    if (hfdcan == NULL)
    {
        return;
    }

    tx_data[0] = (uint8_t)((data >> 24) & 0xFFU);
    tx_data[1] = (uint8_t)((data >> 16) & 0xFFU);
    tx_data[2] = (uint8_t)((data >> 8) & 0xFFU);
    tx_data[3] = (uint8_t)(data & 0xFFU);

    tx.FDFormat           = FDCAN_CLASSIC_CAN;
    tx.TxFrameType        = FDCAN_DATA_FRAME;
    tx.IdType             = FDCAN_EXTENDED_ID;
    tx.MessageMarker      = 0U;
    tx.BitRateSwitch      = FDCAN_BRS_OFF;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.Identifier         = ((uint32_t)command << 8) | (uint32_t)myid;
    tx.DataLength         = FDCAN_DLC_BYTES_4;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, tx_data);
}
