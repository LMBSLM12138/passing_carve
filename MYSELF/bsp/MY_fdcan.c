#include "MY_fdcan.h"
#include "vesc_dri.h"
#include <stddef.h>

DJI_Feedback_t dji3508_fb[2][4];
DJI_Feedback_t dji6020_fb[2][4];
DM_Feedback_t  dm_fb[2];

//DM线性解码反馈
static float DM_UintToFloat(uint32_t value, float min_value, float max_value, uint8_t bits)
{
    float max_int = (float)((1UL << bits) - 1UL);

    return ((float)value * (max_value - min_value) / max_int) + min_value;
}

//DM接收反馈函数
static uint8_t DM_DecodeFeedback(const FDCAN_RxHeaderTypeDef *rx,
                                 const uint8_t data[8],
                                 DM_Feedback_t *feedback)
{
    if ((rx == NULL) || (data == NULL) || (feedback == NULL))
    {
        return 0U;
    }

    if ((rx->IdType != FDCAN_STANDARD_ID) ||
        (rx->DataLength != FDCAN_DLC_BYTES_8))
    {
        return 0U;
    }

    feedback->master_id = (uint16_t)rx->Identifier;

    feedback->motor_id = data[0] & 0x0FU;
    feedback->err      = (data[0] >> 4) & 0x0FU;

    feedback->pos_raw = ((uint16_t)data[1] << 8) | data[2];
    feedback->vel_raw = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    feedback->torque_raw = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];

    feedback->position_rad = DM_UintToFloat(feedback->pos_raw,
                                            DM_POS_MIN_RAD,
                                            DM_POS_MAX_RAD,
                                            16U);
    feedback->velocity_rad_s = DM_UintToFloat(feedback->vel_raw,
                                              DM_VEL_MIN_RADS,
                                              DM_VEL_MAX_RADS,
                                              12U);
    feedback->torque_nm = DM_UintToFloat(feedback->torque_raw,
                                         DM_TORQUE_MIN_NM,
                                         DM_TORQUE_MAX_NM,
                                         12U);

    feedback->temperature_mos   = data[6];
    feedback->temperature_rotor = data[7];
    feedback->timestamp_ms      = HAL_GetTick();

    return 1U;
}

//DJI解码反馈函数
static void DJI_Decode(DJI_Feedback_t *fb,
                       uint8_t bus,
                       uint8_t motor_id,
                       const uint8_t data[8])
{
    fb->bus         = bus;
    fb->motor_id    = motor_id;
    fb->ecd         = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    fb->speed_rpm   = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    fb->current     = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    fb->temperature = data[6];
}

//VESC解码反馈函数
static void VESC_Decode(FDCAN_HandleTypeDef *hfdcan,
                        const FDCAN_RxHeaderTypeDef *rx,
                        const uint8_t data[8])
{
    uint8_t id;
    uint8_t bus;
    uint8_t command;
    VESC_DataTypeDef *v;

    if ((hfdcan != &hfdcan1) && (hfdcan != &hfdcan2))
    {
        return;
    }

    if ((rx->IdType != FDCAN_EXTENDED_ID) ||
        (rx->DataLength != FDCAN_DLC_BYTES_8))
    {
        return;
    }

    id      = (uint8_t)(rx->Identifier & 0xFFU);
    command = (uint8_t)((rx->Identifier >> 8) & 0xFFU);
    bus     = (hfdcan == &hfdcan1) ? CAN_BUS_1 : CAN_BUS_2;

    if (id >= VESC_ID_MAX)
    {
        return;
    }

    v = &vesc_data[bus][id];
    v->id  = id;
    v->bus = bus;

    switch (command)
    {
        case CAN_MESSAGE_STATUES1:
            v->status1.rpm =
                (int32_t)(((uint32_t)data[0] << 24) |
                          ((uint32_t)data[1] << 16) |
                          ((uint32_t)data[2] << 8)  |
                          ((uint32_t)data[3]));
            v->status1.current =
                (int16_t)(((uint16_t)data[4] << 8) | data[5]);
            v->status1.duty =
                (int16_t)(((uint16_t)data[6] << 8) | data[7]);
            break;

        case CAN_MESSAGE_STATUES2:
            v->status2.amp_hours =
                (uint32_t)(((uint32_t)data[0] << 24) |
                           ((uint32_t)data[1] << 16) |
                           ((uint32_t)data[2] << 8)  |
                           ((uint32_t)data[3]));
            v->status2.amp_hours_charged =
                (uint32_t)(((uint32_t)data[4] << 24) |
                           ((uint32_t)data[5] << 16) |
                           ((uint32_t)data[6] << 8)  |
                           ((uint32_t)data[7]));
            break;

        case CAN_MESSAGE_STATUES3:
            v->status3.watt_hours =
                (uint32_t)(((uint32_t)data[0] << 24) |
                           ((uint32_t)data[1] << 16) |
                           ((uint32_t)data[2] << 8)  |
                           ((uint32_t)data[3]));
            v->status3.watt_hours_charged =
                (uint32_t)(((uint32_t)data[4] << 24) |
                           ((uint32_t)data[5] << 16) |
                           ((uint32_t)data[6] << 8)  |
                           ((uint32_t)data[7]));
            break;

        case CAN_MESSAGE_STATUES4:
            v->status4.temp_fet =
                (int16_t)(((uint16_t)data[0] << 8) | data[1]);
            v->status4.temp_motor =
                (int16_t)(((uint16_t)data[2] << 8) | data[3]);
            v->status4.current_in =
                (int16_t)(((uint16_t)data[4] << 8) | data[5]);
            v->status4.pid_pos_now =
                (int16_t)(((uint16_t)data[6] << 8) | data[7]);
            break;

        case CAN_MESSAGE_STATUES5:
            v->status5.tacho_value =
                (uint32_t)(((uint32_t)data[0] << 24) |
                           ((uint32_t)data[1] << 16) |
                           ((uint32_t)data[2] << 8)  |
                           ((uint32_t)data[3]));
            v->status5.v_in =
                (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
            v->status5.rec_0 =
                (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
            break;

        case CAN_MESSAGE_STATUES6:
            v->status6.adc1 =
                (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
            v->status6.adc2 =
                (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
            v->status6.adc3 =
                (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
            v->status6.ppm =
                (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
            break;

        default:
            break;
    }
}

//CAN初始化函数
static void MY_fdcan_start_one(FDCAN_HandleTypeDef *hfdcan, IRQn_Type irq)
{
    (void)HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_REJECT_REMOTE,
                                       FDCAN_REJECT_REMOTE);

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(irq, 5, 0);
    HAL_NVIC_EnableIRQ(irq);
}

//CAN初始化函数
void MY_fdcan_init(void)
{
    MY_fdcan_start_one(&hfdcan1, FDCAN1_IT0_IRQn);
    MY_fdcan_start_one(&hfdcan2, FDCAN2_IT0_IRQn);
}

void MY_fdcan_send(FDCAN_HandleTypeDef *hfdcan,
                   uint8_t mydata1, uint8_t mydata2, uint8_t mydata3, uint8_t mydata4,
                   uint8_t mydata5, uint8_t mydata6, uint8_t mydata7, uint8_t mydata8,
                   uint32_t myid)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    uint8_t data[8];

    if (hfdcan == NULL)
    {
        return;
    }

    data[0] = mydata1;
    data[1] = mydata2;
    data[2] = mydata3;
    data[3] = mydata4;
    data[4] = mydata5;
    data[5] = mydata6;
    data[6] = mydata7;
    data[7] = mydata8;

    tx.Identifier         = myid;
    tx.IdType             = FDCAN_STANDARD_ID;
    tx.TxFrameType        = FDCAN_DATA_FRAME;
    tx.DataLength         = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch      = FDCAN_BRS_OFF;
    tx.FDFormat           = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker      = 0U;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, data);
}
//最终can接收全处理函数
static void CAN_HandleFrame(FDCAN_HandleTypeDef *hfdcan,
                            const FDCAN_RxHeaderTypeDef *rx,
                            const uint8_t data[8])
{
    uint8_t bus;
    uint8_t index;
    uint32_t id;

    if ((hfdcan != &hfdcan1) && (hfdcan != &hfdcan2))
    {
        return;
    }

    bus = (hfdcan == &hfdcan1) ? CAN_BUS_1 : CAN_BUS_2;

    if (rx->IdType == FDCAN_EXTENDED_ID)
    {
        VESC_Decode(hfdcan, rx, data);
        return;
    }

    if ((rx->IdType != FDCAN_STANDARD_ID) ||
        (rx->DataLength != FDCAN_DLC_BYTES_8))
    {
        return;
    }

    id = rx->Identifier;

    if (id == ((bus == CAN_BUS_1) ? DM_CAN1_FEEDBACK_ID : DM_CAN2_FEEDBACK_ID))
    {
        dm_fb[bus].bus = bus;

        if (DM_DecodeFeedback(rx, data, &dm_fb[bus]) != 0U)
        {
            DM_Feedback_RxCallback(hfdcan, &dm_fb[bus]);
        }
        return;
    }

    if ((id >= 0x201U) && (id <= 0x204U))
    {
        index = (uint8_t)(id - 0x201U);
        DJI_Decode(&dji3508_fb[bus][index], bus, (uint8_t)(index + 1U), data);
        return;
    }

    if ((id >= 0x205U) && (id <= 0x208U))
    {
        index = (uint8_t)(id - 0x205U);
        DJI_Decode(&dji6020_fb[bus][index], bus, (uint8_t)(index + 1U), data);
    }
}
//can回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    if ((hfdcan == NULL) ||
        ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan,
                                   FDCAN_RX_FIFO0,
                                   &rx,
                                   data) != HAL_OK)
        {
            break;
        }

        CAN_HandleFrame(hfdcan, &rx, data);
    }
}

__weak void DM_Feedback_RxCallback(FDCAN_HandleTypeDef *hfdcan,
                                   const DM_Feedback_t *feedback)
{
    (void)hfdcan;
    (void)feedback;
}
