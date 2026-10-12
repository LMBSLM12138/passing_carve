/**
 * @file drv_usb.c
 * @brief 仿照SCUT-Robotlab改写的USB通信初始化与配置流程 (C语言版)
 * @note  static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
 *        为实际接收回调函数, 在该文件中
 */

/* Includes ------------------------------------------------------------------*/

#include "USB_Transmat.h"

/* Private variables ---------------------------------------------------------*/

Struct_USB_Manage_Object USB0_Manage_Object = {0};

// USB设备句柄, 由 usbd_cdc_if.c 中定义
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];
bool init_finished = false;

// 全局临时帧
uint8_t g_usb_rx_tmp[512];
uint16_t g_usb_rx_len = 0;
volatile uint8_t g_usb_rx_flag = 0;

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param Callback_Function 处理回调函数
 */
void USB_Init(USB_Callback Callback_Function)
{
    USB0_Manage_Object.Callback_Function = Callback_Function;

    USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
}

/**
 * @brief 发送数据帧
 *
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length)
{
    return (CDC_Transmit_HS(Data, Length));
}

/**
 * @brief USB通信下一轮接收开启前回调函数, 非HAL库回调函数
 *
 * @param Size 接收数据长度
 */
void USB_ReceiveCallback(uint16_t Size)
{
    if (!init_finished)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return;
    }

    // 自设双缓冲USB
    USB0_Manage_Object.Rx_Buffer_Ready = USB0_Manage_Object.Rx_Buffer_Active;
    if (USB0_Manage_Object.Rx_Buffer_Active == USB0_Manage_Object.Rx_Buffer_0)
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_1;
    }
    else
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_0;
    }

    USB0_Manage_Object.Rx_Time_Stamp = SYS_Timestamp_Get_Current_Timestamp();

    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);

    if (USB0_Manage_Object.Callback_Function != NULL)
    {
        USB0_Manage_Object.Callback_Function(USB0_Manage_Object.Rx_Buffer_Ready, Size);
    }
}
//回调函数，将数据转移
void My_USB_Rx_Handler(uint8_t *Buffer, uint16_t Length)
{
    if (Length > sizeof(g_usb_rx_tmp)) Length = sizeof(g_usb_rx_tmp);
    memcpy(g_usb_rx_tmp, Buffer, Length);  // 快速拷贝（中断里，尽量快）
    g_usb_rx_len  = Length;
    g_usb_rx_flag = 1;                     // 置标志，提醒主循环
}

// if (g_usb_rx_flag) {
//     g_usb_rx_flag = 0;
//     Process_My_Frame(g_usb_rx_tmp, g_usb_rx_len); // 耗时解析放到这里
// }

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/