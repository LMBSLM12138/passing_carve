#ifndef __DM_DRI_4310_H
#define __DM_DRI_4310_H

#include <stdint.h>
#include "fdcan.h"
#include "MY_fdcan.h"
#include <string.h>

//MIT控制模式下的量程
#define P_MAX   12.5f   /* 位置量程 ±P_MAX  rad    */
#define V_MAX   30.0f   /* 速度量程 ±V_MAX  rad/s  */
#define KP_MAX  500.0f  /* Kp 量程 0~KP_MAX        */
#define KD_MAX  5.0f    /* Kd 量程 0~KD_MAX        */
#define T_MAX   10.0f   /* 转矩量程 ±T_MAX  N·m    */


void DM_dri_enable(FDCAN_HandleTypeDef *hfdcan);

void DM_dri_po_and_sp(FDCAN_HandleTypeDef *hfdcan,
                      float pos_rad,
                      float vel_rads,
                      uint32_t myid);

#endif
