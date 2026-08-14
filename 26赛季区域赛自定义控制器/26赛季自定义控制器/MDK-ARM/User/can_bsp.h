#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__
#include "ALL.h"
#include "can.h"

void can_bsp_init(void);
uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

extern CAN_RxHeaderTypeDef R1Header;
extern CAN_RxHeaderTypeDef R2Header;

#endif

