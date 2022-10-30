//
// Created by YanYuanbin on 22-10-4.
//

#ifndef GIMBAL_MYCAN_H
#define GIMBAL_MYCAN_H

#include "can.h"

enum{
    _CAN1,
    _CAN2,
    CAN_PORT_NUM,
};

enum{
    _0x1FF,
    _0x200,
    stdID_NUM,
};

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t				data[8];
} CAN_TxFrameTypeDef;

extern CAN_TxFrameTypeDef hcanTxFrame;

/* mycan functions ---------------------------------------------------------*/
extern void CAN_Init(void);
extern void USER_CAN_TxMessage(CAN_HandleTypeDef *hcan,CAN_TxFrameTypeDef *TxHeader);

#endif //GIMBAL_MYCAN_H
