//
// Created by YanYuanbin on 22-10-4.
//
#include "mycan.h"
#include "motor.h"

CAN_RxFrameTypeDef hcanRxFrame;
CAN_TxFrameTypeDef hcanTxFrame = {
							.header.StdId=0x200,
							.header.IDE=CAN_ID_STD,
							.header.RTR=CAN_RTR_DATA,
							.header.DLC=8,
};

void CAN_Init(void)
{
    CAN_FilterTypeDef sFilterConfig={0};

		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterBank = 0;
		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig.SlaveStartFilterBank = 0;

		// 配置CAN标识符滤波器
		if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
		{	
		    Error_Handler();
		}
    // 开启CAN1
    HAL_CAN_Start(&hcan1);
		// 使能接收中断
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void USER_CAN_TxMessage(CAN_HandleTypeDef *hcan,CAN_TxFrameTypeDef *TxHeader)
{
    uint32_t TxMailbox = 0;

//    while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );

    HAL_CAN_AddTxMessage(hcan, &TxHeader->header, TxHeader->data, &TxMailbox);
}

/**
 *	@brief	CAN1接收数据处理
 */
void CAN1_rxDataHandler(uint32_t *canId, uint8_t *rxBuf)
{
	//左轮电机
	get_Motor_Data(canId,rxBuf,&Balance[Left_LSpiral]);
	get_Motor_Data(canId,rxBuf,&Balance[Left_RSpiral]);
	//右轮电机
	get_Motor_Data(canId,rxBuf,&Balance[Right_LSpiral]);
	get_Motor_Data(canId,rxBuf,&Balance[Right_RSpiral]);
}

/**
 *	@brief	重写 CAN RxFifo 中断接收函数
 *	@note	在stm32f4xx_hal_can.c中弱定义
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* CAN1 接收中断 */
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcanRxFrame.header, hcanRxFrame.data);
        CAN1_rxDataHandler(&hcanRxFrame.header.StdId,hcanRxFrame.data);
		}
}
