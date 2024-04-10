#include "main.h"

/**
 * @brief 配置CAN滤波器
*/
void User_Open_Can1(void)
{
	CAN_FilterTypeDef Filter;
	
	Filter.FilterBank = 0;	//过滤器组别0
    Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter.FilterIdHigh         = (((uint32_t)CAN_RxExtId<<3)&0xffff0000)>>16;				/* 要过滤的ID高位 *///0x0000
  	Filter.FilterIdLow          = (((uint32_t)CAN_RxExtId<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff0000; /* 要过滤的ID低位 *///0x0000	
    Filter.FilterMaskIdHigh = 0x0000;	//拓展帧标识符
    Filter.FilterMaskIdLow = 0x0000;
    Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	Filter.FilterActivation = ENABLE;	//使能过滤器
	Filter.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan1, &Filter) != HAL_OK)
	{
			/* Filter configuration Error */
			Error_Handler();
	}
		
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
		
		/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}



/**
 * @brief 配置CAN滤波器
*/
void User_Open_Can2(void)
{
	CAN_FilterTypeDef Filter;
	
	Filter.FilterBank = 15;	//过滤器组别15
    Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter.FilterIdHigh         = (((uint32_t)CAN_RxExtId<<3)&0xffff0000)>>16;				/* 要过滤的ID高位 *///0x0000
  	Filter.FilterIdLow          = (((uint32_t)CAN_RxExtId<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff0000; /* 要过滤的ID低位 *///0x0000	
    Filter.FilterMaskIdHigh = 0x0000;	//拓展帧标识符
    Filter.FilterMaskIdLow = 0x0000;
    Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	Filter.FilterActivation = ENABLE;	//使能过滤器
	Filter.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan2, &Filter) != HAL_OK)
	{
			/* Filter configuration Error */
			Error_Handler();
	}
		
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}



uint8_t RxData[8];
uint8_t RxData2[8];
uint16_t test_id;
/**
 * @brief hal库CAN回调函数,接收电机数据
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	test_id = 2;
	if(hcan == &hcan1)
	{
		CAN_RxHeaderTypeDef CAN1_RX0_message;

		//获取电机数据，&msg中包含接收到的can的ID信息 (RxData 包含电机的状态信息)
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_RX0_message, RxData);
		
		if (CAN1_RX0_message.IDE == CAN_ID_STD)
		{
			//更新电机数据
			get_motor_measure(&CAN1_RX0_message, RxData);
			
			//RM电机进行角度积分
			for(uint16_t m=0;m<8;m++)
				RM_MOTOR_Angle_Integral(&MOTO_REAL_INFO[m]);
		}
	}

	if(hcan == &hcan2)
	{
		CAN_RxHeaderTypeDef CAN2_RX0_message;
		//获取电机数据，&msg中包含接收到的can的ID信息 (RxData 包含电机的状态信息)
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN2_RX0_message, RxData2);
		if(CAN2_RX0_message.IDE == CAN_ID_EXT)
		{
			can_get_data(&CAN2_RX0_message,RxData2);
		}
	}
}


