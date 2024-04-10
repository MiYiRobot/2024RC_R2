#include "main.h"
#include "can.h"

/************************************************************************************************************************************************
2023/10/9 Author: Yang JianYi

VESC 使用
发送端，主要用到4个数据发送函数：
void comm_can_set_duty(uint8_t controller_id, float duty)			//设置驱动器的占空比
void comm_can_set_current(uint8_t controller_id, float current)		//设置输出电流，用于控制力矩
void comm_can_set_rpm(uint8_t controller_id, float rpm)				//设置输出转速
void comm_can_set_pos(uint8_t controller_id, float pos)				//设置输出位置，驱动器的内部是自带有位置环，速度环以及电流环的。但是要使
																	  驱动器输出位置闭环，那么是需要给电机加上编码器
																	  
另外，关于占空比以及电流大小的设置，只需要执行一次即可，否则容易造成指令冲突发，导致电机运行不稳定																	  


关于can命令的发送函数，根据自己需求写一个就行
利用好VESC TOOL这个工具

注意：由于VESC和大疆电调所用的can信号帧类型不同，所以VESC不和大疆电机使用同一条can总线，以免造成can通道的阻塞，该电调使用了CAN2总线

另外关于刹车函数的使用，设置刹车电流，实际上是can信号停止后电机停下的刹车电流
***********************************************************************************************************************************************/


/**
 * @brief can命令发送函数
 * @param VESC电调的ID
 * @param 发送的数据结构体
 * @param 数据长度
 * @return NULL
*/
void comm_can_transmit_eid(uint32_t controller_id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef			TxHeader;   //can数据发送句柄
	uint32_t txmailbox;
	
    if (len > 8)
    {
        len = 8;
    }
	TxHeader.DLC = (uint8_t)8 ;     //数据长度
	TxHeader.RTR = CAN_RTR_DATA ;   
	TxHeader.IDE = CAN_ID_EXT ;     //使用拓展帧
	TxHeader.ExtId = controller_id; //VESC电调id
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0)    //等待发送邮箱空
        HAL_Delay(1);                                       //防止炸邮箱
    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &txmailbox)!= HAL_OK)
    {
        Error_Handler();
    }
	
}


/**
 * @brief 设置驱动器目标占空比 
 * 
 * @param controller_id 
 * @param duty 0.24 = 24%
 */
void comm_can_set_duty(uint8_t controller_id, float duty)
{
    int32_t send_index = 0;     //用于记录数据长度
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000), &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_DUTY << 8),
                            buffer, send_index);
}

/**
 * @brief 设置本杰明电调输出电流
 * @param id 电调接收can id
 * @param current 电流值/A
 **/
void comm_can_set_current(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000), &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)CAN_PACKET_SET_CURRENT << 8),
                          buffer, send_index);
}


/**
 * @brief 设置本杰明电调输出制动电流
 * @param id 电调接收can id
 * @param current 电流值/A
 **/
void comm_can_set_current_brake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;     
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000), &send_index);//接收端的电流单位是mA
    comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8),
                            buffer, send_index);
}


/**
 * @brief 设置本杰明电调输出转速
 * @param id 电调接收can id
 * @param rpm 单位是电转速/eRPM, eRPM = 电机的极对数*机械转速
 **/
void comm_can_set_rpm(uint8_t controller_id, float rpm)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)rpm, &send_index);
    comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_RPM << 8),
                            buffer, send_index);
}


/**
 * @brief 设置本杰明电调输出位置
 * @param id 电调接收can id
 * @param pos 目标位置
 **/
void comm_can_set_pos(uint8_t controller_id, float pos)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 1000000), &send_index);
    comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_POS << 8),
                            buffer, send_index);
}


/**
 * Set current relative to the minimum and maximum current limits.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [-1.0 1.0]
 */
void comm_can_set_current_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8),
                            buffer, send_index);
}


/**
 * Set brake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the current on.
 *
 * @param current_rel
 * The relative current value, range [0.0 1.0]
 */
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8),
                            buffer, send_index);
}

/**
 * Set handbrake current.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The handbrake current value
 */
void comm_can_set_handbrake(uint8_t controller_id, float current)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current, 1e3, &send_index);
    comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8),
                            buffer, send_index);
}

/**
 * Set handbrake current relative to the minimum current limit.
 *
 * @param controller_id
 * The ID of the VESC to set the handbrake current on.
 *
 * @param current_rel
 * The relative handbrake current value, range [0.0 1.0]
 */
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_float32(buffer, current_rel, 1e5, &send_index);
    comm_can_transmit_eid(controller_id |
                            ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8),
                            buffer, send_index);
}


/**
 * Update current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum current (negative value).
 *
 * @param max
 * Maximum current.
 */
void comm_can_conf_current_limits(uint8_t controller_id, bool store, float min, float max)
{
    int32_t send_index = 0;
    uint8_t buffer[8];
    buffer_append_float32(buffer, min, 1e3, &send_index);
    buffer_append_float32(buffer, max, 1e3, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)(store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS : CAN_PACKET_CONF_CURRENT_LIMITS) << 8),
                          buffer, send_index);
}

/**
 * Update input current limits on VESC on CAN-bus.
 *
 * @param controller_id
 * ID of the VESC.
 *
 * @param store
 * Store parameters in emulated EEPROM (FLASH).
 *
 * @param min
 * Minimum current (negative value).
 *
 * @param max
 * Maximum current.
 */
void comm_can_conf_current_limits_in(uint8_t controller_id,
                                     bool store, float min, float max)
{
    int32_t send_index = 0;
    uint8_t buffer[8];
    buffer_append_float32(buffer, min, 1e3, &send_index);
    buffer_append_float32(buffer, max, 1e3, &send_index);
    comm_can_transmit_eid(controller_id |
                              ((uint32_t)(store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN : CAN_PACKET_CONF_CURRENT_LIMITS_IN) << 8),
                          buffer, send_index);
}

/**
 * @brief VESC获取数据,并且赋值到VESC的电机结构体当中,该函数中保留了VESC传回的所有数据。所需数据均在函数末尾进行赋值
 * @param CAN接收的句柄
 * @param CAN接收数据所用的数组
*/
void can_get_data(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
    can_status_msg 	 status_msg;    //储存VESC发来的can数据
    can_status_msg_2 status_msg_2;
    can_status_msg_3 status_msg_3;
    can_status_msg_4 status_msg_4;
    can_status_msg_5 status_msg_5;
    
	uint16_t cmd;
	uint8_t  id = (RxHeader->ExtId & 0xFF); //截取ID

	cmd = (RxHeader->ExtId >> 8);   //获取对应的帧头

    //对can数据包进行解码
	switch (cmd)
	{
		case CAN_PACKET_STATUS:
		{
			int index = 0;  //记录数组操作到哪个位置
			status_msg.id = id;
            status_msg.rx_time = 0; //没啥用
			status_msg.rpm = buffer_get_uint32(RxData,&index);
			status_msg.current = (float)(buffer_get_int16(RxData,&index)/10.0f);
			status_msg.duty = (float)(buffer_get_int16(RxData,&index)/1000.0f);
			// test_id = value->controller_id;
			break;
		}

		case CAN_PACKET_STATUS_2:
		{
			int index = 0;
			status_msg_2.id = id;
			status_msg_2.rx_time = 0;
			status_msg_2.amp_hours = (float)buffer_get_int32(RxData, &index) / 1e4;
			status_msg_2.amp_hours_charged =  (float)buffer_get_int32(RxData, &index) / 1e4;
			break;
		}

		case CAN_PACKET_STATUS_3:
		{
			int index = 0;
			status_msg_3.id = id;
			status_msg_3.rx_time = 0;
			status_msg_3.watt_hours = (float)buffer_get_int32(RxData, &index) / 1e4;
			status_msg_3.watt_hours_charged = (float)buffer_get_int32(RxData, &index) / 1e4;
			break;
		}

		case CAN_PACKET_STATUS_4:
		{
			int index = 0;
			status_msg_4.id = id;
			status_msg_4.rx_time = 0;
			status_msg_4.temp_fet = (float)buffer_get_int16(RxData,&index) / 10.0f;
			status_msg_4.temp_motor = (float)buffer_get_int16(RxData,&index) / 10.0f;
			status_msg_4.current_in = (float)buffer_get_int16(RxData,&index) / 10.0f;
			status_msg_4.pid_pos_now = (float)buffer_get_int16(RxData,&index) / 50.0f;
			break;
		}

		case CAN_PACKET_STATUS_5:
		{
			int index = 0;
			status_msg_5.id = id;
			status_msg_5.rx_time = 0;
			status_msg_5.tacho_value = buffer_get_int32(RxData, &index);
			status_msg_5.v_in =  (float)buffer_get_int16(RxData, &index) / 1e1;
			break;
		}
		
		default: break;
	}

    //给结构体赋值
    VESC_MOTO_INFO[id - 101].NOW_eRPM = status_msg.rpm;
    VESC_MOTO_INFO[id - 101].NOW_ANGLE = status_msg_4.pid_pos_now;
    VESC_MOTO_INFO[id - 101].NOW_VESC_DUTY = status_msg.duty;
    VESC_MOTO_INFO[id - 101].NOW_CURRENT = status_msg.current;
}




/**************************************
 * 限幅函数
 **************************************/
void assert_param_duty(float *duty)
{
	if( fabsf(*duty) > wtrcfg_VESC_COMMAND_DUTY_MAX )
	{
		if(*duty > 0)
		{
			*duty = wtrcfg_VESC_COMMAND_DUTY_MAX;
		}
		else
		{
			*duty = -wtrcfg_VESC_COMMAND_DUTY_MAX;
		}
	}
}

void assert_param_current(float *current)
{
	if( fabsf(*current) > wtrcfg_VESC_COMMAND_CURRENT_MAX )
	{
		if(*current > 0)
		{
			*current = wtrcfg_VESC_COMMAND_CURRENT_MAX;
		}
		else
		{
			*current = -wtrcfg_VESC_COMMAND_CURRENT_MAX;
		}
	}
}

void assert_param_rpm(float *rpm)
{
	if( fabsf(*rpm) > wtrcfg_VESC_COMMAND_ERPM_MAX )
	{
		if(*rpm > 0)
		{
			*rpm = wtrcfg_VESC_COMMAND_ERPM_MAX;
		}
		else
		{
			*rpm = -wtrcfg_VESC_COMMAND_ERPM_MAX;
		}
	}
}

void assert_param_pos(float *pos)
{
	if( fabsf(*pos) > wtrcfg_VESC_COMMAND_POS_MAX )
	{
		if(*pos > 0)
		{
			*pos = wtrcfg_VESC_COMMAND_POS_MAX;
		}
		else
		{
			*pos = -wtrcfg_VESC_COMMAND_POS_MAX;
		}
	}
}


/***********************
 * 类型转换辅助函数
***********************/
//16位int数据填充
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

//16位unsigned int数据填充
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

//32位int数据填充
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

//32位unsigned int数据填充
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

//16位float数据填充
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) 
{
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}


/**
 * @brief 浮点转换函数
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
*/
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index)
{
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}


int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                  ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index)
{
    uint16_t res = ((uint16_t)buffer[*index]) << 8 |
                   ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index)
{
    int32_t res = ((uint32_t)buffer[*index]) << 24      |
                  ((uint32_t)buffer[*index + 1]) << 16  |
                  ((uint32_t)buffer[*index + 2]) << 8   |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index)
{
    uint32_t res = ((uint32_t)buffer[*index]) << 24     |
                   ((uint32_t)buffer[*index + 1]) << 16 |
                   ((uint32_t)buffer[*index + 2]) << 8  |
                   ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)buffer_get_int16(buffer, index) / scale;
}


float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index)
{
    return (float)buffer_get_int32(buffer, index) / scale;
}
