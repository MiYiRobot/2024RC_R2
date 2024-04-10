#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#define START 0x11

enum FLAG{
	CHECK_HEADER_FLAG, 
	RECEIVE_DATA_FLAG, 
	CHECK_VALUE_FLAG, 
	FINAL_RECEIVE_FLAG
};


unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len);
void USART_Send_String(unsigned char *p, short sendSize, USART_TypeDef *usart);
int STM32_READ_FROM_ROS_FLOAT(float *target_chassis_x, float *target_chassis_y, float *target_chassis_w, unsigned char *chassis_flag, 
							  unsigned char *BP_ctrl_state, unsigned char *FW_ctrl_state,unsigned char *FW_open_flag);
void Usart_Send_Data(unsigned char current_bp_state,unsigned char current_fw_state,unsigned char is_ball_in_car,
                             unsigned char color_flag1,unsigned char color_flag2,unsigned char color_flag3);

#endif
