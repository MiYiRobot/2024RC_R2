#include "main.h"

/*********************************************************************************************************************************************************
2023/10/2 Author：Yang JianYi
STM32和ROS的串口通讯，使用的端口为USART1，该通讯协议编写了float类型，int16类型和char类型的数据，预留一个控制位来进行是否使用ROS自动控制的开关
debug用的串口暂时选定为uart5
**********************************************************************************************************************************************************/

 /*--------------------------------发送协议-----------------------------------
	STM32与ROS的通讯协议
	中断回调函数的启用需要有一次中断服务函数的触发，改触发放在了main.c中
--------------------------------------------------------------------------*/
 /*--------------------------------发送协议-----------------------------------
----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
	数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
	注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
	数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
	注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/
 


//通信协议常量
const unsigned char serial_header[2]  = {0x55, 0xaa};
const unsigned char serial_ender[2]   = {0x0d, 0x0a};

//接收电机转速命令共用体
union Recieve_Data_RPM
{
	short data;
	unsigned char tem_array[2];
}RosToStm32_RPM1,RosToStm32_RPM2;

//接收电机角度命令共用体
union Recieve_Data_CHASSIS
{
	float data;
	unsigned char tem_array[4];
}RosToStm32_CHASSIS_X,RosToStm32_CHASSIS_Y,RosToStm32_CHASSIS_W;

unsigned char testRece4 = 0x00;
uint8_t test = 0;
/**
 * @brief 串口中断回调函数，数据解包以及赋值在里面进行操作
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
		R2_CONTROLLER.ros_stm32_noconnected_cnt = 0;
		R2_CONTROLLER.ros_stm32_noconnected_flag = 0;
		test ++;
		//底盘速度，底盘运动标志位，滚筒下一个状态，吸球下一个状态
		STM32_READ_FROM_ROS_FLOAT(&R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_X,&R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_Y,
								  &R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_W,&ROBOT_CHASSI.Chassis_Controller_Flag,
								  &R2_CONTROLLER.NEXT_BP_STATE,&R2_CONTROLLER.NEXT_FW_STATE,&R2_CONTROLLER.FW_open_flag);
			
    }

	if(huart->Instance == UART5)
	{
		STM32_READ_FROM_ROS_FLOAT(&R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_X,&R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_Y,
									  &R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_W,&ROBOT_CHASSI.Chassis_Controller_Flag,
									  &R2_CONTROLLER.NEXT_BP_STATE,&R2_CONTROLLER.NEXT_FW_STATE,&R2_CONTROLLER.FW_open_flag);
	}

	if(huart->Instance == UART4)
	{
		action_data_analyse();
	}
}



//数据接收暂存区
unsigned char receivesend_buff[22]={0};
unsigned char USART_Receiver1              = 0;          //接收数据
/**
 * @brief 接收ROS发来的数据并解包
 * @param 接收的数据地址
*/
int STM32_READ_FROM_ROS_FLOAT(float *target_chassis_x, float *target_chassis_y, float *target_chassis_w, unsigned char *chassis_flag, 
							  unsigned char *BP_ctrl_state, unsigned char *FW_ctrl_state,unsigned char *FW_open_flag)
{
	test =2;
	static unsigned char checkSum             = 0;			//校验值
	static unsigned char USARTsend_bufferIndex     = 0;		//用于记录解包的操作步骤
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;			
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;			//数据包中数据的总长度

	HAL_UART_Receive_DMA(&huart1,&USART_Receiver1,1);		//中断服务函数
	// HAL_UART_Receive_IT(&huart5,&USART_Receiver1,1);		//调试使用，勿管。开关在freertos.c

	if(Start_Flag == START)
	{
		if(USART_Receiver1 == 0xaa)                         //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         			//数据头两位 //buf[0]
			{
				Start_Flag = !START;             			//收到数据头，开始接收数据
				//printf("header ok\n");
				receivesend_buff[0]=serial_header[0];       //buf[0]
				receivesend_buff[1]=serial_header[1];		//buf[1]
				USARTsend_bufferIndex = 0;             		//缓冲区初始化
				checkSum = 0x00;				  			//校验和初始化
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver1;  
		}
	}
	else
    { 
		test =3;
		switch(USARTsend_bufferIndex)
		{
			case 0://接收数据的长度
			{
				receivesend_buff[2] = USART_Receiver1;
				dataLength     =  receivesend_buff[2];            //buf[2]
				USARTsend_bufferIndex++;
				break;
			}
			case 1://接收所有数据，并赋值处理 
			{
					receivesend_buff[j + 3] = USART_Receiver1;	//buf[3] ~ buf[17]	
					j++;
					if(j >= dataLength)    						//进入条件 j = 15                     
					{
						j = 0;									//置0
						USARTsend_bufferIndex++;
					}
					break;
			}
			case 2://接收校验值信息(设定为0x0b)
			{
				receivesend_buff[3 + dataLength] = USART_Receiver1;					//接收数据包中的校验值
				checkSum = serial_get_crc8_value(receivesend_buff, 3 + dataLength); //buf[19]
				
				// 检查信息校验值
				if (checkSum != receivesend_buff[3 + dataLength]) 					//buf[19]
				{
					debug_safe_printf("Received tem_array check sum error!\r\n");
					return 0;
				}
				USARTsend_bufferIndex++;
				break;
			}
				
			case 3://接收信息尾
			{
				if(k==0)
				{
					//数据0d     buf[17]  无需判断
					k++;
				}
				else if(k==1)
				{
					//数据0a     buf[18] 无需判断
					//进行赋值操作						
					
					for(k = 0; k < 4; k++)//浮点型(float)赋值
					{
						RosToStm32_CHASSIS_X.tem_array[k] = receivesend_buff[k + 3]; //buf[3] ~ buf[6]
						RosToStm32_CHASSIS_Y.tem_array[k] = receivesend_buff[k + 7]; //buf[7] ~ buf[10]
						RosToStm32_CHASSIS_W.tem_array[k] = receivesend_buff[k + 11]; //buf[11] ~ buf[14]
					}
							
					//赋值操作
					*target_chassis_x = RosToStm32_CHASSIS_X.data;	
					*target_chassis_y = RosToStm32_CHASSIS_Y.data;
					*target_chassis_w = RosToStm32_CHASSIS_W.data;
					*chassis_flag = receivesend_buff[15];
					*BP_ctrl_state = receivesend_buff[16];
					*FW_ctrl_state = receivesend_buff[17];
					*FW_open_flag = receivesend_buff[18];
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
					USARTsend_bufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			}
			default:break;
		}
	}
	return 0;
}


/**
 * @brief 串口发送数据函数
 * @param 当前滚筒状态，当前吸球状态，是否有球，颜色标志位1，颜色标志位2，颜色标志位3
 * @return NULL
*/
void Usart_Send_Data(unsigned char current_bp_state,unsigned char current_fw_state,unsigned char is_ball_in_car,
                             unsigned char color_flag1,unsigned char color_flag2,unsigned char color_flag3)
    
{
    // 协议数据缓存数组
	unsigned char send_buf[6+6] = {0};
	int i, Length = 0;

    //设置消息头
    for (i = 0; i < 2; i++)
    {
        send_buf[i] = serial_header[i];
    }

    Length = 6; 
	send_buf[2] = Length;    

	//数据位赋值
	send_buf[3] = current_bp_state;	//buf[3]
	send_buf[4] = current_fw_state;	//buf[4]
	send_buf[5] = is_ball_in_car;	//buf[5]
	send_buf[6] = color_flag1;		//buf[6]
	send_buf[7] = color_flag2;		//buf[7]
	send_buf[8] = color_flag3;		//buf[8]
      
    // 设置校验值、消息尾
    send_buf[3 + Length] = serial_get_crc8_value(send_buf, 3 + Length);	//send_buf[9]
    send_buf[3 + Length + 1] = serial_ender[0];							//send_buf[10]
    send_buf[3 + Length + 2] = serial_ender[1];							//send_buf[11]

    USART_Send_String(send_buf, sizeof(send_buf), USART1);
   
}

/**
 * @brief 发送字符数组（字符串）的函数
 * @param 数组地址(直接传入数组名字即可)
 * @param 数组长度(可以用sizeof)
 * @param 串口端口，例如USART1，USART2。。。
*/
void USART_Send_String(unsigned char *p_array, short sendSize, USART_TypeDef *usart)
{ 
	static int length =0;
	while(length<sendSize)
	{  
		while( !(usart->SR&(0x01<<7)) );//发送缓冲区为空
		usart->DR=*p_array;                   
		p_array++;
		length++;
	}
	length =0;
   //test_send++;
}

/**
 * @brief 校验位函数Crc8
 * @param 传入数组
 * @param 当前数组长度
 * @return 检验值
*/
unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len)
{
	unsigned char crc = 0;
	unsigned char i;
	while(len--)
	{
		crc ^= *tem_array++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
				crc=(crc>>1)^0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}

