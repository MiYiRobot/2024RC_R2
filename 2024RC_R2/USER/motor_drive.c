#include "main.h"


/*******************************************************************************************************************
2023/10/12 Author：Yang JianYi

VESC电调为八期新开
电机驱动文件：大疆电机封装在七期的封装上进行一些小改
********************************************************************************************************************/


MOTOR_REAL_INFO     MOTO_REAL_INFO[8];
PID_Data MOTOR_PID_RPM[8];
PID_Data MOTOR_PID_POS[8];	//位置pid信息
VESC_MOTOR_INFO VESC_MOTO_INFO[6];

#define USE_ALL_MOTOR_PACKED 0


//vesc电调控制
void VESC_Control(VESC_MOTOR_INFO *vesc)
{
//	int N5065_poles = 7, U8_poles=21;
//	switch (vesc->MotorType)
//	{
//		case VESC_5065:
//		{
//			if(vesc->TARGET_RPM*N5065_poles > 30000)
//				vesc->TARGET_RPM = 30000/N5065_poles;
//			if(vesc->TARGET_RPM*N5065_poles < -30000)
//				vesc->TARGET_RPM = -30000/N5065_poles;

//			vesc->TARGET_RPM = vesc->TARGET_RPM*N5065_poles;
//			break;
//		}

//		case VESC_U8:
//		{
//			if(vesc->TARGET_RPM*U8_poles > 60000)
//				vesc->TARGET_RPM = 60000/U8_poles;
//			if(vesc->TARGET_RPM*U8_poles < -60000)
//				vesc->TARGET_RPM = -60000/U8_poles;

//			vesc->TARGET_RPM = vesc->TARGET_RPM*U8_poles;
//			break;
//		}
//			
//		default:
//			break;
//	}

	if(vesc->MOTOR_MODE == VESC_SPEED)
	{
		comm_can_set_rpm(vesc->VESC_CAN_ID,vesc->TARGET_RPM);
	}

	if (vesc->MOTOR_MODE == VESC_CURRENT)
	{
		comm_can_set_current(vesc->VESC_CAN_ID,vesc->TARGET_CURRENT);
	}

	if (vesc->MOTOR_MODE == VESC_DUTY)
	{
		comm_can_set_duty(vesc->VESC_CAN_ID,vesc->TARGET_VESC_DUTY);
	}
	
	if (vesc->MOTOR_MODE == VESC_POS)
	{
		comm_can_set_pos(vesc->VESC_CAN_ID,vesc->TARGET_ANGLE);
	}
}


/**
 * @brief 电机控制模式（GM6020待测试）
 * @param NULL
 * @return NULL
*/
void Motor_Control(void)
{
	for(int i=0; i < 8; i++)
	{
		//判断是否有选择电机类型，若无直接退出
		if (MOTO_REAL_INFO[i].Motor_Type == NONE)	
			break;
		
		//电机模式选择
		switch (MOTO_REAL_INFO[i].Motor_Mode)
		{
			case SPEED_CONTROL_MODE: //速度模式
			{
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i],MOTO_REAL_INFO[i].TARGET_RPM,MOTO_REAL_INFO[i].RPM);//速度环
				break;
			}

			case VELOCITY_PLANNING_MODE: //梯形模式
			{
				Velocity_Planning_MODE(&MOTO_REAL_INFO[i]);					
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i],MOTO_REAL_INFO[i].TARGET_RPM,MOTO_REAL_INFO[i].RPM);
				break;
			}

			case CURRENT_MODE: //电流模式(直接赋电流值)
			{
				break;
			}

			case POSITION_CONTROL_MODE://位置模式
			{
				PID_Position_Calculate(&MOTOR_PID_POS[i],MOTO_REAL_INFO[i].TARGET_POS, MOTO_REAL_INFO[i].REAL_ANGLE);//位置环
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i], MOTOR_PID_POS[i].Output, MOTO_REAL_INFO[i].RPM);//速度环
				break;
			}

			case SPEED_TARQUE_CONTROL_MODE://速度转矩模式
			{
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i],MOTO_REAL_INFO[i].Velocity_Tarque.Target_Vel,MOTO_REAL_INFO[i].RPM);	//速度环
				MOTOR_PID_RPM[i].Output = Max_Value_Limit(MOTOR_PID_RPM[i].Output,MOTO_REAL_INFO[i].Velocity_Tarque.TARGET_TORQUE);	//限制转矩模式时电流值
				
				//判断是否夹取
				//flag = 1夹取成功
				if(fabsf(MOTO_REAL_INFO[i].RPM) <=10)
				{
					MOTO_REAL_INFO[i].Velocity_Tarque.Cnt++;
				}
				else
				{
					MOTO_REAL_INFO[i].Velocity_Tarque.Cnt = 0;
				}

				if(MOTO_REAL_INFO[i].Velocity_Tarque.Cnt >= 10)
				{
					MOTO_REAL_INFO[i].Velocity_Tarque.Cnt=0;
					MOTO_REAL_INFO[i].Velocity_Tarque.Flag = 1;
					MOTO_REAL_INFO[i].Motor_Mode= SPEED_CONTROL_MODE;
					MOTO_REAL_INFO[i].TARGET_RPM = 0;
				}
				break;
			}

			case MOTO_OFF://电机关闭
			{
				MOTO_REAL_INFO[i].TARGET_CURRENT = 0.0f;//电流赋值
                MOTOR_PID_RPM[i].Output = 0;
				break;
			}

			case HOMEING_MODE://说实在，和速度转矩没什么差别。直接用速度转矩即可
			{
				Homeing_Mode(&MOTO_REAL_INFO[i]);	//调用校准模式
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i], MOTO_REAL_INFO[i].TARGET_RPM, MOTO_REAL_INFO[i].RPM); 	//速度环
				MOTOR_PID_RPM[i].Output = Max_Value_Limit(MOTOR_PID_RPM[i].Output,MOTO_REAL_INFO[i].HomingMode.TARGET_TORQUE);	//限制校准模式电流
			}

			case POSITION_TORQUE_MODE://位置转矩模式
			{
				PID_Position_Calculate(&MOTOR_PID_POS[i],MOTO_REAL_INFO[i].Position_Tarque.Pos, MOTO_REAL_INFO[i].REAL_ANGLE);//位置环
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i], MOTOR_PID_POS[i].Output, MOTO_REAL_INFO[i].RPM);//速度环
				MOTOR_PID_RPM[i].Output = Max_Value_Limit(MOTOR_PID_RPM[i].Output,MOTO_REAL_INFO[i].Position_Tarque.TARGET_TORQUE);//限制转矩模式时电流值

				//判断是否到达目标位置
				if(fabsf(MOTO_REAL_INFO[i].RPM) <=10)
				{		
					MOTO_REAL_INFO[i].Position_Tarque.Cnt++;
				}
				else
				{
					MOTO_REAL_INFO[i].Position_Tarque.Cnt = 0;
				}
		
				if(MOTO_REAL_INFO[i].Position_Tarque.Cnt>=50)//50ms
				{
					MOTO_REAL_INFO[i].Position_Tarque.Cnt = 0;
					MOTO_REAL_INFO[i].Position_Tarque.Flag = 1;
				}
				break;
			}

			default: break;
		}
	}

	//电机转动参数
	for(int i = 0; i < 8; i++)
	{

		if(MOTO_REAL_INFO[i].Motor_Mode == CURRENT_MODE)//防止选择该模式却无法判断
		{
			
		}//电流模式下的特殊情况
		else
		{
			if(MOTO_REAL_INFO[i].Motor_Type == M_3508)
			{
				if(MOTO_REAL_INFO[i].CURRENT > 13000)	//额定电流 10000 * 1.3 = 13000
					MOTO_REAL_INFO[i].Motor_Mode = MOTO_OFF;
				else
					MOTO_REAL_INFO[i].TARGET_CURRENT = MOTOR_PID_RPM[i].Output*16384.0f/20000.0f;	//M3508单位毫安
			}	
			else if(MOTO_REAL_INFO[i].Motor_Type == M_2006)
			{
				if(MOTO_REAL_INFO[i].CURRENT > 3900)	//额定电流 3000 * 1.3 = 3900
					MOTO_REAL_INFO[i].Motor_Mode = MOTO_OFF;
				else
					MOTO_REAL_INFO[i].TARGET_CURRENT = MOTOR_PID_RPM[i].Output*10000.0f/10000.0f;	//M2006单位毫安
			}
			else if(MOTO_REAL_INFO[i].Motor_Type == M_6020)
			{
				if(MOTO_REAL_INFO[i].CURRENT > 2106)	//额定电流 3000 * 1.3 = 2106
					MOTO_REAL_INFO[i].Motor_Mode = MOTO_OFF;
				else
					MOTO_REAL_INFO[i].GM6020.set_voltage = 0;//暂时没玩过，先留着	(电压的给定范围是-30000 ~ 30000)
			}
			else
			{
				MOTO_REAL_INFO[i].CURRENT = 0;
			}
		}
	}

	//使能电机，发送电流数据
	M3508_Send_Currents();
}



/**
 * @brief 最值限制函数
 * @param 传入值
 * @param 限制值(最值) 
 * @return 输出值
*/
float Max_Value_Limit(float Value, float Limit)
{
	if(Value > Limit) Value = Limit;
	if(Value < -Limit) Value = -Limit;

	return Value;
}



/**
  * @brief  Homing mode 回零模式
  * @param  电机结构体
  * @return NULL
 */
void Homeing_Mode(MOTOR_REAL_INFO* RM_MOTOR)
{
	float Sign_Vel = 1.0f;
	RM_MOTOR->HomingMode.flag = 0;

	if (RM_MOTOR->HomingMode.Vel >= 0)
	{
		Sign_Vel = -1.0f;
	}

	//转速赋值
	RM_MOTOR->TARGET_RPM = RM_MOTOR->HomingMode.Vel;
	
	if(fabsf(RM_MOTOR->RPM) <= 30)
	{
		RM_MOTOR->HomingMode.cnt++;
	}
	else
	{
		RM_MOTOR->HomingMode.cnt = 0;
	}
	
	if(RM_MOTOR->HomingMode.cnt >= 50) //500ms
	{
		//清除输出
		RM_MOTOR->HomingMode.cnt = 0;
		RM_MOTOR->REAL_ANGLE=0.0f;	
		RM_MOTOR->HomingMode.flag=1;
		RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
		RM_MOTOR->TARGET_RPM = 0;
	}
}



/**
 * @brief 梯度速度规划
 * @param M电机结构体
 * @return NULL
*/
void Velocity_Planning_MODE(MOTOR_REAL_INFO *M3508_MOTOR)	
{
	//static int cnt;//记时用
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	
	// 如果所配数据有误，则不执行速度规划		
	if((M3508_MOTOR->Velocity_Planning.Rac > 1) || (M3508_MOTOR->Velocity_Planning.Rac < 0) ||		//加速路程的比例
		 (M3508_MOTOR->Velocity_Planning.Rde > 1) || (M3508_MOTOR->Velocity_Planning.Rde < 0) ||	//减速路程的比例
		 (M3508_MOTOR->Velocity_Planning.Vmax < M3508_MOTOR->Velocity_Planning.Vstart) )			//最大的速度<开始的速度 
	{
		M3508_MOTOR->TARGET_RPM = 0;  // 令夹爪不运动
		return;
	}
	// 匀速模式
	if(M3508_MOTOR->Velocity_Planning.Pstart == M3508_MOTOR->Velocity_Planning.Pend)	//开始位置=结束位置
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vmax;	//开始的速度*最大的速度
		return;
	}
	
	// 计算一些变量
	Ssu = ABS(M3508_MOTOR->Velocity_Planning.Pend - M3508_MOTOR->Velocity_Planning.Pstart); 	//总路程   
	Sac = Ssu * M3508_MOTOR->Velocity_Planning.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * M3508_MOTOR->Velocity_Planning.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;								//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (M3508_MOTOR->Velocity_Planning.Vmax * M3508_MOTOR->Velocity_Planning.Vmax - M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend -   M3508_MOTOR->Velocity_Planning.Vmax *   M3508_MOTOR->Velocity_Planning.Vmax) / (2.0f * Sde);	  
	
	// 过滤异常情况
	if(((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pend)) ||
		      ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pend)))
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pstart);      //开始位置
		
		// 规划RPM
		if     (S < Sac)       M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart);               // 加速阶段
		else if(S < (Sac+Sco)) M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vmax;                                                        // 匀速阶段
		else                   M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
	}
	 
	// 分配合适的正负号
	if(M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
	//判断是否完成
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) < 3)
	{
		M3508_MOTOR->Velocity_Planning.flag = 1;//设置标志位		
		M3508_MOTOR->TARGET_RPM=0;
	}
		
						
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) > 3)
	{
		M3508_MOTOR->Velocity_Planning.flag = 0;
	}
}



/**
  * @brief  位置控制(新位置环程序)
  * @param  target_pos目标位置
  * @return 
*/
float Position_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO,float target_pos)
{
	MOTO_REAL_INFO->Motor_Mode = POSITION_CONTROL_MODE;
	MOTO_REAL_INFO->TARGET_POS = target_pos;
	if(ABS(MOTO_REAL_INFO->TARGET_POS-target_pos)<1)
		return 1;	
	else
		return 0;
}



/**
  * @brief  
	* @param 电机结构体
	* @param  target_torque目标转矩，用电流表示
	* @param target_pos目标位置
	* @retval none
  */
void Pos_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Pos)
{
 	MOTO_REAL_INFO->Motor_Mode = POSITION_TORQUE_MODE;
	MOTO_REAL_INFO->Position_Tarque.Pos = Target_Pos;
	MOTO_REAL_INFO->Position_Tarque.TARGET_TORQUE = Target_Torque;
}



/**
 * @brief 速度模式
 * @param 目标转速
 * @return NULL
*/
void Speed_Control(MOTOR_REAL_INFO *RM_MOTOR, int16_t Target_RPM)
{
	RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
	RM_MOTOR->TARGET_RPM = Target_RPM;
}



/**
  * @brief  速度转矩控制函数,假如你要改变电机的转向，那么直接改变Target_Vel的值即可
  * @param  target_torque目标转矩,用电流表示（正数类型）
  * @param target_vel目标位置（有正负，代表转向）
  * @retval none
*/
void Vel_Torque_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO, uint16_t Target_Torque, float Target_Vel)
{	
	MOTO_REAL_INFO->Motor_Mode = SPEED_TARQUE_CONTROL_MODE;
	MOTO_REAL_INFO->Velocity_Tarque.Target_Vel = Target_Vel;
	MOTO_REAL_INFO->Velocity_Tarque.TARGET_TORQUE = Target_Torque;
}



/**
  * @brief  设置速度规划的参数，开启速度规划控制
  * @param  
  * @param float Pstart;        //开始位置
  * @param float Pend;          //结束位置
  * @param float Vstart;        //开始的速度  单位：RPM 绝对值
  * @param float Vmax;          //最大的速度
  * @param float Vend;          //末尾的速度
  * @param float Rac;           //加速路程的比例
  * @param float Rde;           //减速路程的比例
  * @retval NULL
  */
void Velocity_Planning_setpos(MOTOR_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde)
{
	M3508_MOTOR->Motor_Mode = VELOCITY_PLANNING_MODE;//配置模式
	M3508_MOTOR->Velocity_Planning.Pstart = Pstart;
	M3508_MOTOR->Velocity_Planning.Pend = Pend;
	M3508_MOTOR->Velocity_Planning.Vstart = Vstart;
	M3508_MOTOR->Velocity_Planning.Vmax = Vmax;
	M3508_MOTOR->Velocity_Planning.Vend = Vend;
	M3508_MOTOR->Velocity_Planning.Rac = Rac;
	M3508_MOTOR->Velocity_Planning.Rde = Rde;
	M3508_MOTOR->Velocity_Planning.flag = 0;
}



//突然感觉没啥必要,没那么客观清晰，而且封装电机板一般都是同种类型的电机封装
//大疆电机一个板，VESC电机一个板
//浪费我一个多小时的时间，sad！！！
//2023/10/12 yang jianyi
#if USE_ALL_MOTOR_PACKED
//该电机封装函数使用待改进，可用可不用
/**
 * @brief 总电机调用封装,注意：MOTOR_REAL_INFO 和 VESC_MOTOR_INFO只能传入一个，否则无效
 * @param MOTOR_REAL_INFO 假如使用大疆电机，那么请传入对应的大疆电机结构体，否则输入NULL
 * @param VESC_MOTOR_INFO 假如使用VESC电调，那么请传入VESC电调的结构体，否则输入NULL
 * @param MotorType_TypeDef 输入电机类型
 * @param MOTOR_MODE_E 输入对应的电机模式：
 * 			大疆电机：速度环模式：SPEED_CONTROL_MODE 速度转矩模式：Vel_Torque_Control 位置环模式：POSITION_CONTROL_MODE 位置转矩模式：POSITION_TORQUE_MODE
 * 					 回零模式：HOMEING_MODE	电流环模式：CURRENT_MODE 梯形速度规划模式：VELOCITY_PLANNING_MODE(速度规划需要提前调用函数Velocity_Planning_setpos) 电机关闭：MOTO_OFF
 * @param value 对应模式的目标值。注意！！！大疆电机：对应的电流单位为mA，位置模式的单位为度(°)，速度模式单位为转速RPM
 * 										 VESC：对应的电流单位为A，位置模式的单位为度(°)，速度模式单位为电转速eRPM，占空比为浮点数(0.25 = 25%)
 * @param Current_Limit 该参数是大疆电机速度转矩和位置转矩的电流限幅参数，该参数影响转矩的力的大小
*/
int Motor_Control_All(MOTOR_REAL_INFO *MOTOR, VESC_MOTOR_INFO *vesc, MotorType_TypeDef MotorType, MOTOR_MODE_E Mode, float value, uint16_t Current_Limit)
{
	if((MOTOR && vesc) || (MOTOR == NULL && vesc == NULL))
	{
		return 1;
	}
	else if(MotorType == M_2006 || M_3508 || M_6020)
	{
		switch (Mode)
		{
			case SPEED_CONTROL_MODE:
			{
				Speed_Control(MOTOR, (int16_t)value);
				break;
			}

			case SPEED_TARQUE_CONTROL_MODE:
			{
				Vel_Torque_Control(MOTOR, Current_Limit, value);
				break;
			}
			
			case POSITION_CONTROL_MODE:
			{
				Position_Control(MOTOR, value);
				break;
			}

			case POSITION_TORQUE_MODE:
			{
				Pos_Torque_Control(MOTOR, Current_Limit, value);
				break;
			}

			case HOMEING_MODE:
			{
				Homeing_Mode(MOTOR);
				break;
			}

			case VELOCITY_PLANNING_MODE:
			{
				Velocity_Planning_MODE(MOTOR);
				break;
			}

			case CURRENT_MODE:
			{
				MOTOR->TARGET_CURRENT = (int16_t)value;
				break;
			}

			case MOTO_OFF:
			{
				MOTOR->Motor_Mode = MOTO_OFF;
				break;
			}
			
			default:
				break;
		}

		Motor_Control();
	}
	else if(MotorType == VESC_5065)
	{
		switch (Mode)
		{
			case VESC_SPEED:
			{
				comm_can_set_rpm(vesc->VESC_CAN_ID,value);
				break;
			}

			case VESC_CURRENT:
			{
				comm_can_set_current(vesc->VESC_CAN_ID, value);
				break;
			}

			case VESC_DUTY:
			{
				comm_can_set_duty(vesc->VESC_CAN_ID,value);
			}

			case VESC_POS:
			{
				comm_can_set_pos(vesc->VESC_CAN_ID,value);
				break;
			}

			default:
				break;
		}
	}
	else
	{
		return 1;
	}

	return 0;
}
#endif
