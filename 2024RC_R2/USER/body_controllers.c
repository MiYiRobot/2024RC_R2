#include "body_controllers.h"

static int8_t door_satte; //门状态，1为开启，0为关闭
R2_CONTROLLER_STR R2_CONTROLLER={0};


/**
 * @brief 筛球机构控制器，该控制机构采用VESC驱动N5065电机，电调ID从上往下是101，102，103设置（电调ID可以在VESC TOOL中进行设置）
 * @param 球的预期状态
 * @return 机构的当前状态
*/
int16_t Ball_Process_Function(CONTROLLER_STATE TARGET_STATE)
{
	static int stop_ball = 0;
    for(int i=0; i<3; i++)
        VESC_MOTO_INFO[i].MOTOR_MODE = VESC_SPEED;

    switch (TARGET_STATE)
    {
        case BP_SHOOT_BALL: //存球，放入桶中
        {
            if(door_satte == 1)
            baffle_control(CLOSE);
			stop_ball = 1;
            VESC_MOTO_INFO[0].TARGET_RPM = 1800;
            VESC_MOTO_INFO[1].TARGET_RPM = 1800;
            VESC_MOTO_INFO[2].TARGET_RPM = 1800;

            break;
        }

        case BP_ABANDON_BALL: //筛掉不要的球
        {
            VESC_MOTO_INFO[0].TARGET_RPM = 0;
            VESC_MOTO_INFO[1].TARGET_RPM = 0;
            VESC_MOTO_INFO[2].TARGET_RPM = 1071;

            if(door_satte == 0)
                baffle_control(OPEN);

            break;
        }
		
		case BP_INVERTED:  //电机反转，防止卡球，由于有一个VESC坏了，使用好盈电调临时替代，但效果并不理想
		{
            static uint16_t cnt=0;
            if(door_satte == 1)
                baffle_control(CLOSE);
            
			if(stop_ball == 1)
			{
                cnt++;
				VESC_MOTO_INFO[0].TARGET_RPM = 0;
				VESC_MOTO_INFO[1].TARGET_RPM = 0;
				VESC_MOTO_INFO[2].TARGET_RPM = 0;
				
				if(cnt>=1000)    
				stop_ball = 0;
			
				VESC_MOTO_INFO[0].TARGET_RPM = -1430;
				VESC_MOTO_INFO[1].TARGET_RPM = -1430;
                VESC_MOTO_INFO[2].TARGET_RPM = -285;
				
			}

            if(cnt>=1500)
            {
                VESC_MOTO_INFO[0].TARGET_RPM = 1285;
                VESC_MOTO_INFO[1].TARGET_RPM = 1285;
                VESC_MOTO_INFO[2].TARGET_RPM = 1430;

                cnt = 0;
            }

			break;
		}

        case BP_CONTROLLER_OFF:    //关闭机构控制
        {   
            if(door_satte == 1)
                baffle_control(CLOSE);

            VESC_MOTO_INFO[0].TARGET_RPM = 0;
            VESC_MOTO_INFO[1].TARGET_RPM = 0;
            VESC_MOTO_INFO[2].TARGET_RPM = 0;

            break;
        }
        
        default: break;
    }

    return 0;
}

/**
 * @brief 筛球挡板控制器
 * @param 模式 OPEN or CLOSE
 * @author wnagqi
*/
void baffle_control(DOOR door)
{
    static  int8_t homeing_flag=0,cnt=0;
    switch (door)
    {
		case OPEN:                       // the func is used to get the singal 
		{								//when it get the 1, open the door
            door_satte = 1;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);//气缸开启
			if(cnt++ >= 10)
            {
			    // Position_Control(&MOTO_REAL_INFO[4],120);
                homeing_flag = 0;
                cnt = 0;//利用任务的循环做粗略计时，如果使用精准计时则用TIM计时器
            }
			break;
        }
		
		case CLOSE:
        {        
            door_satte = 0;
			// Position_Control(&MOTO_REAL_INFO[4],0);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);   //气缸关闭
            cnt = 0;
			break;
        }
		
		default: break;
    }
}

/**
 * @brief 吸球控制器
*/
void TakeBall_Controller(CONTROLLER_STATE EXPECT_BALL_STA)
{
    for(int i=3; i<5; i++)
        VESC_MOTO_INFO[i].MOTOR_MODE = VESC_DUTY;

    switch (EXPECT_BALL_STA)
    {
        case FW_TAKE_BALL:  //吸球
        {
            VESC_MOTO_INFO[3].TARGET_VESC_DUTY=0.15;
            VESC_MOTO_INFO[4].TARGET_VESC_DUTY=0.15;
            break;
        }

        case FW_CONTROLLER_OFF:     //停止
        {
            VESC_MOTO_INFO[3].TARGET_VESC_DUTY=0.0;
            VESC_MOTO_INFO[4].TARGET_VESC_DUTY=0.0;
            break;
        }
        
        case FW_INVERTED:       //反转吐球
        {
            VESC_MOTO_INFO[3].TARGET_VESC_DUTY=-0.05;
            VESC_MOTO_INFO[4].TARGET_VESC_DUTY=-0.05;
            break;
        }

        default: 
        {
            VESC_MOTO_INFO[3].TARGET_VESC_DUTY=0.0;
            VESC_MOTO_INFO[4].TARGET_VESC_DUTY=0.0;
            break;
        }
    }
}


/**
 * @brief 底盘控制器
*/
void Chassis_Controller(void)
{
    if(ROBOT_CHASSI.Chassis_Controller_Flag == 1)
	{    
        //ros端控制
        ROBOT_CHASSI.SPEED.Robot_VX = R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_X;
        ROBOT_CHASSI.SPEED.Robot_VY = R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_Y;
        ROBOT_CHASSI.WORLD.World_W = R2_CONTROLLER.CHASSIS_CONTROLLER.WORLD.World_W; 
    }
	else
	{
		ROBOT_CHASSI.WORLD.World_X = 0;
		ROBOT_CHASSI.WORLD.World_Y = 0;
		ROBOT_CHASSI.WORLD.World_W = 0;
	}
    Robot_Wheels_RPM_calculate();
}


uint8_t linear_act_enable = 0;
/**
 * @brief 电动推杆初始化
*/
void liner_actuator(CONTROLLER_STATE EXPECT_STATE)
{
    static uint32_t cnt=0;

    switch (EXPECT_STATE)
    {
        case LINEAR_ACTUATOR_GO:
            if(cnt<4500)
            {
                cnt++;
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
                TIM13->CCR1 = 20000;
                TIM14->CCR1 = 20000;
            }

        break;

        case LINEAR_ACTUATOR_BACK:
            if(cnt<4500)
            {
                cnt++;
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
                TIM13->CCR1 = 20000;
                TIM14->CCR1 = 20000;
            }

            break;

        case LINEAR_ACTUATOR_OFF:
            cnt = 0;
            TIM13->CCR1 = 0;
            TIM14->CCR1 = 0;
            break;

        default:
            break;
    }
}




/**
 * @brief 使用上位机控制机构
*/
void ROS_Control(void)
{
    if(R2_CONTROLLER.ros_stm32_noconnected_flag==0)   //防止串口硬件连接断开，超时200ms
	{   //还未断连
        //筛球机构（等坤更新）
		//Ball_Process_Function(R2_CONTROLLER.NEXT_BP_STATE);     
        //吸球机构控制器
        //R2_CONTROLLER.CURRENT_FW_STATE=suction_controller(R2_CONTROLLER.NEXT_FW_STATE);
        TakeBall_Controller(R2_CONTROLLER.NEXT_FW_STATE);
	}
	else
	{          
        //断连则关闭所有控制器
		//Ball_Process_Function(BP_CONTROLLER_OFF);
		//R2_CONTROLLER.CURRENT_FW_STATE=suction_controller(FW_CONTROLLER_OFF);
        TakeBall_Controller(FW_CONTROLLER_OFF);
		ROBOT_CHASSI.Chassis_Controller_Flag = 0;
	}
}


//----------------------------------新的控制器-------------------------------------

/**
 * @brief 吸球机构控制器，该控制机构采用1个滚筒3508电机和2个摩擦轮3508电机
 * @param 球的预期状态
 * @return 机构的当前状态
*/
CONTROLLER_STATE suction_controller(CONTROLLER_STATE NEXT_STATE)
{
        switch (NEXT_STATE)
        {
        case FW_TAKE_BALL:          //吸球状态
                    
                   
//                    if(R2_CONTROLLER.FW_open_flag)
//                    {
                        Speed_Control(&MOTO_REAL_INFO[5],3000);
                        Speed_Control(&MOTO_REAL_INFO[6],3000);
                  //  }
                      Speed_Control(&MOTO_REAL_INFO[7],2000);
                        return FW_TAKE_BALL;
                    break;
        case FW_CONTROLLER_OFF:
                    MOTO_REAL_INFO[5].Motor_Mode = MOTO_OFF;
                    MOTO_REAL_INFO[6].Motor_Mode = MOTO_OFF;
                    MOTO_REAL_INFO[7].Motor_Mode = MOTO_OFF;
                    return FW_CONTROLLER_OFF;
                    break;    
        case FW_INVERTED:           //摩擦轮反转
                    MOTO_REAL_INFO[7].Motor_Mode = MOTO_OFF;
                    Speed_Control(&MOTO_REAL_INFO[5],-2000);
                    Speed_Control(&MOTO_REAL_INFO[6],-2000);
                    return FW_INVERTED;
                    break;                     
        default:
            return CONTROLLER_ERROR;
            
        }
}