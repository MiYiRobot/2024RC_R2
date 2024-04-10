#include "FSM.h"
//此代码用来实现stm32端的测试工作

/****************************************************************************************************************************
2023/10/11 Author：Yang JianYi
该文件用于编写stm32端的状态机，Robot_Control_Mode函数为切换ros和32手柄控制的开关，开关为航模遥控的SWA按键(上拨为32控制，下拨为ros控制)
*****************************************************************************************************************************/

//状态机编写
void Fsm32()
{
    if(SWA!=0&SWB!=0&SWC!=0&SWD!=0)         //判断是否接收到遥控器数据
    {
        Robot_Control_Mode();   //状态机编写
        Free_Control();         //底盘控制
        Robot_Wheels_RPM_calculate();
        Upper_Control();        //上层机构控制
    }
    else
    {
        Robot_stop();
    }

}

void Upper_Control()
{
    //取球机构控制器
   // R2_CONTROLLER.CURRENT_FW_STATE=suction_controller(R2_CONTROLLER.NEXT_FW_STATE);
   TakeBall_Controller(R2_CONTROLLER.NEXT_FW_STATE);
    //筛球机构控制器
    //Ball_Process_Function(R2_CONTROLLER.NEXT_BP_STATE);
}

/**
 * @brief 机器人控制模式函数，手动or自动
*/
void Robot_Control_Mode(void)
{ 
    if (SWA >= 900 && SWA <= 1100)  //待机状态
    {
        Robot_stop();   //暂停机器人的所有机构执行 
    }
    else if(SWA >= 1900 && SWA <= 2100)   //启动状态
    {
            if(SWD>=1900 && SWD<=2100)  
            {
                //开启低移速模式
                ROBOT_CHASSI.SPEED_LIMI.Vw_MAX = 1.0f;
                ROBOT_CHASSI.SPEED_LIMI.Vx_MAX = 0.5f;
                ROBOT_CHASSI.SPEED_LIMI.Vy_MAX = 0.5f;
            }
            else if(SWD>=900 && SWD<=1100)
            {
                //关闭机器人的低移速模式
                ROBOT_CHASSI.SPEED_LIMI.Vw_MAX = 3.0f;
                ROBOT_CHASSI.SPEED_LIMI.Vx_MAX = 3.0f;
                ROBOT_CHASSI.SPEED_LIMI.Vy_MAX = 3.0f;
			}

            if(SWC>=950 && SWC<=1050)       //SWC上拨，机构停止
            {
                R2_CONTROLLER.NEXT_FW_STATE = FW_TAKE_BALL;    //取球机构停止
                //R2_CONTROLLER.NEXT_BP_STATE = BP_CONTROLLER_OFF;    //筛球机构停止
            }
            else if(SWC>=1450 && SWC<=1550) //SWC中拨，取球
            {
                R2_CONTROLLER.NEXT_FW_STATE = FW_CONTROLLER_OFF;    //取球机构停止
                //R2_CONTROLLER.NEXT_BP_STATE = BP_ABANDON_BALL;    //筛球机构启动
            }
            else if(SWC>1950 && SWC<2050)       //放球
            {
                R2_CONTROLLER.NEXT_FW_STATE = FW_INVERTED;    //取球机构反转
                //R2_CONTROLLER.NEXT_BP_STATE = BP_SHOOT_BALL;      //射球机构启动
            }            
    }
}


/**
 * @brief 关闭手柄的底盘控制，锁死底盘电机
*/
void Robot_stop(void)
{
    R2_CONTROLLER.NEXT_FW_STATE = FW_CONTROLLER_OFF;   //摩擦轮关闭
    R2_CONTROLLER.NEXT_BP_STATE = BP_CONTROLLER_OFF;   //滚筒机构关闭
    ROBOT_CHASSI.SPEED_LIMI.Vw_MAX = 0.0f;
    ROBOT_CHASSI.SPEED_LIMI.Vx_MAX = 0.0f;
    ROBOT_CHASSI.SPEED_LIMI.Vy_MAX = 0.0f;
}



