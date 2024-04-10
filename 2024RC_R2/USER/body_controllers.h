#ifndef __BODY_CONTROLLERS_H
#define __BODY_CONTROLLERS_H

#include "main.h"

typedef enum
{
    FW_INVERTED=2,  //吸球反转  应该无用
    FW_TAKE_BALL,   //吸球取球
    FW_CONTROLLER_OFF,  //吸球机构停止
    BP_SHOOT_BALL,  //滚筒出球放框
    BP_ABANDON_BALL,//滚筒筛球
	BP_INVERTED,    //滚筒反转  无用
    BP_CONTROLLER_OFF,  //滚筒机构关闭

    LINEAR_ACTUATOR_GO,         //无用
    LINEAR_ACTUATOR_BACK,       //无用    
    LINEAR_ACTUATOR_OFF,        //无用
    CONTROLLER_ERROR        //错误状态
}CONTROLLER_STATE;

typedef enum
{
	OPEN=7,
	CLOSE
}DOOR;


typedef struct R2_CONTROLLER_STR
{
    uint8_t ros_stm32_noconnected_cnt;
    uint8_t ros_stm32_noconnected_flag;

    int8_t if_use_auto; //是否接入上位机
    uint8_t is_ball_in_car; //是否有球在车上
    uint8_t FW_open_flag; //摩擦轮是否打开
    
    ROBOT_CHASSIS CHASSIS_CONTROLLER;
    CONTROLLER_STATE NEXT_BP_STATE; //滚筒装置的下个状态，ros决定
    CONTROLLER_STATE NEXT_FW_STATE; //吸球装置的下个状态，ros决定
    CONTROLLER_STATE CURRENT_PB_STATE; //滚筒装置的当前状态
    CONTROLLER_STATE CURRENT_FW_STATE; //吸球装置的当前状态
}R2_CONTROLLER_STR;

extern R2_CONTROLLER_STR R2_CONTROLLER;

void baffle_control(DOOR door);
int16_t Ball_Process_Function(CONTROLLER_STATE TARGET_STATE);
void TakeBall_Controller(CONTROLLER_STATE EXPECT_BALL_STA);
void Chassis_Controller(void);
void ROS_Control(void);
void liner_actuator(CONTROLLER_STATE EXPECT_STATE);
CONTROLLER_STATE suction_controller(CONTROLLER_STATE NEXT_STATE);


#endif
