#include "haoying_u8.h"


/**
 * @brief 好盈电调初始化程序
*/
void HaoYing_U8_Init(void)
{
    HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);   //开启PWM通道
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);

    // u8_speed_set(100,U8_PWM_PIN6);     //初始化好盈电调，先把油门拉满，网上是这么说的。但是亲测好像不用，留待后来者观察一下
    // u8_speed_set(100,U8_PWM_PIN6);     //加入此代码，会让电机初始化时冲一下子，很短暂
    // u8_speed_set(100,N5065_PWM_PIN9);
    
    u8_speed_set(0,U8_PWM_PIN6);     //然后踩刹车
    u8_speed_set(0,U8_PWM_PIN8);
    u8_speed_set(0,N5065_PWM_PIN9);
    HAL_Delay(100);
}


/**
 * @brief 设置U8电机的速度，实际上是踩油门。好盈电调是类似于一个模拟油门的控制器
 * @param 油门值（0~100）
*/
void u8_speed_set(int16_t pulse_value, PWM_PIN PIN)
{
    switch(PIN)
    {
        case U8_PWM_PIN6:   //PF6
        {
            
            TIM10->CCR1 = 2000 + pulse_value*20;    //由于TIM10挂载在APB2总线(为168MHz)
            break;
        }

        case U8_PWM_PIN8:   //PF8
        {
            TIM13->CCR1 = 1000 + pulse_value*10;    //由于TIM13挂载在APB1总线(为84MHz),所以二者的
            break;
        }

        case N5065_PWM_PIN9:    //PF9
        {
            TIM14->CCR1 = 1000 + pulse_value*10;    //由于TIM14挂载在APB1总线(为84MHz),所以二者的
            break;
        }
        
        default:
            TIM10->CCR1 = 2000;
            TIM13->CCR1 = 1000;
            TIM14->CCR1 = 1000;
            break;
    }
}
