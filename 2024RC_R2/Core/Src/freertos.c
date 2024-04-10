/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_PRINTF_ENABLE 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ROS_32_Serial */
osThreadId_t ROS_32_SerialHandle;
const osThreadAttr_t ROS_32_Serial_attributes = {
  .name = "ROS_32_Serial",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Motor_Control */
osThreadId_t Motor_ControlHandle;
const osThreadAttr_t Motor_Control_attributes = {
  .name = "Motor_Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for U8_and_VESC */
osThreadId_t U8_and_VESCHandle;
const osThreadAttr_t U8_and_VESC_attributes = {
  .name = "U8_and_VESC",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for upper_control */
osThreadId_t upper_controlHandle;
const osThreadAttr_t upper_control_attributes = {
  .name = "upper_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for chassis_control */
osThreadId_t chassis_controlHandle;
const osThreadAttr_t chassis_control_attributes = {
  .name = "chassis_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for uart_queue */
osMessageQueueId_t uart_queueHandle;
const osMessageQueueAttr_t uart_queue_attributes = {
  .name = "uart_queue"
};
/* Definitions for safe_printf */
osMutexId_t safe_printfHandle;
const osMutexAttr_t safe_printf_attributes = {
  .name = "safe_printf"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void debug_safe_printf(const char *format, ...)
{
#if DEBUG_PRINTF_ENABLE
  
  HAL_UART_MspInit(&huart5);

  osStatus xReturn;
  va_list args;
  va_start(args,format);

  xReturn = osMutexAcquire(safe_printfHandle,portMAX_DELAY);
  if(xReturn == osOK)
  {
    vprintf(format, args);
  }
  xReturn = osMutexRelease(safe_printfHandle);

#else
  HAL_UART_MspDeInit(&huart5);//为了用uart5进行串口数据传输的调试，不用时关�?
  (void)0;
#endif
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ROS_32_Serial_Function(void *argument);
void Motor_Control_Function(void *argument);
void U8_and_VESC_Function(void *argument);
void upper_control_f(void *argument);
void chassis_control_f(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of safe_printf */
  safe_printfHandle = osMutexNew(&safe_printf_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart_queue */
  uart_queueHandle = osMessageQueueNew (4, sizeof(uint8_t), &uart_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ROS_32_Serial */
  ROS_32_SerialHandle = osThreadNew(ROS_32_Serial_Function, NULL, &ROS_32_Serial_attributes);

  /* creation of Motor_Control */
  Motor_ControlHandle = osThreadNew(Motor_Control_Function, NULL, &Motor_Control_attributes);

  /* creation of U8_and_VESC */
  U8_and_VESCHandle = osThreadNew(U8_and_VESC_Function, NULL, &U8_and_VESC_attributes);

  /* creation of upper_control */
  upper_controlHandle = osThreadNew(upper_control_f, NULL, &upper_control_attributes);

  /* creation of chassis_control */
  chassis_controlHandle = osThreadNew(chassis_control_f, NULL, &chassis_control_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    extern uint8_t test;
  /* Infinite loop */
  for(;;)
  {
    /* Infinite loop */
    for(;;)
    {
        //????
       // Fsm32();     
      osDelay(10);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ROS_32_Serial_Function */
/**
* @brief Function implementing the ROS_32_Serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ROS_32_Serial_Function */
void ROS_32_Serial_Function(void *argument)
{
  /* USER CODE BEGIN ROS_32_Serial_Function */
  
  /* Infinite loop */
  for(;;)
  {
    //the communication between ros and stm32
    Usart_Send_Data(R2_CONTROLLER.CURRENT_PB_STATE,R2_CONTROLLER.CURRENT_FW_STATE,R2_CONTROLLER.is_ball_in_car,0,1,0);  
    osDelay(50);
  }
  /* USER CODE END ROS_32_Serial_Function */
}

/* USER CODE BEGIN Header_Motor_Control_Function */
/**
* @brief Function implementing the Motor_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Control_Function */
void Motor_Control_Function(void *argument)
{
  /* USER CODE BEGIN Motor_Control_Function */

  /* Infinite loop */
  for(;;)
  {        
    Motor_Control();
    osDelay(1);             
  }
  /* USER CODE END Motor_Control_Function */
}

/* USER CODE BEGIN Header_U8_and_VESC_Function */
/**
* @brief Function implementing the U8_and_VESC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_U8_and_VESC_Function */
void U8_and_VESC_Function(void *argument)
{
  /* USER CODE BEGIN U8_and_VESC_Function */
  static int8_t cnt=0;
  /* Infinite loop */
  for(;;)
  {
    for(int i=0;i<2;i++)
      VESC_Control(&VESC_MOTO_INFO[i+cnt]);         //0 1 2 3 4 5
    cnt+=2;
    if(cnt>=6)
      cnt=0;
    osDelay(1);
  }
  /* USER CODE END U8_and_VESC_Function */
}

/* USER CODE BEGIN Header_upper_control_f */
/**
* @brief Function implementing the upper_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_upper_control_f */
void upper_control_f(void *argument)
{
  /* USER CODE BEGIN upper_control_f */
  /* Infinite loop */
  for(;;)
  {
    //?????ros?
    ROS_Control();
    osDelay(5);
  }
  /* USER CODE END upper_control_f */
}

/* USER CODE BEGIN Header_chassis_control_f */
/**
* @brief Function implementing the chassis_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_control_f */
void chassis_control_f(void *argument)
{
  /* USER CODE BEGIN chassis_control_f */
  /* Infinite loop */
  for(;;)
  { 
    //?????ros?
    Chassis_Controller(); 
    osDelay(5);
  }
  /* USER CODE END chassis_control_f */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

