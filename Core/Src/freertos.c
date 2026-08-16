/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "action_api.h"
#include "chassis_main.h"
#include "computer_link.h"
#include "dt35_pnp_link.h"
#include "imu_main.h"
#include "lora_link.h"
#include "mcu_link.h"
#include "up_main.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for startupReminder */
osThreadId_t startupReminderHandle;
const osThreadAttr_t startupReminder_attributes = {
  .name = "startupReminder",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for liftTask */
osThreadId_t liftTaskHandle;
const osThreadAttr_t liftTask_attributes = {
  .name = "liftTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartStartupReminderTask(void *argument);
void StartChassisTask(void *argument);
void StartLiftTask(void *argument);
void StartCommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{

}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startupReminder */
  startupReminderHandle = osThreadNew(StartStartupReminderTask, NULL, &startupReminder_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);

  /* creation of liftTask */
  liftTaskHandle = osThreadNew(StartLiftTask, NULL, &liftTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartStartupReminderTask */
/**
  * @brief  Function implementing the startupReminder thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStartupReminderTask */
__weak void StartStartupReminderTask(void *argument)
{
  /* USER CODE BEGIN StartStartupReminderTask */
  (void)argument;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* 每次 MCU 复位后重新播放两声启动提示音。 */
  for (uint32_t beep = 0U; beep < 2U; beep++)
  {
for (uint32_t cycle = 0U; cycle < 80U; cycle++)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  (void)osDelay(1U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
  (void)osDelay(1U);
}

    (void)osDelay(80U);
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
  osThreadExit();
  /* USER CODE END StartStartupReminderTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
  * @brief  Function implementing the chassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartChassisTask */
__weak void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  uint32_t next_tick = osKernelGetTickCount();
  HAL_StatusTypeDef chassis_result;
  HAL_StatusTypeDef imu_result;

  (void)argument;
  imu_result = ImuMain_Init();
  chassis_result = Chassis_Init();

  /* Infinite loop */
  for(;;)
  {
    if (imu_result == HAL_OK)
    {
      ImuMain_Run1ms();
    }
    if (chassis_result == HAL_OK)
    {
      Chassis_Run1ms();
    }

    next_tick += 1U;
    (void)osDelayUntil(next_tick);
  }
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartLiftTask */
/**
* @brief Function implementing the liftTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLiftTask */
__weak void StartLiftTask(void *argument)
{
  /* USER CODE BEGIN StartLiftTask */
  uint32_t next_tick = osKernelGetTickCount();
  HAL_StatusTypeDef up_result;

  (void)argument;
  up_result = Up_Init();
  /* Infinite loop */
  for(;;)
  {
    if (up_result == HAL_OK)
    {
      Up_Run1ms();
      Action_Run1ms();
    }

    next_tick += 1U;
    (void)osDelayUntil(next_tick);
  }
  /* USER CODE END StartLiftTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
__weak void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  (void)argument;
  (void)ComputerLink_Init(&huart4);
  (void)DT35PnpLink_Init(&huart9);
  (void)LoraLink_Init(&huart7);
  (void)McuLink_Init(&huart6);

  /* Infinite loop */
  for(;;)
  {
    DT35PnpLink_Run();
    LoraLink_Run();
    McuLink_Run();
    Action_UpdatePnp(pnp_link[SENSOR_LINK_F_INDEX].trigger,
                     pnp_link[SENSOR_LINK_L_B_INDEX].trigger);
    ComputerLink_Run();
    osDelay(1);
  }
  /* USER CODE END StartCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

