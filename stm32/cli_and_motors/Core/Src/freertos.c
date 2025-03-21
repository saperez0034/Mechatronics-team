/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cli_app.h"
#include "stepper.h"
#include "actuators.h"
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
osThreadId_t cmdLineTaskHandle; // new command line task
const osThreadAttr_t cmdLineTask_attributes = {
  .name = "cmdLineTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
osThreadId_t stepperXTaskHandle; // new command line task
const osThreadAttr_t stepperXTask_attributes = {
  .name = "stepperXTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
osThreadId_t stepperYTaskHandle; // new command line task
const osThreadAttr_t stepperYTask_attributes = {
  .name = "stepperYTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
osThreadId_t rotServoTaskHandle; // new command line task
const osThreadAttr_t rotServoTask_attributes = {
  .name = "rotServoTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
osThreadId_t linServoTaskHandle; // new command line task
const osThreadAttr_t linServoTask_attributes = {
  .name = "linServoTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
osThreadId_t linActTaskHandle; // new command line task
const osThreadAttr_t linActTask_attributes = {
  .name = "linActTask", // defined in cli_app.c
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  cmdLineTaskHandle = osThreadNew(vCommandConsoleTask, NULL, &cmdLineTask_attributes);
  stepperXTaskHandle = osThreadNew(vStepperControlX, NULL, &stepperXTask_attributes);
  stepperYTaskHandle = osThreadNew(vStepperControlY, NULL, &stepperYTask_attributes);
  rotServoTaskHandle = osThreadNew(vServoControl, NULL, &rotServoTask_attributes);
  linServoTaskHandle = osThreadNew(vLinServoControl, NULL, &linServoTask_attributes);
  linActTaskHandle = osThreadNew(vLinActControl, NULL, &linActTask_attributes);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

