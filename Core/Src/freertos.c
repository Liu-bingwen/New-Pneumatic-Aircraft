/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "queueHandle.h"
#include "StableControl.h"
#include "semphr.h"
#include "pmw3901.h"
#include "bmi08x_defs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
TaskHandle_t commandTaskHandle;
QueueHandle_t xQueuePMW3901;
QueueHandle_t xQueueACCE;
QueueHandle_t XQueueGYRO;
QueueHandle_t xQueueVL53;
static SemaphoreHandle_t g_xI2CMutex;
// SemaphoreHandle_t i2cMutex;
uint8_t Sprintmessage12[512] = {'\0'};
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
void GetI2C()
{
  xSemaphoreTake(g_xI2CMutex,portMAX_DELAY);
}
void PutI2C()
{
  xSemaphoreGive(g_xI2CMutex);
}
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

  xQueuePMW3901 = xQueueCreate(10, 10);
  xQueueACCE = xQueueCreate(10, 12);
  XQueueGYRO = xQueueCreate(10, 12);
  xQueueVL53 = xQueueCreate(10, sizeof(int));
  g_xI2CMutex = xSemaphoreCreateMutex();
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
  xTaskCreate(TaskPMW3901, "PMW3901 Task", 400, NULL, osPriorityNormal, NULL);
  xTaskCreate(TaskBMI088, "BMI088 Task", 400, NULL, osPriorityNormal, NULL);
  xTaskCreate(TaskVL53, "VL53 Task", 400, NULL, osPriorityNormal, NULL);
  // xTaskCreate(uartReceiveTask, "UART Receive Task", 400, NULL, osPriorityNormal, NULL);
  // xTaskCreate(commandProcessTask, "Command Process Task", 400, NULL, osPriorityNormal, &commandTaskHandle);
  xTaskCreate(FLIGHTControl, "Flight Control", 400, NULL, osPriorityNormal, NULL);
  // xTaskCreate(uartReceiveTask, "UART Receive Task", 128, NULL, osPriorityNormal, NULL);
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
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// void TaskPMW3901(void *pvParameters)
// {
//   for (;;)
//   {
//     pmw3901_read();
//     vTaskDelay(pdMS_TO_TICKS(200));
//   }
// }
// void TaskBMI088(void *pvParameters)
// {
//   for (;;)
//   {
//     bmi088_read();
//     vTaskDelay(pdMS_TO_TICKS(200));
//   }
// }
// void TaskVL53(void *pvParameters)
// {
//   for (;;)
//   {
//     vl53_read();
//     vTaskDelay(pdMS_TO_TICKS(200));
//   }
// }

/* USER CODE END Application */

