#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
extern QueueHandle_t xQueuePMW3901;
extern QueueHandle_t xQueueACCE;
extern QueueHandle_t XQueueGYRO;
extern QueueHandle_t xQueueVL53;
extern QueueHandle_t uartQueue;
extern TickType_t bmi088_task_end_time;

extern SemaphoreHandle_t xSemaphorePMW3901;
extern SemaphoreHandle_t xSemaphoreBMI088;
extern SemaphoreHandle_t xSemaphoreControl;
extern SemaphoreHandle_t xSemaphoreVL53;
void initSemaphores(void);
