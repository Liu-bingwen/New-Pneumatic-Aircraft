#include <queueHandle.h>

void initSemaphores(void) {
    

    xSemaphorePMW3901 = xSemaphoreCreateBinary();   //A
    xSemaphoreBMI088 = xSemaphoreCreateBinary();      //B
    xSemaphoreVL53 = xSemaphoreCreateBinary();      //C
    xSemaphoreControl = xSemaphoreCreateBinary();      //D

    // 初始化后启动第一个任务的信号量
    xSemaphoreGive(xSemaphorePMW3901);
}