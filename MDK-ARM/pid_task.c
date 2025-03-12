// #include "pid_task.h"
// #include "cmsis_os.h"
// #include <string.h>
// #include "queueHandle.h"

// // PID 控制器实例
// PIDController pid_ax;
// PIDController pid_ay;
// PIDController pid_az;
// PIDController pid_tz;

// // FreeRTOS 任务 & 资源
// osThreadId_t pidTaskHandle;
// QueueHandle_t commandQueue;
// SemaphoreHandle_t xSemaphoreControl;

// // PID 任务
// void PID_Task(void *argument) {
//     float command[3]; // 用于存储接收的 PID 参数

//     for (;;) {
//         // 从队列中接收 PID 参数
//         if (xQueueReceive(commandQueue, command, portMAX_DELAY) == pdTRUE) {
//             // 更新 PID 参数
//             pid_ax.Kp = command[0];
//             pid_ax.Ki = command[1];
//             pid_ax.Kd = command[2];

//             pid_ay.Kp = command[0];
//             pid_ay.Ki = command[1];
//             pid_ay.Kd = command[2];

//             pid_az.Kp = command[0];
//             pid_az.Ki = command[1];
//             pid_az.Kd = command[2];

//             pid_tz.Kp = command[0];
//             pid_tz.Ki = command[1];
//             pid_tz.Kd = command[2];

//             // 通知控制任务 PID 计算可以开始
//             xSemaphoreGive(xSemaphoreControl);
//         }
//     }
// }

// // 任务初始化
// void PID_Task_Init(void) {
//     // 初始化 PID 控制器
//     PIDController_Init(&pid_ax);
//     PIDController_Init(&pid_ay);
//     PIDController_Init(&pid_az);
//     PIDController_Init(&pid_tz);

//     // 创建队列 (存储 5 组 PID 参数，每组 3 个 float)
//     commandQueue = xQueueCreate(5, sizeof(float) * 3);

//     // 创建信号量
//     xSemaphoreControl = xSemaphoreCreateBinary();

//     // 创建任务
//     pidTaskHandle = osThreadNew(PID_Task, NULL, &((osThreadAttr_t){
//         .name = "PID_Task",
//         .priority = osPriorityHigh,
//         .stack_size = 512
//     }));
// }
