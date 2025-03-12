// #include "tasks.h"
// #include "control.h"
// #include "pmw3901.h"
// #include "bmi08x_defs.h"
// #include "queueHandle.h"
// #define BUFFER_SIZE 50
// char rx_buffer1[BUFFER_SIZE];
// char rx_data1;
// QueueHandle_t uartQueue;
// char sprintfmessage[512] = {'\0'};
// int uart_busy = 0;

// void uartReceiveTask(void *params)
// {
//     // 初始化接受的变量
//     int distance;                    // VL53L1X
//     struct bmi08x_sensor_data accel; // BMNI088
//     struct bmi08x_sensor_data gyro;
//     pmw3901 user_pmw3901; // PMW3901
//     // 初始化消息
//     uartQueue = xQueueCreate(10, BUFFER_SIZE);

//     // sprintf(sprintfmessage, "Queue Init Success\n");
//     // HAL_UART_Transmit(&huart1, (uint8_t *)sprintfmessage, strlen(sprintfmessage),1000);
//     if (uartQueue == NULL)
//     {
//         // 错误处理：队列创建失败
//         sprintf(sprintfmessage, "uartQueue Error\n");
//         uart_busy = 1;
//         HAL_UART_Transmit_IT(&huart1, (uint8_t *)sprintfmessage, strlen(sprintfmessage));
//         return;
//     }

//     // 启动UART接收中断
//     HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_buffer1, sizeof(rx_buffer1));

//     while (1)
//     {
       
//         vTaskDelay(pdMS_TO_TICKS(20)); // 避免任务长期占用CPU
//     }
// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     HAL_UART_Transmit_IT(&huart1,(uint8_t *)rx_buffer1,sizeof(rx_buffer1));
    
//     // xQueueSendToBackFromISR(uartQueue, rx_buffer1, &xHigherPriorityTaskWoken);
//     // static int index = 0;

//     // if (huart->Instance == USART1)
//     // {
//     //     if (rx_data1 == '\n' || index >= BUFFER_SIZE - 1)
//     //     {
//     //         rx_buffer1[index] = '\0'; // 终止字符串

//     //         BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     //         // // 将接收到的数据发送到队列（使用 FromISR 版本的函数）
//     //         xQueueSendToBackFromISR(uartQueue, rx_buffer1, &xHigherPriorityTaskWoken);

//     //         //xQueueSendFromISR(uartQueue, &rx_data, NULL);

//     //          memset(rx_buffer1, 0, BUFFER_SIZE);
//     //         // // 判断是否需要进行任务切换
//     //         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

//     //         index = 0; // 重置缓冲区
//     //     }
//     //     else
//     //     {
//     //         rx_buffer1[index++] = rx_data1; // 将字符存入缓冲区
//     //     }

//         // 再次启动接收中断
//         HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_buffer1, sizeof(rx_buffer1));
//     // }
// }

// void HAL_UART_TxCpltCallback1(UART_HandleTypeDef *huart)
// {
//     // 发送完成时的回调函数
//     if (huart->Instance == USART1)
//     {
//         uart_busy = 0; // 标记UART不再忙碌，可以发送下一个数据
//     }
// }

// void commandProcessTask(void *params)
// {
//     char str1[20];
//     char str2[20];
//     int distance;
//     char command[BUFFER_SIZE];
//     // sprintf(sprintfmessage, "IN COMMMAND\n");
//     // HAL_UART_Transmit(&huart1, (uint8_t *)sprintfmessage, strlen(sprintfmessage), 1000);
//     HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_buffer1, sizeof(rx_buffer1));



//     while (1)
//     {

//         if (xQueueReceive(uartQueue, &command, pdMS_TO_TICKS(20)) == pdPASS)
//         {

//             sprintf(sprintfmessage, "接收到信息\n");
//             HAL_UART_Transmit(&huart1, (uint8_t *)sprintfmessage, strlen(sprintfmessage),1000);
//             if (strcmp(command, "up") == 0)
//             {
//                 move_up();
//             }
//             else if (strcmp(command, "down") == 0)
//             {
//                 move_down();
//             }
//             else if (strcmp(command, "left") == 0)
//             {
//                 move_left();
//             }
//             else if (strcmp(command, "right") == 0)
//             {
//                 move_right();
//             }
//             else if (strcmp(command, "forward") == 0)
//             {
//                 move_forward();
//             }
//             else if (strcmp(command, "back") == 0)
//             {
//                 move_back();
//             }
//             else if (strcmp(command, "shut down") == 0)
//             {

//                 shut_down();
//             }
//             else 
//                 if(sscanf(command,"%s %s %d",str1,str2,&distance) == 3)
                
//                 {
//                     if((strcmp(str1,"keep") == 0) && (strcmp(str2,"height") == 0))
//                     {
//                         keep_height(distance);
//                     }
//                 }
//         }
//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }
