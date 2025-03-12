#include "control.h"
#include "queueHandle.h"

void move_up(void)
{
    MotorA_SetDirection(1);
    MotorC_SetDirection(1);
    MotorB_SetDirection(0);
    MotorD_SetDirection(0);

    MotorA_SetSpeed(500);
    
    MotorB_SetSpeed(500);

    MotorC_SetSpeed(500);

    MotorD_SetSpeed(500);
}

void move_down(void)
{
    MotorA_SetDirection(1);
    MotorC_SetDirection(1);
    MotorB_SetDirection(0);
    MotorD_SetDirection(0);

    MotorA_SetSpeed(500);

    MotorC_SetSpeed(500);

    MotorB_SetSpeed(0);

    MotorD_SetSpeed(0);
}

void move_left(void)
{
    MotorA_SetDirection(1);
    MotorC_SetDirection(1);
    MotorB_SetDirection(0);
    MotorD_SetDirection(0);

    MotorA_SetSpeed(500);
    MotorB_SetSpeed(300);
    MotorD_SetSpeed(500);
    MotorC_SetSpeed(300);
}

void move_right(void)
{
    MotorA_SetDirection(1);
    MotorC_SetDirection(1);
    MotorB_SetDirection(0);
    MotorD_SetDirection(0);

    MotorA_SetSpeed(300);
    MotorB_SetSpeed(500);
    MotorD_SetSpeed(300);
    MotorC_SetSpeed(500);
}

void move_forward(void)
{
    MotorA_SetDirection(1);
    MotorC_SetDirection(1);
    MotorB_SetDirection(0);
    MotorD_SetDirection(0);

    MotorA_SetSpeed(500);
    MotorD_SetSpeed(300);
    MotorC_SetSpeed(300);
    MotorB_SetSpeed(500);
}

void move_back(void)
{
    MotorA_SetDirection(1);
    MotorC_SetDirection(1);
    MotorB_SetDirection(0);
    MotorD_SetDirection(0);

    MotorA_SetSpeed(300);
    MotorC_SetSpeed(500);
    MotorD_SetSpeed(500);
    MotorB_SetSpeed(300);
}

void shut_down(void)
{
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
    MotorC_SetSpeed(0);
    MotorD_SetSpeed(0);
}
void keep_height(int d_distance)
{
    int distance = 0;
    if (xQueueReceive(xQueueVL53, &distance, pdMS_TO_TICKS(20)) == pdPASS)
    {
        if (distance < d_distance)
        {
            while (1)
            {
                if (xQueueReceive(xQueueVL53, &distance, pdMS_TO_TICKS(20)) == pdPASS)
                {
                    if (distance < d_distance)
                    {
                        move_up();
                    }
                }
                if(xQueueReceive(xQueueVL53, &distance, pdMS_TO_TICKS(20)) == pdPASS)
                {
                    if(distance >= d_distance)
                    {
                        shut_down();
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        if(distance > d_distance){
            while (1)
            {
                if (xQueueReceive(xQueueVL53, &distance, pdMS_TO_TICKS(20)) == pdPASS)
                {
                    if (distance > d_distance)
                    {
                        move_down();
                    }
                }
                if(xQueueReceive(xQueueVL53, &distance, pdMS_TO_TICKS(20)) == pdPASS)
                {
                    if(distance <= d_distance)
                    {
                        shut_down();
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }
}
