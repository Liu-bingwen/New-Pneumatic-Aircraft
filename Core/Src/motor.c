#include "motor.h"
#include "math.h"

void MotorA_SetDirection(uint8_t dir)
{
    if (dir == 0) // 正转
    {
        HAL_GPIO_WritePin(AIN1_PIN_GPIO_Port, AIN1_PIN_Pin, GPIO_PIN_SET);   // AIN1 = 1
        HAL_GPIO_WritePin(AIN2_PIN_GPIO_Port, AIN2_PIN_Pin, GPIO_PIN_RESET); // AIN2 = 0
    }
    else if (dir == 1) // 反转
    {
        HAL_GPIO_WritePin(AIN1_PIN_GPIO_Port, AIN1_PIN_Pin, GPIO_PIN_RESET); // AIN1 = 0
        HAL_GPIO_WritePin(AIN2_PIN_GPIO_Port, AIN2_PIN_Pin, GPIO_PIN_SET);   // AIN2 = 1
    }
}

void MotorC_SetDirection(uint8_t dir)
{
    if (dir == 1) // 正转
    {
        HAL_GPIO_WritePin(BIN1_PIN_GPIO_Port, BIN1_PIN_Pin, GPIO_PIN_SET);   // AIN1 = 1
        HAL_GPIO_WritePin(BIN2_PIN_GPIO_Port, BIN2_PIN_Pin, GPIO_PIN_RESET); // AIN2 = 0
    }
    else if (dir == 0) // 反转
    {
        HAL_GPIO_WritePin(BIN1_PIN_GPIO_Port, BIN1_PIN_Pin, GPIO_PIN_RESET); // AIN1 = 0
        HAL_GPIO_WritePin(BIN2_PIN_GPIO_Port, BIN2_PIN_Pin, GPIO_PIN_SET);   // AIN2 = 1
    }
}
void MotorB_SetDirection(uint8_t dir)
{
    if (dir == 0) // 正转
    {
        HAL_GPIO_WritePin(AIN21_PIN_GPIO_Port, AIN21_PIN_Pin, GPIO_PIN_SET);   // AIN1 = 1
        HAL_GPIO_WritePin(AIN22_PIN_GPIO_Port, AIN22_PIN_Pin, GPIO_PIN_RESET); // AIN2 = 0
    }
    else if (dir == 1) // 反转
    {
        HAL_GPIO_WritePin(AIN21_PIN_GPIO_Port, AIN21_PIN_Pin, GPIO_PIN_RESET); // AIN1 = 0
        HAL_GPIO_WritePin(AIN22_PIN_GPIO_Port, AIN22_PIN_Pin, GPIO_PIN_SET);   // AIN2 = 1
    }
}
void MotorD_SetDirection(uint8_t dir)
{
    if (dir == 1) // 正转
    {
        HAL_GPIO_WritePin(BIN21_PIN_GPIO_Port, BIN21_PIN_Pin, GPIO_PIN_SET);   // AIN1 = 1
        HAL_GPIO_WritePin(BIN22_PIN_GPIO_Port, BIN22_PIN_Pin, GPIO_PIN_RESET); // AIN2 = 0
    }
    else if (dir == 0) // 反转
    {
        HAL_GPIO_WritePin(BIN21_PIN_GPIO_Port, BIN21_PIN_Pin, GPIO_PIN_RESET); // AIN1 = 0
        HAL_GPIO_WritePin(BIN22_PIN_GPIO_Port, BIN22_PIN_Pin, GPIO_PIN_SET);   // AIN2 = 1
    }
}

void MotorA_SetSpeed(float speed)
{
    // if (speed < 0)
    //     speed = fabs(speed);
    uint16_t speed_t = (uint16_t)speed;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed_t);
}

void MotorC_SetSpeed(float speed)
{
    // if (speed < 0)
    //     speed = fabs(speed);
    uint16_t speed_t = (uint16_t)speed;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed_t);
}
void MotorB_SetSpeed(float speed)
{
    // if (speed < 0)
    //     speed = fabs(speed);
    uint16_t speed_t = (uint16_t)speed;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed_t);
}
void MotorD_SetSpeed(float speed)
{
    // if (speed < 0)
    //     speed = fabs(speed);
    uint16_t speed_t = (uint16_t)speed;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed_t);
}

void Motor_Init(void)
{
    HAL_GPIO_WritePin(STBY_PIN_GPIO_Port, STBY_PIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(STBY2_PIN_GPIO_Port, STBY2_PIN_Pin, GPIO_PIN_SET);
    MotorA_SetDirection(1);
    MotorC_SetDirection(1);
    MotorB_SetDirection(1);
    MotorD_SetDirection(1);


    // while (1)
    // {
    //     MotorA_SetSpeed(60);
    //     HAL_Delay(3000);
    //     MotorA_SetSpeed(0);
    //     HAL_Delay(3000);
    //     MotorB_SetSpeed(60);
    //     HAL_Delay(3000);
    //     MotorB_SetSpeed(0);
    //     HAL_Delay(3000);
    //     MotorC_SetSpeed(60);
    //     HAL_Delay(3000);
    //     MotorC_SetSpeed(0);
    //     HAL_Delay(3000);
    //     MotorD_SetSpeed(60);
    //     HAL_Delay(3000);
    //     MotorD_SetSpeed(0);
    //     HAL_Delay(3000);
    //     // MotorA_SetSpeed(80);
    //     // MotorB_SetSpeed(0);
    //     // MotorC_SetSpeed(80);
    //     // MotorD_SetSpeed(0);

    //     // HAL_Delay(5000);

    //     // MotorA_SetSpeed(100);
    //     // MotorB_SetSpeed(100);
    //     // MotorC_SetSpeed(100);
    //     // MotorD_SetSpeed(100);

    //     HAL_Delay(5000);
    // }
}