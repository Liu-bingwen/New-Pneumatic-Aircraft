#include "main.h"


//设置A电机的方向
void MotorA_SetDirection(uint8_t dir);

//B电机方向
void MotorB_SetDirection(uint8_t dir);

//C电机方向
void MotorC_SetDirection(uint8_t dir);

//D电机方向
void MotorD_SetDirection(uint8_t dir);

//A电机转速
void MotorA_SetSpeed(float speed);

//B电机转速
void MotorB_SetSpeed(float speed);

//C电机转速
void MotorC_SetSpeed(float speed);

//D电机转速
void MotorD_SetSpeed(float speed);

//电机初始化
void Motor_Init(void);
