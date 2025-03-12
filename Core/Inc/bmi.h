/*
 * bmi.h
 *
 *  Created on: May 14, 2024
 *      Author: Administrator
 */

#ifndef INC_BMI_H_
#define INC_BMI_H_
#include "main.h"
#include "bmi08x.h"
#include "bmi088.h"
#include "i2c.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
// 外部变量声明
extern int8_t rslt;
extern uint8_t data;
extern int32_t bmi08x_sensor_temp;
extern struct bmi08x_sensor_data user_accel_bmi088;
extern struct bmi08x_sensor_data user_gyro_bmi088;
extern float accRange;
extern float gyroRange;

// 函数声明
int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void bmi088_read(void);

// 外部结构体声明
extern struct bmi08x_dev dev;

void bmi088_read(void);
void bmi088_starting(void);
void TaskBMI088(void *pvParameters);
#endif /* INC_BMI_H_ */
