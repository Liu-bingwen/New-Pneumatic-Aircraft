/*
 * pmw3901.h
 *
 *  Created on: May 27, 2024
 *      Author: Administrator
 */

#ifndef INC_PMW3901_H_
#define INC_PMW3901_H_

#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "main.h"

typedef struct
{
    int16_t deltaX;
    int16_t deltaY;
    /* data */
}__attribute__((aligned(4))) pmw3901;


uint8_t PMW3901_init(void);
uint8_t WriteReg(uint8_t reg, uint8_t value);
uint8_t ReadReg(uint8_t reg);
uint8_t initRegisters(void);
void ReadMotion(int16_t *deltax, int16_t *deltay);
void pmw3901_read(void);
void TaskPMW3901(void *pvParameters);




#endif /* INC_PMW3901_H_ */
