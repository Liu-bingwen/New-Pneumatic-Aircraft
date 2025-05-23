/*
 * pmw3901.c
 *
 *  Created on: May 27, 2024
 *      Author: Administrator
 */

#include "pmw3901.h"
#include "queueHandle.h"
int16_t deltax = 0;
int16_t deltay = 0;

uint8_t message[512] = {'\0'};

pmw3901 sensorData;

uint8_t PMW3901_init()
{
	HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(40);

	uint8_t error = 0;
	// POWER On RESET
	error = WriteReg(0x3A, 0x5A);
	if (error)
	{
		while (1)
		{
			sprintf(message, "Write Failed 1");
			HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
			HAL_Delay(1000);
		}
		return error;
	}
	HAL_Delay(50);

	// Read PRODUCT_ID

	uint8_t chipID = 0x49;
	int i = 0;
	while (1)
	{
		chipID = ReadReg(0x00);
		if (chipID != 0x49)
		{
			// while(1){

			sprintf(message, "CHIP_ID: %X\tLoop: %d", chipID, i);
			HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
			i++;
			HAL_Delay(500);
			//}
		}
		else
		{
			sprintf(message, "CHIP_ID == 0x49 ID = %X\tLoop: %d", chipID, i);
			HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
			i++;
			HAL_Delay(300);
			break;
		}
		if (i == 60)
			return 1;
	}

	HAL_Delay(10);

	// Read PMW_REG_INVERSE_PRODUCT_ID
	// uint8_t inverID = 0;
	// inverID = ReadReg(0x5f);

	uint8_t error1 = 0;
	ReadReg(0x02);
	ReadReg(0x03);
	ReadReg(0x04);
	ReadReg(0x05);
	ReadReg(0x06);

	HAL_Delay(10);
	error1 = initRegisters();

	if (error1)
	{
		return 1;
	}
}

// uint8_t WriteReg(uint8_t reg, uint8_t value){
//		uint8_t Error = 0x01;
//
//	    // Select the SPI device by pulling CS pin low
//	    HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_RESET);
//
//	    // Construct the data to be transmitted
//	    uint8_t data[2] = {reg | 0x80, value}; // Address byte with write operation
//
//	    // Transmit data over SPI
//	    Error = HAL_SPI_Transmit(&hspi1, data, sizeof(data), HAL_MAX_DELAY);
//
//	    HAL_Delay(5);
//	    // Check for transmission error
//	    if (Error != HAL_OK) {
//	        // SPI transmission failed
//	        HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET); // Deselect SPI device
//	        // Handle error or log message
//	        while(1){
//	        	sprintf(message, "Write Failed: %d", Error);
//	        	HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
//	        	HAL_Delay(1000);
//	        }
//
//	        return 1;
//	    }
//
//	    // Deselect the SPI device by pulling CS pin high
//	    HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET);
//
//	    return 0; // Success
// }
uint8_t WriteReg(uint8_t reg, uint8_t value)
{
	uint8_t Error = 0x01;

	uint8_t txData[2];
	txData[0] = reg | 0x80; // Set MSB to 1 for write operation
	txData[1] = value;

	HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_RESET); // CS low
	Error = HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
	// Check for transmission error
	if (Error != HAL_OK)
	{
		// SPI transmission failed
		HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET); // Deselect SPI device
		// Handle error or log message
		while (1)
		{
			sprintf(message, "Write Failed");
			HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
			HAL_Delay(1000);
		}
		// return 1;
	}

	// Deselect the SPI device by pulling CS pin high
	HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET);

	return 0; // Success
}

// uint8_t ReadReg(uint8_t reg){
//		uint8_t txData = reg & 0x7f; // Set MSB to 0 for read operation
//		uint8_t rxData ;
//	    uint8_t dummy[16] = {0x00};
//	    uint8_t Error = 0x01;
//
//	    // Select the SPI device by pulling CS pin low
//	    HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_RESET);
//	    HAL_Delay(5);
//
//
//	    Error = HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, HAL_MAX_DELAY);
//	    if (Error != HAL_OK) {
//	        // SPI reception failed
//	        HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET); // Deselect SPI device
//	        while(1){
//	        	sprintf(message, "Read Failed: Receive : %d", Error);
//	        	HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
//	        	HAL_Delay(1000);
//	        }
//	        return 0xff;
//	    }
//
//	    // Deselect the SPI device by pulling CS pin high
//	    HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET);
//
//	    return rxData;
// }
uint8_t ReadReg(uint8_t reg)
{
	uint8_t txData = reg & 0x7F; // Set MSB to 0 for read operation
	uint8_t rxData = 0;
	uint8_t Error = 0x01;
	uint8_t dummy[16] = {0x00};

	// Select the SPI device by pulling CS pin low
	HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_RESET);

	// Transmit address byte to read from register
	Error = HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
	if (Error != HAL_OK)
	{
		// SPI transmission failed
		HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET); // Deselect SPI device
		return 0xFF;
	}

	// Add a small delay to ensure PMW3901 has time to prepare data
	HAL_Delay(1); // Adjust delay as necessary

	// Receive data from register
	Error = HAL_SPI_TransmitReceive(&hspi1, dummy, &rxData, 1, 1000);
	if (Error != HAL_OK)
	{
		// SPI reception failed
		HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET); // Deselect SPI device
		return 0xFF;
	}

	// Deselect the SPI device by pulling CS pin high
	HAL_GPIO_WritePin(PIN_CS_GPIO_Port, PIN_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(1);

	return rxData;
}

uint8_t initRegisters()
{

	uint8_t HAL_Error = 0;
	HAL_Error = WriteReg(0x7F, 0x00);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x61, 0xAD);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x03);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x40, 0x00);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x05);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x41, 0xB3);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x43, 0xF1);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x45, 0x14);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x5B, 0x32);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x5F, 0x34);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7B, 0x08);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x06);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x44, 0x1B);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x40, 0xBF);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4E, 0x3F);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x08);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x65, 0x20);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x6A, 0x18);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x09);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4F, 0xAF);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x5F, 0x40);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x48, 0x80);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x49, 0x80);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x57, 0x77);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x60, 0x78);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x61, 0x78);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x62, 0x08);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x63, 0x50);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x0A);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x45, 0x60);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x00);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4D, 0x11);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x55, 0x80);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x74, 0x1F);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x75, 0x1F);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4A, 0x78);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4B, 0x78);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x44, 0x08);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x45, 0x50);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x64, 0xFF);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x65, 0x1F);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x14);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x65, 0x60);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x66, 0x08);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x63, 0x78);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x15);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x48, 0x58);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x07);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x41, 0x0D);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x43, 0x14);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4B, 0x0E);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x45, 0x0F);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x44, 0x42);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4C, 0x80);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x10);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x5B, 0x02);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x07);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x40, 0x41);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x70, 0x00);
	if (HAL_Error)
		return HAL_Error;

	HAL_Delay(100);
	HAL_Error = WriteReg(0x32, 0x44);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x07);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x40, 0x40);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x06);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x62, 0xf0);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x63, 0x00);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x0D);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x48, 0xC0);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x6F, 0xd5);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x7F, 0x00);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x5B, 0xa0);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x4E, 0xA8);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x5A, 0x50);
	if (HAL_Error)
		return HAL_Error;
	HAL_Error = WriteReg(0x40, 0x80);
	if (HAL_Error)
		return HAL_Error;

	return 0;
}

void ReadMotion(int16_t *deltax, int16_t *deltay)
{
	ReadReg(0x02);
	*deltax = ((int16_t)ReadReg(0x04) << 8) | ReadReg(0x03);
	*deltay = ((int16_t)ReadReg(0x06) << 8) | ReadReg(0x05);
}

void pmw3901_read()
{
	ReadMotion(&deltax, &deltay);
	if (deltax >= -32640 && deltax < -30000)
	{
		deltax = deltax + 32640;
	}

	if (deltay >= -32640 && deltay < -30000)
	{
		deltay = deltay + 32640;
	}
	sensorData.deltaX = deltax;
	sensorData.deltaY = deltay;
	// sprintf(message, "deltaX: %d \t deltaY: %d \n", deltax,deltay);
	// HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
}

void TaskPMW3901(void *pvParameters)
{
	// TickType_t start_tick, end_tick;

	for (;;)
	{
		xSemaphoreTake(xSemaphorePMW3901,portMAX_DELAY);
		// start_tick = xTaskGetTickCount();
		pmw3901_read();
		xQueueSend(xQueuePMW3901, &sensorData, portMAX_DELAY);
		// sprintf(message, "deltaX: %d \t deltaY: %d \n", sensorData.deltaX,sensorData.deltaY);
		// HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
		// end_tick = xTaskGetTickCount();
		// TickType_t execution_time = end_tick - start_tick;

		// sprintf(message, "PMW3901 time: %lu ms\n", execution_time * portTICK_PERIOD_MS);
		// HAL_UART_Transmit(&huart1, message, strlen(message), 1000);

		xSemaphoreGive(xSemaphoreBMI088);

		vTaskDelay(pdMS_TO_TICKS(10)); // ??????????????
	}
}
