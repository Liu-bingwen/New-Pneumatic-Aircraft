
#include <bmi.h>
#include <usart.h>
#include <string.h>
#include <queueHandle.h>
#include "semphr.h"

// #ifdef __GNUC__
//	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
// #else
//	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
// #endif
// PUTCHAR_PROTOTYPE
//{
//		HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
//		return ch;
// }
/*检测加速度和陀螺仪*/

int8_t rslt;
uint8_t data = 0;
int32_t bmi08x_sensor_temp;
struct bmi08x_sensor_data user_accel_bmi088;
struct bmi08x_sensor_data user_gyro_bmi088;
struct bmi08x_sensor_data_f accel_bmi088_f;
struct bmi08x_sensor_data_f gyro_bmi088_f;
uint8_t message1[512] = {'\0'};
uint8_t in = 0;
float accRange;
float gyroRange;
TickType_t bmi088_task_end_time;

struct bmi08x_dev dev = {
	.accel_id = BMI08X_ACCEL_I2C_ADDR_SECONDARY,
	.gyro_id = BMI08X_GYRO_I2C_ADDR_SECONDARY,
	.intf = BMI08X_I2C_INTF,
	.read = &stm32_i2c_read,
	.write = &stm32_i2c_write,
	.delay_ms = &HAL_Delay};

void bmi088_starting()
{

	//	printf("BMI088 I2C Test\n");
	//	printf("Bolgen Studio\n");

	/* Initializing the bmi088 sensors the below function will Initialize both accel and gyro sensors */
	rslt = bmi088_init(&dev);

	if (rslt == BMI08X_OK)
	{
		/* Read accel chip id */
		rslt = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &dev);
		//		printf("Accel Initialization OK\n");
		//		printf("Accel Chip ID: 0x%02X\n",data);

		// sprintf(message1, "Accel Chip ID: 0x%02X\n",data);
		// HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);
		// HAL_Delay(1000);

		if (rslt == BMI08X_OK)
		{
			/* Read gyro chip id */
			rslt = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &dev);
			//			printf("GYRO Initialization OK\n");
			//			printf("GYRO Chip ID: 0x%02X\n",data);
			// sprintf(message1, "GYRO Chip ID: 0x%02X\n",data);
			// HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);
			// HAL_Delay(1000);
		}
	}
	else
	{

		//		printf("BMI088 Initialization Error\n");
		while (1)
		{
			sprintf(message1, "BMI088 Initialization Error\n");
			HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);
			HAL_Delay(1000);
		}
	}

	/* Perform soft reset */
	rslt = bmi08a_soft_reset(&dev);
	if (rslt != BMI08X_OK)
	{

		//		printf("BMI088 Soft Reset Error\n");
		while (1)
		{
			sprintf(message1, "BMI088 Soft Reset Error\n");
			HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);
			HAL_Delay(1000);
		}
	}

	/* Read the accel power mode */
	rslt = bmi08a_get_power_mode(&dev);
	/* Read the accel sensor config parameters (odr,bw,range) */
	rslt = bmi08a_get_meas_conf(&dev);
	/* Initialize the device instance as per the initialization example */

	/* Assign the desired configurations */
	dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;

	rslt = bmi08a_set_power_mode(&dev);

	/* Wait for 10ms to switch between the power modes - delay taken care inside the function */
	rslt = bmi08a_set_meas_conf(&dev);

	/* Configuring the gyro	 */
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

	rslt = bmi08g_set_power_mode(&dev);
	/* Wait for 30ms to switch between the power modes - delay taken care inside the function */

	/* Assign the desired configurations */
	dev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
	dev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;

	rslt = bmi08g_set_meas_conf(&dev);

	switch (dev.accel_cfg.range)
	{
	case BMI088_ACCEL_RANGE_3G:
		accRange = 3000;
		break;
	case BMI088_ACCEL_RANGE_6G:
		accRange = 6000;
		break;
	case BMI088_ACCEL_RANGE_12G:
		accRange = 12000;
		break;
	case BMI088_ACCEL_RANGE_24G:
		accRange = 24000;
		break;
	default:
		accRange = 3000;
		break;
	}
}
typedef int8_t (*bmi08x_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t stm32_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr, 1, data, len, 100);
	while (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY)
		;

	return 0;
}

int8_t stm32_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr, 1, data, len, 100);

	return 0;
}

void bmi088_read()
{

	/*读取数据*/
	//		while(1){
	/* Read the sensor data into the sensor data instance */
	rslt = bmi08a_get_data(&user_accel_bmi088, &dev);
	/* Read the sensor data into the sensor data instance */
	rslt = bmi08g_get_data(&user_gyro_bmi088, &dev);

	// 3G：3/32768*9.7936 = 0.000896630859375
	accel_bmi088_f.x = user_accel_bmi088.x * 0.000896630859375; //* 0.000896630859375;
	accel_bmi088_f.y = user_accel_bmi088.y * 0.000896630859375; //* 0.000896630859375;
	accel_bmi088_f.z = user_accel_bmi088.z * 0.000896630859375; //* 0.000896630859375;

	// 1000dps
	gyro_bmi088_f.y = user_gyro_bmi088.x / 32.768;	  //* 0.005326;
	gyro_bmi088_f.x = user_gyro_bmi088.y / 32.768;	  /// 32.768;
	gyro_bmi088_f.z = -(user_gyro_bmi088.z / 32.768); /// 32.768;
													  // sprintf(message1, "加速度x:%.3f \t", accel_bmi088_f.x);
													  // HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);

	// sprintf(message1, "y:%.3f \t",accel_bmi088_f.y);
	// HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);

	// sprintf(message1, "z:%.3f \n",accel_bmi088_f.z);
	// HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);

	// sprintf(message1, "陀螺仪x:%.3f \t",gyro_bmi088_f.x);
	// HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);

	// sprintf(message1, "y:%.3f \t",gyro_bmi088_f.y);
	// HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);

	// sprintf(message1, "z:%.3f \n",gyro_bmi088_f.z);
	// HAL_UART_Transmit(&huart1, message1, strlen(message1), HAL_MAX_DELAY);
}
extern void GetI2C();

extern void PutI2C();
void TaskBMI088(void *pvParameters)
{
	TickType_t start_tick, end_tick;

	for (;;)
	{
		// start_tick = xTaskGetTickCount();
		xSemaphoreTake(xSemaphoreBMI088, portMAX_DELAY);
		// if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
		// {
		GetI2C();
		bmi088_read();
		PutI2C();
		xQueueSend(xQueueACCE, &accel_bmi088_f, portMAX_DELAY);
		xQueueSend(XQueueGYRO, &gyro_bmi088_f, portMAX_DELAY);
		// sprintf(message1, "acc %.2f,%,2f,%.2f\n",accel_bmi088_f.x,accel_bmi088_f.y,accel_bmi088_f.z);
		// HAL_UART_Transmit(&huart1, message1, strlen(message1), 1000);
		// xSemaphoreGive(i2cMutex);
		// }
		// else
		// {
		// sprintf(message1, "阻塞BMI088");
		// HAL_UART_Transmit(&huart1, message1, strlen(message1), 1000);
		// }
		// sprintf(message1, "获取信号量失败bmi");
		// HAL_UART_Transmit(&huart1, message1, strlen(message1), 1000);
		// end_tick = xTaskGetTickCount();
		// TickType_t execution_time = end_tick - start_tick;

		// sprintf(message1, "BMI088 time: %lu ms\n", execution_time * portTICK_PERIOD_MS);
		// HAL_UART_Transmit(&huart1, message1, strlen(message1), 1000);
		xSemaphoreGive(xSemaphoreVL53);

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
