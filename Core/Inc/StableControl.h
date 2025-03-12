#include "FreeRTOS.h"
#include "queue.h"
#include "PID.h"
#include "motor.h"
#include "pmw3901.h"
#include "bmi08x_defs.h"
#include "queueHandle.h"
#include "newton.h"

void get_sensor_data();
void compute_motor_thrust();
void apply_motor_thrust();
void FLIGHTControl(void *pvParameters);
