#include "PID.h"
#include "main.h"
uint8_t printmessagepid[512] = {'\0'};
void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->prevVelocity = 0.0f;
	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement,float measure_v,float set_v) {

	/*
	* Error signal
	*/
    float error = measurement - setpoint;
	 
	/*
	* Proportional
	*/
    float proportional = pid->Kp * (error);


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);
	// pid->differentiator = pid->Kd * (measure_v - set_v);

	/*
	* Compute output and apply limits
	*/
    pid->out = proportional - pid->integrator - pid->differentiator;

    // if (pid->out > pid->limMax) {

    //     pid->out = pid->limMax;

    // } else if (pid->out < pid->limMin) {

    //     pid->out = pid->limMin;

    // }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;
	pid->prevVelocity = measure_v;

	/* Return controller output */
    return pid->out;

}

float PIDController_Update_az(PIDController *pid, float setpoint, float measurement, float measure_v,float set_v) {

	/*
	* Error signal
	*/
    float error = measurement - setpoint;
    

	/*
	* Proportional
	*/
    float proportional = pid->Kp * (error);
	// sprintf(printmessagepid,"P = %.2f\n",proportional);
	// HAL_UART_Transmit(&huart1,printmessagepid,strlen(printmessagepid),1000);
	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }
	// sprintf(printmessagepid,"I = %.2f\n",pid->integrator);
	// HAL_UART_Transmit(&huart1,printmessagepid,strlen(printmessagepid),1000);

	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);
		// pid->differentiator = pid->Kd*(measure_v - set_v);

	// sprintf(printmessagepid,"D = %.2f,measure_v = %.2f,preV = %.2f \n",pid->differentiator,measure_v,pid->prevVelocity);
	// HAL_UART_Transmit(&huart1,printmessagepid,strlen(printmessagepid),1000);
	/*
	* Compute output and apply limits
	*/
    pid->out = proportional - pid->integrator - pid->differentiator;
	// sprintf(printmessagepid,"P = %.2f,I = %.2f,D = %.2f ,z = %.2f,zd=%.2f\n",proportional,pid->integrator,pid->differentiator,measurement,setpoint);
	// HAL_UART_Transmit(&huart1,printmessagepid,strlen(printmessagepid),1000);

    // if (pid->out > pid->limMax) {

    //     pid->out = pid->limMax;

    // } else if (pid->out < pid->limMin) {

    //     pid->out = pid->limMin;

    // }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;
	pid->prevVelocity = measure_v;
	/* Return controller output */
    return pid->out;

}