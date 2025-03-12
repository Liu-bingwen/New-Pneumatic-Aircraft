#include "StableControl.h"
#include "math.h"
#include "queueHandle.h"
#define alpha 0.1
/* Controller parameters */
#define PID_KP 1.2f
#define PID_KI 1.2f
#define PID_KD 0.075f

float pid_kp = 0.1;
float pid_ki = 0.1;
float pid_kd = 0.1;


#define PID_TAU 0.3f

#define PID_LIM_MIN -1.0f
#define PID_LIM_MAX 1.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT 5.0f

#define SAMPLE_TIME_S 0.987f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f
uint8_t printmessage[512] = {'\0'};
PIDController pid_az;
PIDController pid_tz;
PIDController pid_ax;
PIDController pid_ay;
int uart_busy = 0;
// PID初始化
// PIDController pid_az = {pid_kp, 0.01f, 1.0f,
//                         PID_TAU,
//                         PID_LIM_MIN, PID_LIM_MAX,
//                         PID_LIM_MIN_INT, PID_LIM_MAX_INT,
//                         SAMPLE_TIME_S};
// PIDController pid_tz = {0.02f, 0.0f, 0.01f,
//                         PID_TAU,
//                         PID_LIM_MIN, PID_LIM_MAX,
//                         PID_LIM_MIN_INT, PID_LIM_MAX_INT,
//                         SAMPLE_TIME_S};
// PIDController pid_ax = {-0.015f, 0.001f, 0.001f,
//                         PID_TAU,
//                         PID_LIM_MIN, PID_LIM_MAX,
//                         PID_LIM_MIN_INT, PID_LIM_MAX_INT,
//                         SAMPLE_TIME_S};
// PIDController pid_ay = {-0.015f, 0.001f, 0.001f,
//                         PID_TAU,
//                         PID_LIM_MIN, PID_LIM_MAX,
//                         PID_LIM_MIN_INT, PID_LIM_MAX_INT,
//                         SAMPLE_TIME_S};
// 用户参数
float yaw_angle = 0.1; // ψ_z
float previous_yaw_angle = 0.0;
float dt = 0.987;
float lastValue = 0;
uint8_t rx_buffer[12];
uint8_t rx_data;
float bias = 0;
int calibration_samples = 1000;
int index = 0;
QueueHandle_t commandQueue; // 接收命令
// 目标参数
int z_d = 2;
float yaw_d = 0.0;
float x_d = 0;
float y_d = 0;
float z_d_dot = 0.0;
float x_d_dot = 0.0;
float y_d_dot = 0.0;
float yaw_d_dot = 0.0;
float az = 0, tz = 0, ax = 0, ay = 0;
volatile pmw3901 target_pmw3901; // PMW3901
double q1 = 10.0;
double q2 = 10.0;

// 当前姿态
float velocity_x = 0.0f;
float velocity_y = 0.0f;
float velocity_z = 0.0f;
int ac_z;
float ac_z_f;
float ac_yaw;
float ac_x = 0.0;
float ac_y = 0.0;

float previous_v_z = 0;
struct bmi08x_sensor_data_f ac_accel;
struct bmi08x_sensor_data_f ac_gyro;
pmw3901 ac_pmw3901;
float x, y;
float a_z, a_x, a_y;
float w_z;
float tau_z = 0.0;
// double q1, q2;
double d, A, B, C;
// 电机推力
float motor_thrust[4]; // M1, M2, M3, M4的推力

void PID_upgrade(PIDController *pid,float kp,float ki,float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void PID_init(PIDController *pid,float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->tau = PID_TAU;
    pid->limMin = PID_LIM_MIN;
    pid->limMax = PID_LIM_MAX;
    pid->limMinInt = PID_LIM_MIN_INT;
    pid->limMaxInt = PID_LIM_MAX_INT;
    pid->T = SAMPLE_TIME_S;
}

// 初始化PID控制器
void init_pid_controllers()
{
    PID_init(&pid_ax,pid_kp,pid_ki,pid_kd);
    PID_init(&pid_ay,pid_kp,pid_ki,pid_kd);
    PID_init(&pid_az,pid_kp,pid_ki,pid_kd);
    PID_init(&pid_tz,pid_kp,pid_ki,pid_kd);

    PIDController_Init(&pid_ax);
    PIDController_Init(&pid_ay);
    PIDController_Init(&pid_az);
    PIDController_Init(&pid_tz);
}
float kalmanFilter(float measurement)
{
    static float q = 0.1; // 过程噪声
    static float r = 0.1; // 测量噪声
    static float x = 0;   // 估计值
    static float p = 1;   // 误差协方差
    static float k;       // 卡尔曼增益

    // 预测更新
    p = p + q;

    // 测量更新
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;

    return x;
}
void get_sensor_data()
{

    yaw_angle += ac_gyro.z * dt;
    yaw_angle = fmod(yaw_angle, 360.0f);
    if (yaw_angle > 180.0f)
    {
        yaw_angle -= 360.0f;
    }
    else if (yaw_angle < -180.0f)
    {
        yaw_angle += 360.0f;
    }

    // w_z = ac_gyro.z;                                                                             // 偏航角速度 度
    // yaw_angle = fmod(previous_yaw_angle + w_z * dt, 360 /*previous_yaw_angle + w_z * dt, 360*/); // 偏航角    度
    // previous_yaw_angle = yaw_angle;
    // sprintf(printmessage, "Wz = %.2f, yaw increment = %.2f, ψ = %.2f\n", w_z,w_z * dt,yaw_angle);
    // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);

    // bias += (ac_accel.z - 9.7936);
    // index++;
    // if(index == 500){
    //     while (1)
    //     {
    //         sprintf(printmessage,"bias = %.2f",bias/500);
    //         HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);
    //         vTaskDelay(pdMS_TO_TICKS(100));
    //     }

    // }
    // v_z = previous_v_z + acce.z * dt;
    // previous_v_z = v_z;
    velocity_x += ac_accel.x * dt;
    velocity_y += ac_accel.y * dt;
    velocity_z += ((ac_accel.z + 9.7936 - 0.1) * dt);

    /* code */
    previous_v_z = ac_z_f;
    ac_z_f = ac_z / 1000.0f;

    ac_x += ac_pmw3901.deltaX * 1.0;
    ac_y += -ac_pmw3901.deltaY * 1.0;
    // sprintf(printmessage, "x1 = %.2f\ty1 = %.2f\n",ac_x,ac_y);
    // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);
}

float exponentialSmoothing(float newValue)
{
    lastValue = alpha * newValue + (1 - alpha) * lastValue;
    return lastValue;
}

void compute_motor_thrust()
{

    az = PIDController_Update_az(&pid_az, z_d, ac_z_f, velocity_z, z_d_dot); // PID得到az
    tau_z = PIDController_Update(&pid_tz, yaw_d, yaw_angle, w_z, yaw_d_dot); // PID得到wz
    // sprintf(printmessage, "ac_z=%.2f,previous_vz=%.2f,v_z=%.2f\n", ac_z_f,previous_v_z,(ac_z_f - previous_v_z)/dt);
    // sprintf(printmessage, "z_d=%.1f,ac_z=%.1f,v_z=%.1f,az_d=%.1f,z_d_dot=%.1f,az=%.1f\n", z_d,ac_z_f,velocity_z,ac_accel.z /*+ 9.7936 - 0.1*/,z_d_dot,az);
    // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);

    newtonRaphson(&q1, &q2, velocity_z, 0.4, az, tau_z); // 牛顿拉弗森反演

    ax = PIDController_Update(&pid_ax, x_d, ac_x, velocity_x, x_d_dot);
    ay = PIDController_Update(&pid_ay, y_d, ac_y, velocity_y, y_d_dot);
    // sprintf(printmessage,"deltaX = %.2f, deltaY = %.2f",ac_x,ac_y);
    // sprintf(printmessage, "g = %.3f,q1 = %.2f, q2 = %.3f\nax = %.3f, ay = %.3f\n", ac_accel.z,q1, q2,ax,ay);
    // sprintf(printmessage, "最终的q1 = %.2f,q2 = %.2f,height=%.2f\naz=%.2f\ttau_z=%.2f,ax=%.2f,ay=%.2f,v_z=%.2f\n", q1, q2, ac_z_f, az, tau_z, ax, ay, velocity_z);

    // if (ac_x != 0 || ac_y != 0)
    // {
    // sprintf(printmessage, "yaw_angle=%.2f,w_z=%.2f\n", yaw_angle, ac_gyro.z);
    // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);
    // }

    motor_thrust[0] = (q1 - ax + ay) / 2;
    motor_thrust[1] = (q2 - ax - ay) / 2;
    motor_thrust[2] = (q1 + ax + ay) / 2;
    motor_thrust[3] = (q2 + ax - ay) / 2;

    // for (int i = 0; i < 4; i++)
    // {
    //     motor_thrust[i] = exponentialSmoothing(motor_thrust[i]);
    // }
    for (int i = 0; i < 4; i++)
    {
        motor_thrust[i] = motor_thrust[i] / (0.1 * sin(45.0));
        motor_thrust[i] = motor_thrust[i] / 0.262;
    }

    // motor_thrust[0] = motor_thrust[2];
    // motor_thrust[1] = motor_thrust[3];

    for (int i = 0; i < 4; i++)
    {
        if (motor_thrust[i] > 100)
            motor_thrust[i] = 100;

        if (motor_thrust[i] < 0)
        {
            motor_thrust[i] = 0;
        }
    }

    
}
void apply_motor_thrust()
{
    MotorA_SetSpeed(fabs(motor_thrust[1]));
    MotorD_SetSpeed(fabs(motor_thrust[2]));
    MotorC_SetSpeed(fabs(motor_thrust[0]));
    MotorB_SetSpeed(fabs(motor_thrust[3]));

    // sprintf(printmessage, "x1 = %.2f\ty1 = %.2f\nax = %.2f\tay = %.2f\n", ac_x, ac_y,ax,ay);
    // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);
    // sprintf(printmessage,"A: %.2f\tB:%.2f\nC: %.2f\tD: %.2f\n",motor_thrust[1],motor_thrust[0],motor_thrust[3],motor_thrust[2]);
    // sprintf(printmessage, "q1 = %.2f\tq2 = %.2f\naz = %.2f\tψz = %.2f\nax = %.2f\tay = %.2f,yaw = %.2f\n", q1,q2, az,tau_z, ax, ay,yaw_angle);//

    // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendToBackFromISR(commandQueue, rx_buffer, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        

        // 再次启动接收中断
        HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));
    }
}



void FLIGHTControl(void *pvParameters)
{
    TickType_t start_tick, end_tick;
    TickType_t flight_task_start_time;
    TickType_t task_dt;
    char str1[20];
    char str2[20];
    float distance;
    float command[3];
    commandQueue = xQueueCreate(10, 12);
    if (commandQueue == NULL)
    {
        sprintf(printmessage, "commandQueue 初始化失败\n");
        HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);
        while (1);
    }
    HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));
    while (1)
    {
        
        // start_tick = xTaskGetTickCount();
        xSemaphoreTake(xSemaphoreControl, portMAX_DELAY);
        if (xQueueReceive(commandQueue, command, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            PID_upgrade(&pid_ax, command[0], command[1], command[2]);
            PID_upgrade(&pid_ay, command[0], command[1], command[2]);
            PID_upgrade(&pid_az, command[0], command[1], command[2]);
            PID_upgrade(&pid_tz, command[0], command[1], command[2]);
            
            // memcpy(&pid_kp, command, sizeof(float));
            // memcpy(&pid_ki, command + sizeof(float), sizeof(float));
            // memcpy(&pid_kd, command + 2 * sizeof(float), sizeof(float));
            // PID_upgrade(&pid_ax,pid_kp,pid_ki,pid_kd);
            // PID_upgrade(&pid_ay,pid_kp,pid_ki,pid_kd);
            // PID_upgrade(&pid_az,pid_kp,pid_ki,pid_kd);
            // PID_upgrade(&pid_tz,pid_kp,pid_ki,pid_kd);

        }
        
        BaseType_t acce_ready, pmw_ready, vl53_ready, gyro_ready;
        pmw_ready = xQueueReceive(xQueuePMW3901, &ac_pmw3901, pdMS_TO_TICKS(10));
        // sprintf(printmessage, "pmw %d,%d\n", ac_pmw3901.deltaX, ac_pmw3901.deltaY);
        // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);

        acce_ready = xQueueReceive(xQueueACCE, &ac_accel, pdMS_TO_TICKS(10));
        // sprintf(printmessage, "accel %.2f,%.2f,%.2f\n", ac_accel.x, ac_accel.y,ac_accel.z);
        // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);

        gyro_ready = xQueueReceive(XQueueGYRO, &ac_gyro, pdMS_TO_TICKS(10));
        // sprintf(printmessage, "gyro %.2f,%.2f,%.2f\n", ac_gyro.x,ac_gyro.y,ac_gyro.z);
        // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);

        vl53_ready = xQueueReceive(xQueueVL53, &ac_z, pdMS_TO_TICKS(10));

        sprintf(printmessage,"%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",ac_pmw3901.deltaX,ac_pmw3901.deltaY,ac_z,ac_accel.x * dt,ac_accel.y * dt,((ac_accel.z + 9.7936 - 0.1) * dt),ac_gyro.z,ac_gyro.z * dt,pid_ax.Kp,pid_ax.Ki,pid_ax.Kd);

        // sprintf(printmessage, "vl53 %d\n", ac_z);
        HAL_UART_Transmit_DMA(&huart1, printmessage, strlen(printmessage));

        if (acce_ready == pdPASS && pmw_ready == pdPASS && vl53_ready == pdPASS && gyro_ready == pdPASS)
        {

            get_sensor_data();

            compute_motor_thrust();

            apply_motor_thrust();
            // end_tick = xTaskGetTickCount();
            // TickType_t execution_time = end_tick - start_tick;

            // flight_task_start_time = xTaskGetTickCount();
            // if (flight_task_start_time >= bmi088_task_end_time)
            // {
            //     task_dt = flight_task_start_time - bmi088_task_end_time;
            // }
            // else
            // {
            //     task_dt = (UINT32_MAX - bmi088_task_end_time) + flight_task_start_time;
            // }
            // dt = task_dt * portTICK_PERIOD_MS / 1000.0f; // 转换为秒

            // sprintf(printmessage, "start: %lu ms,end: %lu ms\n", bmi088_task_end_time*portTICK_PERIOD_MS,flight_task_start_time*portTICK_PERIOD_MS);
            // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);

            // sprintf(printmessage, "Control time: %lu ms\n", execution_time * portTICK_PERIOD_MS);
            // HAL_UART_Transmit(&huart1, printmessage, strlen(printmessage), 1000);

            xSemaphoreGive(xSemaphorePMW3901);

            vTaskDelay(pdMS_TO_TICKS(10));
        }
        else
        {
            xSemaphoreGive(xSemaphorePMW3901);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
