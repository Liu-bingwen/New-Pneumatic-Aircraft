#include "newton.h"
#include "main.h"
#include "StableControl.h"
#define C_d 1.0
#define C_psi 5.0
// #define C_z 1.0
// #define C_z_n -1.0
#define TOL 1e-6
#define ALPHA 0.1

#define MAX_ITER 1000
uint8_t printmessage1[512] = {'\0'};
float C_z;
// // 定义函数 f1 和 f2
// double f1(double q1, double q2, double v_z, double r, double a_z)
// {
//     return C_z * (q1 * q2 - pow(q1 - q2, 2)) - C_d * v_z * r - a_z;
// }

// double f2(double q1, double q2, double tau_z)
// {
//     return C_d * (q1 - q2) - tau_z;
// }

// // 定义雅可比矩阵元素
// double df1_dq1(double q1, double q2)
// {
//     return C_z * (q2 - 2 * (q1 - q2));
// }

// double df1_dq2(double q1, double q2)
// {
//     return C_z * (q1 + 2 * (q1 - q2));
// }

// double df2_dq1(double q1, double q2)
// {
//     return C_psi;
// }

// double df2_dq2(double q1, double q2)
// {
//     return -C_psi;
// }

// // Newton - Raphson 迭代法
// void newton_raphson(double *q1, double *q2, double v_z, double r, double az, double tau_z)
// {

//     double delta_q1, delta_q2;
//     double F1, F2;
//     double J11, J12, J21, J22; // 雅可比矩阵元素
//     double det;                // 行列式

//     for (int i = 0; i < MAX_ITER; i++)
//     {
//         // 计算当前的 f1 和 f2
//         F1 = f1(*q1, *q2, v_z, r, az);
//         F2 = f2(*q1, *q2, tau_z);

//         // 计算雅可比矩阵的元素
//         J11 = df1_dq1(*q1, *q2);
//         J12 = df1_dq2(*q1, *q2);
//         J21 = df2_dq1(*q1, *q2);
//         J22 = df2_dq2(*q1, *q2);

//         // 计算行列式
//         det = J11 * J22 - J12 * J21;

//         // 如果行列式为0，无法继续
//         if (fabs(det) < 1e-9)
//         {
//             sprintf(printmessage1, "行列式为零df1_dq1=%.2f,df1_dq2=%.2f\ndf2_dq1=%.2f,df2_dq2=%.2f\n", J11, J12, J21, J22);
//             HAL_UART_Transmit(&huart1, printmessage1, strlen(printmessage1), 1000);
//             // printf("雅可比矩阵不可逆，迭代停止。\n");
//             return;
//         }

//         // 使用雅可比矩阵求解增量 delta_q1 和 delta_q2
//         delta_q1 = (J22 * (-F1) - J12 * (-F2)) / det;
//         delta_q2 = (J11 * (-F2) - J21 * (-F1)) / det;
//         sprintf(printmessage1, "行列式=%.3f,delta_q1=%.3f,delta_q2=%.3f\n", det,delta_q1, delta_q2);
//         HAL_UART_Transmit(&huart1, printmessage1, strlen(printmessage1), 1000);
//         // 更新 q1 和 q2
//         *q1 = *q1 - 0.1*delta_q1;
//         *q2 = *q1 - 0.1*delta_q2;

//         // 检查是否满足容忍度，判断收敛
//         if (fabs(delta_q1) < TOL && fabs(delta_q2) < TOL)
//         {
//             // printf("收敛到解：q1 = %lf, q2 = %lf\n", q1, q2);
//             sprintf(printmessage1, "收敛得到q1 = %.3f,\tq2 = %.3f\n", q1, q2);
//             HAL_UART_Transmit(&huart1, printmessage1, strlen(printmessage1), 1000);

//             return;
//         }
//     }
//     sprintf(printmessage1, "无法收敛delta_q1=%.2f,delta_q2=%.2f\ndf1_dq1=%.2f,df1_dq2=%.2f\ndf2_dq1=%.2f,df2_dq2=%.2f\n", delta_q1, delta_q2, J11, J12, J21, J22);
//     HAL_UART_Transmit(&huart1, printmessage1, strlen(printmessage1), 1000);
//     // 达到最大迭代次数仍未收敛
//     // printf("未在规定的迭代次数内收敛。q1 = %lf, q2 = %lf\n", q1, q2);
// }
void newtonRaphson(double *q1, double *q2, double v_z, double r, double a_z, double tau_z)
{
    int iter = 0;
    double J[2][2], invJ[2][2];
    double f[2], delta_q[2];
    double prev_q1 = *q1;
    double prev_q2 = *q2;
    if(a_z < 0){
        C_z = -1.0;
    }
    else if (a_z >=0 )
    {
        C_z = 1.0;
    }
    

    while (iter < MAX_ITER)
    {
        // Evaluate the functions at the current guess
        f[0] = f1(*q1, *q2, v_z, r, a_z);
        f[1] = f2(*q1, *q2, tau_z);

        // Check for convergence
        if (fabs(f[0]) < TOL && fabs(f[1]) < TOL)
        {
            break;
        }

        // Compute the Jacobian matrix and its inverse
        jacobian(*q1, *q2, J);
        inverseJacobian(J, invJ);

        // Newton-Raphson update: delta_q = -inv(J) * f
        delta_q[0] = -(invJ[0][0] * f[0] + invJ[0][1] * f[1]);
        delta_q[1] = -(invJ[1][0] * f[0] + invJ[1][1] * f[1]);

        // Update q1 and q2
        *q1 += delta_q[0];
        *q2 += delta_q[1];

        iter++;
    }
    if (iter < MAX_ITER)
    {
        prev_q1 = *q1;
        prev_q2 = *q2;
    }
    if (iter == MAX_ITER)
    {
        
        // sprintf(printmessage1, "无法收敛q1=%.2f,q2=%.2f\n", *q1, *q2);

        // HAL_UART_Transmit(&huart1, printmessage1, strlen(printmessage1), 1000);
        *q1 = prev_q1;
        *q2 = prev_q2;
    }
}

// Define the first nonlinear function f1
double f1(double q1, double q2, double v_z, double r, double a_z)
{
    return C_z * (q1 * q2 - pow(q1 - q2, 2)) - C_d * v_z * r - a_z;
}

// Define the second nonlinear function f2
double f2(double q1, double q2, double tau_z)
{
    return C_psi * (q1 - q2) - tau_z;
}

// Define the Jacobian matrix
void jacobian(double q1, double q2, double J[2][2])
{
    J[0][0] = C_z * (q2 - 2 * (q1 - q2)); // df1/dq1
    J[0][1] = C_z * (q1 - 2 * (q2 - q1)); // df1/dq2
    J[1][0] = C_psi;                      // df2/dq1
    J[1][1] = -C_psi;                     // df2/dq2
}

// Inverse of the 2x2 Jacobian matrix
void inverseJacobian(double J[2][2], double invJ[2][2])
{
    double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (fabs(det) < 1e-10)
    {
        // sprintf(printmessage1, "Jacobin is Singular\n");
        // HAL_UART_Transmit(&huart1, printmessage1, strlen(printmessage1), 1000);
        return;
    }

    invJ[0][0] = J[1][1] / det;
    invJ[0][1] = -J[0][1] / det;
    invJ[1][0] = -J[1][0] / det;
    invJ[1][1] = J[0][0] / det;
}
