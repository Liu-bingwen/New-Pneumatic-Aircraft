#include "math.h"
#include "stdio.h"

//初始猜测值


// void newton_raphson(double *q1_init, double *q2_init, double v_z, double r, double a_z, double tau_z);
// double f1(double q1, double q2, double v_z, double r, double a_z);
// double f2(double q1, double q2, double tau_z);

// double df1_dq1(double q1, double q2);

// double df1_dq2(double q1, double q2);

// double df2_dq1(double q1, double q2);

// double df2_dq2(double q1, double q2);
void newtonRaphson(double *q1, double *q2,double v_z,double r,double a_z,double tau_z);

double f1(double q1, double q2,double v_z,double r,double a_z);

double f2(double q1, double q2,double tau_z);

void jacobian(double q1, double q2, double J[2][2]);

void inverseJacobian(double J[2][2], double invJ[2][2]);
