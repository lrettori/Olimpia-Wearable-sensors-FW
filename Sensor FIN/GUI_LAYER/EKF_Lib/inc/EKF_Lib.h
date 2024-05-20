#ifndef __EKF_LIB_H
#define __EKF_LIB_H

#define MAX_C 3
#define T     0.02; /*50Hz*/
double det(double m[MAX_C][MAX_C], int car);
void coff_inv(double m[MAX_C][MAX_C], int car);
void Step1(double q_plus[4]);
void Step2(void);
void Step3(double H[3][4],double R[3][3]);
void Step4(float in_sensor[3],double h[3],double q_plus[4]);
void Step5(double H[3][4]);
void EKF(float in_acc[3], float in_gyr[3], double q_plus[4]);

#endif