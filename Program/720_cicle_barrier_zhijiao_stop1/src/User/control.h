#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "include.h"

#define FABS(x) x>0?x:-x

//AD��ֵ����
#define AD1MAX 3500
#define AD2MAX 3500
#define AD3MAX 3500
#define AD4MAX 3500
#define AD5MAX 3500
#define AD6MAX 3500
//AD��ֵ����
#define AD1MIN 0
#define AD2MIN 0
#define AD3MIN 0
#define AD4MIN 0
#define AD5MIN 0
#define AD6MIN 0
//AD��һ����
#define AD1DIFF AD1MAX-AD1MIN
#define AD2DIFF AD2MAX-AD2MIN
#define AD3DIFF AD3MAX-AD3MIN
#define AD4DIFF AD4MAX-AD4MIN
#define AD5DIFF AD5MAX-AD5MIN
#define AD6DIFF AD6MAX-AD6MIN

//����ٶ�����
#define MOTOR_MAX 2900

//���λ��������
#define STEER_MAX 3700//�ұ�
#define STEER_MIN 2500//���

#define SET_ANGLE 3030

void motor_out();
void steer_out();

void motor_cal();
void steer_cal();

void adval_get();
void speed_get();

void sr04_use();
void get_slope();

void barrier_sr04();
#endif
