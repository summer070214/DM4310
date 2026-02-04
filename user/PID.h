//
// Created by 28715 on 2026/1/24.
//

#ifndef PID_H
#define PID_H
#include "main.h"
typedef struct{
    float kp;            //比例系数
    float ki;            //积分系数
    float kd;            //微分系数
    float goal;          //目标值
    float current;       //当前值
    float error;         //误差
    float last_error;    //上次误差
    float error_sum;      //误差和
    float ki_output;     //积分值
    float kd_output;
    float kp_output;   //微分值
}PID_Data;
extern PID_Data pid_data;
void PIDInit( PID_Data *pid,float kp,float ki,float kd);
void PID_Changegoal( PID_Data *pid,float goal);
float PIDCompute( PID_Data *pid,float current);
#endif //DM4310_PID_H

