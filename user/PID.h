//
// Created by 28715 on 2026/1/24.
//

#ifndef PID_H
#define PID_H
#include "main.h"
#include "struct_typedef.h"

extern PID_Data pid_data_v;
extern PID_Data pid_data_p;
void PIDInit( PID_Data *pid,float kp,float ki,float kd);
void PID_Changegoal( PID_Data *pid,float goal);
float PIDCompute( PID_Data *pid,float current);
#endif //DM4310_PID_H

