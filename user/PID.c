//
// Created by 28715 on 2026/1/24.
//
#include "main.h"
#include "PID.h"
#include "struct_typedef.h"

#include <stdint.h>
#include "PID.h"
void PIDInit( PID_Data *pid,float kp,float ki,float kd) {
    pid->kp=kp;
    pid->ki=ki;
    pid->kd=kd;
    pid->goal=0.0f;
    pid->error_sum=0.0f;
    pid->last_error=0.0f;
}
void PID_Changegoal( PID_Data *pid,float goal) {
    pid->goal=goal;
}
float PIDCompute( PID_Data *pid,float current) {
    pid->current=current;
//    pid->error=pid->current-pid->goal;
	  pid->error=pid->goal-pid->current;
	if(pid->error>1){pid->error_sum=0;}
	else{
    pid->error_sum=pid->error+pid->error_sum;
		if(pid->error_sum>4.5) pid->error_sum=4.5;
		if(pid->error_sum<-4.5) pid->error_sum=-4.5;
	}
    pid->kp_output=pid->kp*pid->error;
    pid->ki_output=pid->ki*pid->error_sum;
    pid->kd_output=pid->kd*(pid->error-pid->last_error);
    pid->last_error=pid->error;
    float output=pid->kp_output+pid->ki_output+pid->kd_output;
    return output;
}
