//
// Created by 28715 on 2026/1/24.
//
#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H
#include "main.h"


typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef struct {
    uint8_t DM4310_ID;
    uint8_t DM4310_ERR;
    float DM4310_POS;
    float DM4310_VEL;
    float DM4310_T;
    float DM4310_T_MOS;
    float DM4310_T_Rotor;
    float DM4310_POS_Last;
    float DM4310_VEL_Last;
    float DM4310_T_Last;
		uint16_t key;
}DM4310_Rx_Data_t;
typedef struct {
    uint8_t DM4310_ID;
    float DM4310_P_des;
    float DM4310_V_des;
    float DM4310_Kp;
    float DM4310_Kd;
    float DM4310_T_ff;
}DM4310_Tx_Data_t;
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


#endif



