//
// Created by 28715 on 2026/1/24.
//

#ifndef DM4310_CAN_H
#define DM4310_CAN_H
#include "main.h"
#include "can.h"
#include "struct_typedef.h"
#include <stdint.h>
#include "stm32f4xx_hal_can.h"
#define DM4310_CANID  0x01
//接收数据
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
}DM4310_Rx_Data_t;
typedef struct {
    uint8_t DM4310_ID;
    float DM4310_P_des;
    float DM4310_V_des;
    float DM4310_Kp;
    float DM4310_Kd;
    float DM4310_T_ff;
}DM4310_Tx_Data_t;

extern DM4310_Rx_Data_t DM4310_Rx_Data ;
extern DM4310_Tx_Data_t DM4310_Tx_Data ;

#define P_MIN -12.5f  // 位置最小值 (rad)
#define P_MAX 12.5f   // 位置最大值 (rad)
#define V_MIN -30.0f  // 速度最小值 (rad/s)
#define V_MAX 30.0f   // 速度最大值 (rad/s)
#define T_MIN -10.0f  // 扭矩最小值 (N.m)
#define T_MAX 10.0f   // 扭矩最大值 (N.m)
#define KP_MIN 0.0f   // Kp 最小值
#define KP_MAX 500.0f // Kp 最大值
#define KD_MIN 0.0f   // Kd 最小值
#define KD_MAX 5.0f   // Kd 最大值


// 辅助函数：将 uint 转换回 float
// x_int: 接收到的整数, x_min/x_max: 物理量范围, bits: 数据位数
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float f_constrain(float val, float min, float max);
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);



void DM4310_Get_Data(DM4310_Rx_Data_t *DM4310_Rx_Data,uint8_t *Rx_Data);
void DM4310_Tx_Data_Init(
    DM4310_Tx_Data_t *DM4310_Tx_Data,
    float DM4310_P_des,
    float DM4310_V_des,
    float DM4310_Kp,
    float DM4310_Kd,
    float DM4310_T_ff);
void DM4310_Control(CAN_HandleTypeDef *hcan,DM4310_Tx_Data_t *DM4310_Tx_Data);



extern CAN_HandleTypeDef hcan1;
extern uint8_t Rx_Data[8];
extern uint8_t Tx_Data[8];

#endif //DM4310_CAN_H
		