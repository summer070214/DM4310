//
// Created by 28715 on 2026/1/24.
//
#include "main.h"
#include "CAN.h"
#include "PID.h"
#include "can.h"
#include "struct_typedef.h"
#include <stdint.h>
#define DM4310_CANID 0x01
#define DM4310_MasterID 0x11
#define P_MIN -12.5f  // 位置最小值 (rad)
#define P_MAX 12.5f   // 位置最大值 (rad)
#define V_MIN -45.0f  // 速度最小值 (rad/s)
#define V_MAX 45.0f   // 速度最大值 (rad/s)
#define T_MIN -18.0f  // 扭矩最小值 (N.m)
#define T_MAX 18.0f   // 扭矩最大值 (N.m)

// 辅助函数：将 uint 转换回 float
// x_int: 接收到的整数, x_min/x_max: 物理量范围, bits: 数据位数
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int * span / ((float)((1 << bits) - 1))) + offset;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (rx_header.StdId == DM4310_MasterID) {
            DM4310_Get_Data(&DM4310_Rx_Data,rx_data);
						DM4310_Rx_Data.key++;
        }

    }
}


void DM4310_Get_Data(DM4310_Rx_Data_t *DM4310_Rx_Data,uint8_t *rx_data) {
    DM4310_Rx_Data->DM4310_POS_Last=DM4310_Rx_Data->DM4310_POS;
    DM4310_Rx_Data->DM4310_VEL_Last=DM4310_Rx_Data->DM4310_VEL;
    DM4310_Rx_Data->DM4310_T_Last=DM4310_Rx_Data->DM4310_T;
    DM4310_Rx_Data->DM4310_ID = rx_data[0] & 0x0F;
    DM4310_Rx_Data->DM4310_ERR=(rx_data[0] >> 4) & 0x0F;
    uint16_t  pos=(uint16_t)(rx_data[1] << 8) | rx_data[2];
//    DM4310_Rx_Data->DM4310_POS= f_constrain(uint_to_float(pos, P_MIN, P_MAX, 16), P_MIN,  P_MAX);
    DM4310_Rx_Data->DM4310_POS=uint_to_float(pos, P_MIN, P_MAX, 16);
    uint16_t  vel=(uint16_t)(rx_data[3] << 4) | (rx_data[4]>>4);
//    DM4310_Rx_Data->DM4310_VEL= f_constrain( uint_to_float(vel, V_MIN, V_MAX, 12),V_MIN, V_MAX);
	  DM4310_Rx_Data->DM4310_VEL=  uint_to_float(vel, V_MIN, V_MAX, 12);
    uint16_t  t=(uint16_t)((rx_data[4] & 0x0F) << 8) | rx_data[5];
//    DM4310_Rx_Data->DM4310_T=f_constrain(uint_to_float(t, T_MIN, T_MAX, 12),T_MIN, T_MAX);
    DM4310_Rx_Data->DM4310_T=uint_to_float(t, T_MIN, T_MAX, 12);
    DM4310_Rx_Data->DM4310_T_MOS= rx_data[6];
    DM4310_Rx_Data->DM4310_T_Rotor= rx_data[7];
}


// 限制范围函数
float f_constrain(float val, float min, float max)
{
    if (val < min) return min;
//			if (val < min) DM4310_DisEnable(&hcan1);
    if (val > max) return max;
//	    if (val > max) DM4310_DisEnable(&hcan1);

    return val;
}

// 浮点转整型映射函数
uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    // 限制输入范围，防止溢出
    x = f_constrain(x, x_min, x_max);

    // 映射计算
    return (uint16_t) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}



void DM4310_Tx_Data_Init(
    DM4310_Tx_Data_t *DM4310_Tx_Data,
    float _P_des,
    float _V_des,
    float _Kp,
    float _Kd,
    float _T_ff) {
	DM4310_Tx_Data->DM4310_ID=DM4310_CANID;
    DM4310_Tx_Data->DM4310_P_des=_P_des;
    DM4310_Tx_Data->DM4310_V_des=_V_des;
    DM4310_Tx_Data->DM4310_Kp=_Kp;
    DM4310_Tx_Data->DM4310_Kd=_Kd;
    DM4310_Tx_Data->DM4310_T_ff=_T_ff;
}

void DM4310_Enable(CAN_HandleTypeDef *hcan) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    TxHeader.StdId = DM4310_CANID; 
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;      

		TxData[0] = 0xFF;       
    TxData[1] = 0xFF;       
    TxData[2] = 0xFF;      
    TxData[3] = 0xFF;       
    TxData[4] = 0xFF;       
    TxData[5] = 0xFF;       
    TxData[6] = 0xFF;       
    TxData[7] = 0xFC;      

    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);


}
void DM4310_DisEnable(CAN_HandleTypeDef *hcan){
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    TxHeader.StdId = DM4310_CANID; 
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;      

		TxData[0] = 0xFF;       
    TxData[1] = 0xFF;       
    TxData[2] = 0xFF;      
    TxData[3] = 0xFF;       
    TxData[4] = 0xFF;       
    TxData[5] = 0xFF;       
    TxData[6] = 0xFF;       
    TxData[7] = 0xFD;      

    HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);


}


void DM4310_Control(CAN_HandleTypeDef *hcan,DM4310_Tx_Data_t *DM4310_Tx_Data)
{
//	  DM4310_Enable(&hcan1); // 使能 ID 为 0x01 的电机

    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
//    uint16_t p_int = float_to_uint(DM4310_Tx_Data->DM4310_P_des, P_MIN,  P_MAX,  16); // 16位
//    uint16_t v_int = float_to_uint(DM4310_Tx_Data->DM4310_V_des, V_MIN,  V_MAX,  12); // 12位
//    uint16_t kp_int = float_to_uint(DM4310_Tx_Data->DM4310_Kp,   KP_MIN, KP_MAX, 12); // 12位
//    uint16_t kd_int = float_to_uint(DM4310_Tx_Data->DM4310_Kd,   KD_MIN, KD_MAX, 12); // 12位
	  uint16_t p_int = 0;
    uint16_t v_int = 0;
    uint16_t kp_int =0;
    uint16_t kd_int =0;
//    uint16_t t_int = float_to_uint(f_constrain(DM4310_Tx_Data->DM4310_T_ff, T_MIN,  T_MAX),  T_MIN,  T_MAX,  12); // 12位
    uint16_t t_int = float_to_uint(DM4310_Tx_Data->DM4310_T_ff,  T_MIN,  T_MAX,  12); // 12位
    // D[0], D[1]: 位置 (16位)
    tx_data[0] = p_int >> 8;        // 高8位
    tx_data[1] = p_int & 0xFF;      // 低8位

    // D[2]: 速度高8位
    tx_data[2] = v_int >> 4;

    // D[3]: 速度低4位 | Kp高4位 (拼接点!)
    // v_int&0xF 取速度的低4位，左移4位放到字节的高位
    // kp_int>>8 取Kp的高4位，放到字节的低位
    tx_data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);

    // D[4]: Kp低8位
    tx_data[4] = kp_int & 0xFF;

    // D[5]: Kd高8位
    tx_data[5] = kd_int >> 4;

    // D[6]: Kd低4位 | 扭矩高4位 (拼接点!)
    tx_data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);

    // D[7]: 扭矩低8位
    tx_data[7] = t_int & 0xFF;
    // 3. 配置 CAN 发送头
    tx_header.StdId = DM4310_CANID; 		// 电机 ID (如 0x01)
//		tx_header.ExtId = 0x00; 
    tx_header.IDE = CAN_ID_STD;        // 标准帧
    tx_header.RTR = CAN_RTR_DATA;      // 数据帧
    tx_header.DLC = 8;                 // 数据长度 8 字节

    // 4. 发送
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;                // 过滤器编号，可选 0-27
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32位宽度
    sFilterConfig.FilterIdHigh = 0x0000;         // ID高16位
    sFilterConfig.FilterIdLow = 0x0000;          // ID低16位
    sFilterConfig.FilterMaskIdHigh = 0x0000;     // 掩码高16位（全0表示不屏蔽任何位，即全部接收）
    sFilterConfig.FilterMaskIdLow = 0x0000;      // 掩码低16位
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; // 匹配到的报文放入 FIFO 0
    sFilterConfig.FilterActivation = ENABLE;     // 激活过滤器
    sFilterConfig.SlaveStartFilterBank = 14;     // CAN1/CAN2 过滤器分配（F4通常0-13给CAN1）

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];


