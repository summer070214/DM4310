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
    uint8_t rx_data[8];
    //DM4310_Rx_Data_t *DM4310_Rx_Data={0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (rx_header.StdId == DM4310_CANID) {
             DM4310_Get_Data(&DM4310_Rx_Data,rx_data);
            DM4310_Tx_Data.DM4310_T_ff=PIDCompute(&pid_data,DM4310_Rx_Data.DM4310_T);
            DM4310_Control(&hcan1,&DM4310_Tx_Data);
        }

    }
}
void DM4310_Get_Data(DM4310_Rx_Data_t *DM4310_Rx_Data,uint8_t *rx_data) {
    DM4310_Rx_Data->DM4310_POS_Last=DM4310_Rx_Data->DM4310_POS;
    DM4310_Rx_Data->DM4310_VEL_Last=DM4310_Rx_Data->DM4310_VEL;
    DM4310_Rx_Data->DM4310_T_Last=DM4310_Rx_Data->DM4310_T;
    DM4310_Rx_Data->DM4310_ID = rx_data[0] & 0x0F;
    DM4310_Rx_Data->DM4310_ERR=(rx_data[0] >> 4) & 0x0F;
    uint8_t  pos=(rx_data[1] << 8) | rx_data[2];
    DM4310_Rx_Data->DM4310_POS= uint_to_float(pos, P_MIN, P_MAX, 16);
    uint8_t  vel=(rx_data[3] << 4) | (rx_data[4]>>4);
    DM4310_Rx_Data->DM4310_VEL=  uint_to_float(vel, V_MIN, V_MAX, 12);
    uint8_t  t=((rx_data[4] & 0x0F) << 8) | rx_data[5];
    DM4310_Rx_Data->DM4310_T=uint_to_float(t, T_MIN, T_MAX, 12);
    DM4310_Rx_Data->DM4310_T_MOS= rx_data[6];
    DM4310_Rx_Data->DM4310_T_Rotor= rx_data[7];
}


// 限制范围函数
float f_constrain(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
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
    float DM4310_P_des,
    float DM4310_V_des,
    float DM4310_Kp,
    float DM4310_Kd,
    float DM4310_T_ff) {
    DM4310_Tx_Data->DM4310_P_des=DM4310_P_des;
    DM4310_Tx_Data->DM4310_V_des=DM4310_V_des;
    DM4310_Tx_Data->DM4310_Kp=DM4310_Kp;
    DM4310_Tx_Data->DM4310_Kd=DM4310_Kd;
    DM4310_Tx_Data->DM4310_T_ff=DM4310_T_ff;
}
/*void DM4310_Control(CAN_HandleTypeDef *hcan,
    float DM4310_P_des,
    float DM4310_V_des,
    float DM4310_Kp,
    float DM4310_Kd,
    float DM4310_T_ff
) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    uint16_t p_int = float_to_uint(DM4310_P_des, P_MIN,  P_MAX,  16); // 16位
    uint16_t v_int = float_to_uint(DM4310_V_des, V_MIN,  V_MAX,  12); // 12位
    uint16_t kp_int = float_to_uint(DM4310_Kp,   KP_MIN, KP_MAX, 12); // 12位
    uint16_t kd_int = float_to_uint(DM4310_Kd,   KD_MIN, KD_MAX, 12); // 12位
    uint16_t t_int = float_to_uint(DM4310_T_ff,  T_MIN,  T_MAX,  12); // 12位
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
    tx_header.StdId = DM4310_CANID;        // 电机 ID (如 0x01)
    tx_header.IDE = CAN_ID_STD;        // 标准帧
    tx_header.RTR = CAN_RTR_DATA;      // 数据帧
    tx_header.DLC = 8;                 // 数据长度 8 字节

    // 4. 发送
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}*/
void DM4310_Control(CAN_HandleTypeDef *hcan,DM4310_Tx_Data_t *DM4310_Tx_Data)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    uint16_t p_int = float_to_uint(DM4310_Tx_Data->DM4310_P_des, P_MIN,  P_MAX,  16); // 16位
    uint16_t v_int = float_to_uint(DM4310_Tx_Data->DM4310_V_des, V_MIN,  V_MAX,  12); // 12位
    uint16_t kp_int = float_to_uint(DM4310_Tx_Data->DM4310_Kp,   KP_MIN, KP_MAX, 12); // 12位
    uint16_t kd_int = float_to_uint(DM4310_Tx_Data->DM4310_Kd,   KD_MIN, KD_MAX, 12); // 12位
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
    tx_header.StdId = DM4310_CANID;        // 电机 ID (如 0x01)
    tx_header.IDE = CAN_ID_STD;        // 标准帧
    tx_header.RTR = CAN_RTR_DATA;      // 数据帧
    tx_header.DLC = 8;                 // 数据长度 8 字节

    // 4. 发送
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}