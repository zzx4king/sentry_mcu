/**
 * C620 电调协议解析
 * 参考大疆 C620 电调手册 + RM 开发板 C 型 14.CAN 官方例程
 *
 * 所有函数均设计为可在 FreeRTOS 任务中安全调用，不含阻塞操作。
 */

#include "dbus_c620.h"

/**
 * @brief  将 4 个电机的目标电流打包为 8 字节 CAN 数据帧
 * @param  m1~m4: 目标电流 [-16384, 16384]
 * @param  tx_data[8]: 输出 CAN 数据域
 */
void C620_PackChassisCmd(int16_t m1, int16_t m2, int16_t m3, int16_t m4,
                         uint8_t tx_data[8])
{
    tx_data[0] = (uint8_t)(m1 >> 8);
    tx_data[1] = (uint8_t)(m1);
    tx_data[2] = (uint8_t)(m2 >> 8);
    tx_data[3] = (uint8_t)(m2);
    tx_data[4] = (uint8_t)(m3 >> 8);
    tx_data[5] = (uint8_t)(m3);
    tx_data[6] = (uint8_t)(m4 >> 8);
    tx_data[7] = (uint8_t)(m4);
}

/**
 * @brief  解析电机反馈帧 (8 字节)
 * @param  rx_data[8]: 原始反馈数据 (大端序)
 * @param  motor: 输出解析结果
 */
void C620_DecodeFeedback(const uint8_t rx_data[8], Motor_Feedback_t *motor)
{
    if (!motor) return;

    /* 保存上一次的角度，用于后续计算转子总圈数 */
    motor->last_ecd = motor->ecd;

    /* 按大疆手册格式解析 (大端序) */
    motor->ecd           = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
    motor->speed_rpm     = (int16_t)((uint16_t)rx_data[2] << 8 | rx_data[3]);
    motor->given_current = (int16_t)((uint16_t)rx_data[4] << 8 | rx_data[5]);
    motor->temperate     = rx_data[6];
}
