#ifndef __SAFETY_MANAGER_H
#define __SAFETY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化安全管理器（清除急停锁存）
 * @param 无
 * @return 无
 */
void Safety_Init(void);

/**
 * @brief 轮询 K0/K1 按键并更新急停锁存
 * @param 无
 * @return 无
 */
void Safety_PollKeys_AndUpdate(void);

/**
 * @brief 读取急停锁存状态
 * @param 无
 * @return true 已锁存，false 未锁存
 */
bool Safety_IsEstopLatched(void);

/**
 * @brief 软件触发急停锁存
 * @param 无
 * @return 无
 */
void Safety_ForceEstopLatch(void);

/**
 * @brief 最终输出门控函数
 * @param in1_in  [in] 原始 IN1 命令
 * @param in2_in  [in] 原始 IN2 命令
 * @param pwm_in  [in] 原始 PWM 命令
 * @param in1_out [out] 门控后的 IN1 命令
 * @param in2_out [out] 门控后的 IN2 命令
 * @param pwm_out [out] 门控后的 PWM 命令
 * @return 无
 * @note 急停锁存时强制输出 00 + PWM=0
 */
void Safety_GateOutput(uint8_t in1_in, uint8_t in2_in, uint16_t pwm_in,
                       uint8_t *in1_out, uint8_t *in2_out, uint16_t *pwm_out);

#endif
