#include "safety_manager.h"
#include "main.h"

/* 急停锁存标志：一旦置位，仅复位可恢复 */
static volatile bool g_estop_latched = false;

/* 初始化安全状态 */
void Safety_Init(void) { g_estop_latched = false; }

/* 轮询按键，任一按下即锁存急停 */
void Safety_PollKeys_AndUpdate(void) {
    if ((HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) ||
        (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)) {
        g_estop_latched = true;
    }
}

/* 查询急停锁存 */
bool Safety_IsEstopLatched(void) { return g_estop_latched; }

/* 软件触发急停 */
void Safety_ForceEstopLatch(void) { g_estop_latched = true; }

/* 输出门控：急停时强制输出 00 + PWM=0 */
void Safety_GateOutput(uint8_t in1_in, uint8_t in2_in, uint16_t pwm_in, uint8_t *in1_out, uint8_t *in2_out,
                       uint16_t *pwm_out) {
    (void)in1_in;
    (void)in2_in;
    (void)pwm_in;

    if (g_estop_latched) {
        *in1_out = 0u;
        *in2_out = 0u;
        *pwm_out = 0u;
    } else {
        *in1_out = in1_in;
        *in2_out = in2_in;
        *pwm_out = pwm_in;
    }
}
