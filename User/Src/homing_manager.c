#include "homing_manager.h"
#include "encoder.h"

/* 回零时的搜索 PWM 命令 */
#define HOME_SEEK_PWM 2200u
/* 判定停滞所需连续 tick 数 */
#define HOME_STALL_TICKS 20u
/* 停滞二次确认 tick 数 */
#define HOME_VERIFY_TICKS 8u
/* 回零超时阈值 */
#define HOME_TIMEOUT_TICKS 1200u
/* 判定有效移动的最小编码器变化量 */
#define HOME_MIN_MOVE_COUNTS 8

static HomeAxisState_e g_state[HOME_AXIS_NUM];
static uint16_t g_stall_ticks[HOME_AXIS_NUM];
static uint16_t g_verify_ticks[HOME_AXIS_NUM];
static uint16_t g_seek_ticks[HOME_AXIS_NUM];
static int16_t g_start_cnt[HOME_AXIS_NUM];
static int16_t g_last_cnt[HOME_AXIS_NUM];
static bool g_busy = false;
static bool g_done = false;
static bool g_failed = false;

/* 计算 int32 绝对值 */
static int32_t abs32(int32_t v) { return (v < 0) ? -v : v; }

/* 初始化回零上下文 */
void Homing_Init(void) {
    uint8_t i;
    for (i = 0; i < HOME_AXIS_NUM; i++) {
        g_state[i] = HOME_IDLE;
        g_stall_ticks[i] = 0u;
        g_verify_ticks[i] = 0u;
        g_seek_ticks[i] = 0u;
        g_start_cnt[i] = 0;
        g_last_cnt[i] = 0;
    }
    g_busy = false;
    g_done = false;
    g_failed = false;
}

/* 启动 6 轴并行回零 */
void Homing_StartParallel(void) {
    uint8_t i;
    for (i = 0; i < HOME_AXIS_NUM; i++) {
        g_state[i] = HOME_START;
        g_stall_ticks[i] = 0u;
        g_verify_ticks[i] = 0u;
        g_seek_ticks[i] = 0u;
        g_start_cnt[i] = Encoder_GetRawCount(i);
        g_last_cnt[i] = g_start_cnt[i];
    }
    g_busy = true;
    g_done = false;
    g_failed = false;
}

/* 中止回零并置失败 */
void Homing_Abort(void) {
    uint8_t i;
    for (i = 0; i < HOME_AXIS_NUM; i++) {
        g_state[i] = HOME_FAIL;
    }
    g_busy = false;
    g_done = false;
    g_failed = true;
}

/* 10ms 推进一次并行回零状态机 */
void Homing_Tick10ms(Actuator_t acts[HOME_AXIS_NUM]) {
    uint8_t i;
    bool all_done = true;

    if (!g_busy) {
        return;
    }

    for (i = 0; i < HOME_AXIS_NUM; i++) {
        int16_t cnt = Encoder_GetRawCount(i);

        switch (g_state[i]) {
        case HOME_START:
            g_start_cnt[i] = cnt;
            g_last_cnt[i] = cnt;
            g_stall_ticks[i] = 0u;
            g_verify_ticks[i] = 0u;
            g_seek_ticks[i] = 0u;
            acts[i].cmd_in1 = 1u;
            acts[i].cmd_in2 = 0u;
            acts[i].cmd_pwm = HOME_SEEK_PWM;
            g_state[i] = HOME_SEEK;
            all_done = false;
            break;

        case HOME_SEEK:
            g_seek_ticks[i]++;
            if (cnt != g_last_cnt[i]) {
                g_stall_ticks[i] = 0u;
            } else {
                g_stall_ticks[i]++;
            }
            g_last_cnt[i] = cnt;

            acts[i].cmd_in1 = 1u;
            acts[i].cmd_in2 = 0u;
            acts[i].cmd_pwm = HOME_SEEK_PWM;

            if (g_seek_ticks[i] > HOME_TIMEOUT_TICKS) {
                g_state[i] = HOME_FAIL;
                g_failed = true;
                break;
            }

            if (g_stall_ticks[i] >= HOME_STALL_TICKS) {
                if (abs32((int32_t)cnt - (int32_t)g_start_cnt[i]) >= HOME_MIN_MOVE_COUNTS) {
                    g_verify_ticks[i] = 0u;
                    g_state[i] = HOME_VERIFY_STALL;
                } else {
                    g_state[i] = HOME_FAIL;
                    g_failed = true;
                }
            }
            all_done = false;
            break;

        case HOME_VERIFY_STALL:
            if (cnt == g_last_cnt[i]) {
                g_verify_ticks[i]++;
            } else {
                g_state[i] = HOME_SEEK;
                g_stall_ticks[i] = 0u;
            }
            g_last_cnt[i] = cnt;

            acts[i].cmd_in1 = 1u;
            acts[i].cmd_in2 = 0u;
            acts[i].cmd_pwm = HOME_SEEK_PWM;

            if (g_verify_ticks[i] >= HOME_VERIFY_TICKS) {
                g_state[i] = HOME_ZERO_SET;
            }
            all_done = false;
            break;

        case HOME_ZERO_SET:
            Encoder_ResetPos(i);
            Actuator_SetBrakeCommand(&acts[i]);
            g_state[i] = HOME_DONE;
            break;

        case HOME_DONE:
            Actuator_SetBrakeCommand(&acts[i]);
            break;

        case HOME_FAIL:
            Actuator_SetBrakeCommand(&acts[i]);
            all_done = false;
            break;

        case HOME_IDLE:
        default:
            all_done = false;
            break;
        }
    }

    if (g_failed) {
        g_busy = false;
        g_done = false;
    } else if (all_done) {
        g_busy = false;
        g_done = true;
    }
}

bool Homing_IsBusy(void) { return g_busy; }
bool Homing_IsDone(void) { return g_done; }
bool Homing_IsFailed(void) { return g_failed; }
const HomeAxisState_e *Homing_GetStates(void) { return g_state; }
