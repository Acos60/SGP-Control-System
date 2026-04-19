#include "homing_manager.h"
#include "encoder.h"

/* 回零方向与 Actuator_ManualHome 保持一致：IN1=0, IN2=1 */
#define HOME_SEEK_IN1 0u
#define HOME_SEEK_IN2 1u
/* 回零时的搜索 PWM 命令，与旧阻塞回零流程保持一致 */
#define HOME_SEEK_PWM 3500u
/* 启动宽限 tick 数，避免刚启动或机械间隙阶段被误判为停滞 */
#define HOME_STARTUP_IGNORE_TICKS 20u
/* 判定停滞所需连续 tick 数，与旧阻塞回零流程保持一致 */
#define HOME_STALL_TICKS 20u
/* 回零超时阈值 */
#define HOME_TIMEOUT_TICKS 1200u

static HomeAxisState_e g_state[HOME_AXIS_NUM];
static uint16_t g_stall_ticks[HOME_AXIS_NUM];
static uint16_t g_startup_ticks[HOME_AXIS_NUM];
static uint16_t g_seek_ticks[HOME_AXIS_NUM];
static int16_t g_last_cnt[HOME_AXIS_NUM];
static bool g_busy = false;
static bool g_done = false;
static bool g_failed = false;

/* 初始化回零上下文 */
void Homing_Init(void) {
    uint8_t i;
    for (i = 0; i < HOME_AXIS_NUM; i++) {
        g_state[i] = HOME_IDLE;
        g_stall_ticks[i] = 0u;
        g_startup_ticks[i] = 0u;
        g_seek_ticks[i] = 0u;
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
        g_startup_ticks[i] = 0u;
        g_seek_ticks[i] = 0u;
        g_last_cnt[i] = Encoder_GetRawCount(i);
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
            g_last_cnt[i] = cnt;
            g_stall_ticks[i] = 0u;
            g_startup_ticks[i] = 0u;
            g_seek_ticks[i] = 0u;
            acts[i].cmd_in1 = HOME_SEEK_IN1;
            acts[i].cmd_in2 = HOME_SEEK_IN2;
            acts[i].cmd_pwm = HOME_SEEK_PWM;
            g_state[i] = HOME_SEEK;
            all_done = false;
            break;

        case HOME_SEEK:
            g_seek_ticks[i]++;
            acts[i].cmd_in1 = HOME_SEEK_IN1;
            acts[i].cmd_in2 = HOME_SEEK_IN2;
            acts[i].cmd_pwm = HOME_SEEK_PWM;

            if (g_seek_ticks[i] > HOME_TIMEOUT_TICKS) {
                g_state[i] = HOME_FAIL;
                g_failed = true;
                break;
            }

            if (g_startup_ticks[i] < HOME_STARTUP_IGNORE_TICKS) {
                g_startup_ticks[i]++;
                g_stall_ticks[i] = 0u;
                g_last_cnt[i] = cnt;
                all_done = false;
                break;
            }

            if (cnt != g_last_cnt[i]) {
                g_stall_ticks[i] = 0u;
                g_last_cnt[i] = cnt;
            } else {
                g_stall_ticks[i]++;
            }

            if (g_stall_ticks[i] >= HOME_STALL_TICKS) {
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
