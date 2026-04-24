#include "imu_calibrator.h"

#include "Actuator.h"
#include "control_manager.h"
#include "homing_manager.h"
#include "hwt901b.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define IMU_CAL_BASE_LEN_MM 40.0f
#define IMU_CAL_DELTA_LEN_MM 2.0f
#define IMU_CAL_ZERO_SAMPLES 300u
#define IMU_CAL_POSE_SAMPLES 160u
#define IMU_CAL_MIN_SETTLE_MS 800u
#define IMU_CAL_TIMEOUT_MS 12000u
#define IMU_CAL_SAMPLE_TIMEOUT_MS 8000u
#define IMU_CAL_TARGET_ERR_MM 0.5f
#define IMU_CAL_AXIS_VEL_STABLE_MM_S 0.8f
#define IMU_CAL_GYRO_STABLE_DPS 0.8f
#define IMU_CAL_ACC_NORM_MIN_G 0.85f
#define IMU_CAL_ACC_NORM_MAX_G 1.15f

typedef struct {
    float angle_sum[3];
    float angle_sum_sq[3];
    float gyro_rms_sum;
    float acc_norm_sum;
    float mag_norm_sum;
    float last_yaw_unwrapped;
    uint32_t last_imu_ms;
    uint32_t start_ms;
    uint16_t count;
    uint8_t initialized;
} ImuSampleAccum_t;

typedef struct {
    ImuCalState_e state;
    ImuCalMode_e mode;
    ImuCalResult_t result;
    ImuSampleAccum_t sample;
    char last_error[24];
    uint32_t phase_start_ms;
    uint32_t settle_since_ms;
    uint32_t mon_last_ms;
    uint16_t mon_period_ms;
    uint8_t monitor_enabled;
    uint8_t pose_index;
} ImuCalContext_t;

static ImuCalContext_t g_cal;

static float wrap_180(float deg) {
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static float unwrap_near(float raw_deg, float ref_unwrapped_deg) {
    float out = raw_deg;
    while ((out - ref_unwrapped_deg) > 180.0f) {
        out -= 360.0f;
    }
    while ((out - ref_unwrapped_deg) < -180.0f) {
        out += 360.0f;
    }
    return out;
}

static float vec3_norm(const float v[3]) {
    return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static float mag_norm_i16(const int16_t v[3]) {
    float x = (float)v[0];
    float y = (float)v[1];
    float z = (float)v[2];
    return sqrtf((x * x) + (y * y) + (z * z));
}

static void set_error(const char *err) {
    strncpy(g_cal.last_error, err, sizeof(g_cal.last_error) - 1u);
    g_cal.last_error[sizeof(g_cal.last_error) - 1u] = '\0';
}

static void enter_state(ImuCalState_e state) {
    g_cal.state = state;
    g_cal.phase_start_ms = HAL_GetTick();
    g_cal.settle_since_ms = 0u;
}

static void fail_with(const char *err) {
    set_error(err);
    ControlMgr_SetIdleMode();
    enter_state(IMU_CAL_STATE_FAILED);
    printf("ERR,CAL,%s\r\n", err);
}

static bool imu_ready(void) {
    HWT901B_Data_t imu;
    uint32_t now = HAL_GetTick();

    HWT901B_GetData(&imu);
    return (imu.online != 0u) && (imu.last_update_ms != 0u) && ((now - imu.last_update_ms) < 300u);
}

static bool platform_motion_stable(void) {
    HWT901B_Data_t imu;
    float gyro_rms;
    uint8_t i;

    HWT901B_GetData(&imu);
    gyro_rms = vec3_norm(imu.gyro_dps);
    if (gyro_rms > IMU_CAL_GYRO_STABLE_DPS) {
        return false;
    }

    for (i = 0u; i < IMU_CAL_AXIS_NUM; i++) {
        if (fabsf(acts[i].target_pos - acts[i].current_pos) > IMU_CAL_TARGET_ERR_MM) {
            return false;
        }
        if (fabsf(acts[i].current_vel) > IMU_CAL_AXIS_VEL_STABLE_MM_S) {
            return false;
        }
    }
    return true;
}

static bool wait_until_stable(void) {
    uint32_t now = HAL_GetTick();

    if (platform_motion_stable()) {
        if (g_cal.settle_since_ms == 0u) {
            g_cal.settle_since_ms = now;
        }
        return (now - g_cal.settle_since_ms) >= IMU_CAL_MIN_SETTLE_MS;
    }

    g_cal.settle_since_ms = 0u;
    return false;
}

static void sample_reset(void) {
    memset(&g_cal.sample, 0, sizeof(g_cal.sample));
    g_cal.sample.start_ms = HAL_GetTick();
}

static bool sample_push(void) {
    HWT901B_Data_t imu;
    float angle[3];
    float gyro_rms;
    float acc_norm;
    float mag_norm;
    uint8_t i;

    HWT901B_GetData(&imu);
    if ((imu.online == 0u) || (imu.last_update_ms == 0u) || (imu.last_update_ms == g_cal.sample.last_imu_ms)) {
        return false;
    }
    g_cal.sample.last_imu_ms = imu.last_update_ms;

    angle[0] = imu.angle_deg[0];
    angle[1] = imu.angle_deg[1];
    if (!g_cal.sample.initialized) {
        g_cal.sample.last_yaw_unwrapped = imu.angle_deg[2];
        g_cal.sample.initialized = 1u;
    } else {
        g_cal.sample.last_yaw_unwrapped = unwrap_near(imu.angle_deg[2], g_cal.sample.last_yaw_unwrapped);
    }
    angle[2] = g_cal.sample.last_yaw_unwrapped;

    gyro_rms = vec3_norm(imu.gyro_dps);
    acc_norm = vec3_norm(imu.acc_g);
    mag_norm = mag_norm_i16(imu.mag);

    if ((acc_norm < IMU_CAL_ACC_NORM_MIN_G) || (acc_norm > IMU_CAL_ACC_NORM_MAX_G)) {
        return false;
    }

    for (i = 0u; i < 3u; i++) {
        g_cal.sample.angle_sum[i] += angle[i];
        g_cal.sample.angle_sum_sq[i] += angle[i] * angle[i];
    }
    g_cal.sample.gyro_rms_sum += gyro_rms;
    g_cal.sample.acc_norm_sum += acc_norm;
    g_cal.sample.mag_norm_sum += mag_norm;
    g_cal.sample.count++;
    return true;
}

static bool sample_done(uint16_t target_count) {
    uint32_t now = HAL_GetTick();

    (void)sample_push();
    if (g_cal.sample.count >= target_count) {
        return true;
    }
    if ((now - g_cal.sample.start_ms) > IMU_CAL_SAMPLE_TIMEOUT_MS) {
        fail_with("SAMPLE_TIMEOUT");
    }
    return false;
}

static void sample_mean_std(float mean[3], float std_deg[3]) {
    uint8_t i;

    for (i = 0u; i < 3u; i++) {
        float n = (float)g_cal.sample.count;
        float var;

        mean[i] = g_cal.sample.angle_sum[i] / n;
        var = (g_cal.sample.angle_sum_sq[i] / n) - (mean[i] * mean[i]);
        if (var < 0.0f) {
            var = 0.0f;
        }
        std_deg[i] = sqrtf(var);
    }
}

static void finish_zero_sample(void) {
    float mean[3];

    sample_mean_std(mean, g_cal.result.static_std_deg);
    g_cal.result.roll_offset_deg = mean[0];
    g_cal.result.pitch_offset_deg = mean[1];
    g_cal.result.yaw_offset_deg = wrap_180(mean[2]);
    g_cal.result.gyro_rms_dps = g_cal.sample.gyro_rms_sum / (float)g_cal.sample.count;
    g_cal.result.acc_norm_g = g_cal.sample.acc_norm_sum / (float)g_cal.sample.count;
    g_cal.result.mag_norm = g_cal.sample.mag_norm_sum / (float)g_cal.sample.count;
    g_cal.result.zero_samples = g_cal.sample.count;
    g_cal.result.valid = 1u;
}

static void pose_to_axis_delta(uint8_t pose, uint8_t *axis, float *delta) {
    *axis = (uint8_t)(pose / 2u);
    *delta = ((pose % 2u) == 0u) ? IMU_CAL_DELTA_LEN_MM : -IMU_CAL_DELTA_LEN_MM;
}

static bool set_pose_target(uint8_t pose) {
    uint8_t axis;
    float delta;
    float len[IMU_CAL_AXIS_NUM];
    uint8_t i;

    if (!ControlMgr_SetTargetLenAll(IMU_CAL_BASE_LEN_MM)) {
        return false;
    }

    pose_to_axis_delta(pose, &axis, &delta);
    for (i = 0u; i < IMU_CAL_AXIS_NUM; i++) {
        len[i] = IMU_CAL_BASE_LEN_MM;
    }
    len[axis] += delta;
    return ControlMgr_SetTargetLenAxis(axis, len[axis]);
}

static void finish_pose_sample(void) {
    float mean[3];
    float std_deg[3];
    uint8_t axis;
    float delta;
    uint8_t i;

    sample_mean_std(mean, std_deg);
    (void)std_deg;
    pose_to_axis_delta(g_cal.pose_index, &axis, &delta);
    for (i = 0u; i < 3u; i++) {
        float offset;
        if (i == 0u) {
            offset = g_cal.result.roll_offset_deg;
        } else if (i == 1u) {
            offset = g_cal.result.pitch_offset_deg;
        } else {
            offset = g_cal.result.yaw_offset_deg;
        }
        g_cal.result.response_deg_per_mm[i][axis] += wrap_180(mean[i] - offset) / delta;
    }
    g_cal.result.pose_samples = g_cal.sample.count;
}

static void average_full_response(void) {
    uint8_t axis;
    uint8_t i;

    for (i = 0u; i < 3u; i++) {
        for (axis = 0u; axis < IMU_CAL_AXIS_NUM; axis++) {
            g_cal.result.response_deg_per_mm[i][axis] *= 0.5f;
        }
    }
}

static void monitor_tick(void) {
    HWT901B_Data_t imu;
    float cal[3];
    float raw[3];
    float gyro_rms;
    float acc_norm;
    uint32_t now;

    if (!g_cal.monitor_enabled) {
        return;
    }

    now = HAL_GetTick();
    if ((g_cal.mon_last_ms != 0u) && ((now - g_cal.mon_last_ms) < g_cal.mon_period_ms)) {
        return;
    }
    g_cal.mon_last_ms = now;

    HWT901B_GetData(&imu);
    raw[0] = imu.angle_deg[0];
    raw[1] = imu.angle_deg[1];
    raw[2] = imu.angle_deg[2];
    (void)ImuCal_ApplyToAngles(raw, cal);
    gyro_rms = vec3_norm(imu.gyro_dps);
    acc_norm = vec3_norm(imu.acc_g);

    printf("CALDATA,%lu,%d,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\r\n", (unsigned long)now,
           (int)g_cal.state, (unsigned)g_cal.pose_index, raw[0], raw[1], raw[2], cal[0], cal[1], cal[2], gyro_rms,
           acc_norm, platform_motion_stable() ? 1u : 0u);
}

void ImuCal_Init(void) {
    memset(&g_cal, 0, sizeof(g_cal));
    g_cal.state = IMU_CAL_STATE_IDLE;
    g_cal.mode = IMU_CAL_MODE_NONE;
}

void ImuCal_Tick(void) {
    uint32_t now;

    monitor_tick();
    now = HAL_GetTick();

    switch (g_cal.state) {
    case IMU_CAL_STATE_IDLE:
    case IMU_CAL_STATE_DONE:
    case IMU_CAL_STATE_ABORTED:
    case IMU_CAL_STATE_FAILED:
        return;

    case IMU_CAL_STATE_PRECHECK:
        if (ControlMgr_IsEstopLatched()) {
            fail_with("ESTOP");
        } else if (!imu_ready()) {
            fail_with("IMU_OFFLINE");
        } else {
            enter_state(IMU_CAL_STATE_HOME_START);
        }
        break;

    case IMU_CAL_STATE_HOME_START:
        ControlMgr_StartHoming();
        enter_state(IMU_CAL_STATE_HOME_WAIT);
        break;

    case IMU_CAL_STATE_HOME_WAIT:
        if (ControlMgr_IsEstopLatched()) {
            fail_with("ESTOP");
        } else if (Homing_IsFailed()) {
            fail_with("HOME_FAILED");
        } else if (ControlMgr_IsHomed()) {
            enter_state(IMU_CAL_STATE_MOVE_BASE);
        } else if ((now - g_cal.phase_start_ms) > 30000u) {
            fail_with("HOME_TIMEOUT");
        }
        break;

    case IMU_CAL_STATE_MOVE_BASE:
        if (!ControlMgr_SetTargetLenAll(IMU_CAL_BASE_LEN_MM)) {
            fail_with("BASE_RANGE");
        } else {
            ControlMgr_SetRunMode();
            enter_state(IMU_CAL_STATE_SETTLE);
        }
        break;

    case IMU_CAL_STATE_SETTLE:
        if (!imu_ready()) {
            fail_with("IMU_OFFLINE");
        } else if (wait_until_stable()) {
            sample_reset();
            if (g_cal.result.valid == 0u) {
                enter_state(IMU_CAL_STATE_SAMPLE_ZERO);
            } else {
                enter_state(IMU_CAL_STATE_SAMPLE_POSE);
            }
        } else if ((now - g_cal.phase_start_ms) > IMU_CAL_TIMEOUT_MS) {
            fail_with("SETTLE_TIMEOUT");
        }
        break;

    case IMU_CAL_STATE_SAMPLE_ZERO:
        if (sample_done(IMU_CAL_ZERO_SAMPLES)) {
            finish_zero_sample();
            if (g_cal.mode == IMU_CAL_MODE_QUICK) {
                ControlMgr_SetIdleMode();
                enter_state(IMU_CAL_STATE_DONE);
                printf("OK,CAL,QUICK_DONE\r\n");
            } else {
                memset(g_cal.result.response_deg_per_mm, 0, sizeof(g_cal.result.response_deg_per_mm));
                g_cal.pose_index = 0u;
                enter_state(IMU_CAL_STATE_MOVE_POSE);
            }
        }
        break;

    case IMU_CAL_STATE_MOVE_POSE:
        if (g_cal.pose_index >= (IMU_CAL_AXIS_NUM * 2u)) {
            average_full_response();
            (void)ControlMgr_SetTargetLenAll(IMU_CAL_BASE_LEN_MM);
            ControlMgr_SetIdleMode();
            enter_state(IMU_CAL_STATE_DONE);
            printf("OK,CAL,FULL_DONE\r\n");
        } else if (!set_pose_target(g_cal.pose_index)) {
            fail_with("POSE_RANGE");
        } else {
            ControlMgr_SetRunMode();
            enter_state(IMU_CAL_STATE_SETTLE);
        }
        break;

    case IMU_CAL_STATE_SAMPLE_POSE:
        if (sample_done(IMU_CAL_POSE_SAMPLES)) {
            finish_pose_sample();
            g_cal.pose_index++;
            enter_state(IMU_CAL_STATE_MOVE_POSE);
        }
        break;

    default:
        fail_with("BAD_STATE");
        break;
    }
}

bool ImuCal_StartQuick(void) {
    if ((g_cal.state != IMU_CAL_STATE_IDLE) && (g_cal.state != IMU_CAL_STATE_DONE) &&
        (g_cal.state != IMU_CAL_STATE_ABORTED) && (g_cal.state != IMU_CAL_STATE_FAILED)) {
        return false;
    }
    memset(&g_cal.result, 0, sizeof(g_cal.result));
    set_error("NONE");
    g_cal.mode = IMU_CAL_MODE_QUICK;
    g_cal.pose_index = 0u;
    enter_state(IMU_CAL_STATE_PRECHECK);
    return true;
}

bool ImuCal_StartFull(void) {
    if (!ImuCal_StartQuick()) {
        return false;
    }
    g_cal.mode = IMU_CAL_MODE_FULL;
    return true;
}

void ImuCal_Abort(void) {
    if ((g_cal.state != IMU_CAL_STATE_IDLE) && (g_cal.state != IMU_CAL_STATE_DONE)) {
        ControlMgr_SetIdleMode();
        set_error("ABORTED");
        enter_state(IMU_CAL_STATE_ABORTED);
    }
}

void ImuCal_EnableMonitor(uint16_t period_ms) {
    g_cal.mon_period_ms = (period_ms < 10u) ? 10u : period_ms;
    g_cal.monitor_enabled = 1u;
    g_cal.mon_last_ms = 0u;
    printf("CALDATA,t_ms,state,pose,roll,pitch,yaw,roll_cal,pitch_cal,yaw_cal,gyro_rms,acc_norm,stable\r\n");
}

void ImuCal_DisableMonitor(void) {
    g_cal.monitor_enabled = 0u;
}

bool ImuCal_ApplyToAngles(const float raw_deg[3], float calibrated_deg[3]) {
    calibrated_deg[0] = raw_deg[0] - g_cal.result.roll_offset_deg;
    calibrated_deg[1] = raw_deg[1] - g_cal.result.pitch_offset_deg;
    calibrated_deg[2] = wrap_180(raw_deg[2] - g_cal.result.yaw_offset_deg);
    return g_cal.result.valid != 0u;
}

ImuCalState_e ImuCal_GetState(void) {
    return g_cal.state;
}

ImuCalMode_e ImuCal_GetMode(void) {
    return g_cal.mode;
}

const char *ImuCal_GetStateName(void) {
    switch (g_cal.state) {
    case IMU_CAL_STATE_IDLE:
        return "IDLE";
    case IMU_CAL_STATE_PRECHECK:
        return "PRECHECK";
    case IMU_CAL_STATE_HOME_START:
        return "HOME_START";
    case IMU_CAL_STATE_HOME_WAIT:
        return "HOME_WAIT";
    case IMU_CAL_STATE_MOVE_BASE:
        return "MOVE_BASE";
    case IMU_CAL_STATE_SETTLE:
        return "SETTLE";
    case IMU_CAL_STATE_SAMPLE_ZERO:
        return "SAMPLE_ZERO";
    case IMU_CAL_STATE_MOVE_POSE:
        return "MOVE_POSE";
    case IMU_CAL_STATE_SAMPLE_POSE:
        return "SAMPLE_POSE";
    case IMU_CAL_STATE_DONE:
        return "DONE";
    case IMU_CAL_STATE_ABORTED:
        return "ABORTED";
    case IMU_CAL_STATE_FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}

const char *ImuCal_GetLastError(void) {
    return g_cal.last_error;
}

const ImuCalResult_t *ImuCal_GetResult(void) {
    return &g_cal.result;
}

void ImuCal_PrintStatus(void) {
    printf("DATA,CAL_STATUS,state=%s,mode=%d,pose=%u,zero_n=%u,pose_n=%u,valid=%u,error=%s\r\n",
           ImuCal_GetStateName(), (int)g_cal.mode, (unsigned)g_cal.pose_index, (unsigned)g_cal.result.zero_samples,
           (unsigned)g_cal.sample.count, (unsigned)g_cal.result.valid, g_cal.last_error);
}

void ImuCal_PrintReport(void) {
    uint8_t axis;

    printf("DATA,CAL_REPORT,mode=%d,valid=%u,roll0=%.4f,pitch0=%.4f,yaw0=%.4f,relative_yaw=1,std_r=%.4f,std_p=%.4f,std_y=%.4f,gyro_rms=%.4f,acc_norm=%.4f,mag_norm=%.2f,zero_n=%u,pose_n=%u,result=%s\r\n",
           (int)g_cal.mode, (unsigned)g_cal.result.valid, g_cal.result.roll_offset_deg, g_cal.result.pitch_offset_deg,
           g_cal.result.yaw_offset_deg, g_cal.result.static_std_deg[0], g_cal.result.static_std_deg[1],
           g_cal.result.static_std_deg[2], g_cal.result.gyro_rms_dps, g_cal.result.acc_norm_g,
           g_cal.result.mag_norm, (unsigned)g_cal.result.zero_samples, (unsigned)g_cal.result.pose_samples,
           g_cal.result.valid ? "PASS" : "NO_DATA");

    for (axis = 0u; axis < IMU_CAL_AXIS_NUM; axis++) {
        printf("DATA,CAL_K,axis=%u,roll=%.6f,pitch=%.6f,yaw=%.6f\r\n", (unsigned)(axis + 1u),
               g_cal.result.response_deg_per_mm[0][axis], g_cal.result.response_deg_per_mm[1][axis],
               g_cal.result.response_deg_per_mm[2][axis]);
    }
}
