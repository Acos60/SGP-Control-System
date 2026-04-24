#ifndef __IMU_CALIBRATOR_H
#define __IMU_CALIBRATOR_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_CAL_AXIS_NUM 6u

typedef enum {
    IMU_CAL_STATE_IDLE = 0,
    IMU_CAL_STATE_PRECHECK,
    IMU_CAL_STATE_HOME_START,
    IMU_CAL_STATE_HOME_WAIT,
    IMU_CAL_STATE_MOVE_BASE,
    IMU_CAL_STATE_SETTLE,
    IMU_CAL_STATE_SAMPLE_ZERO,
    IMU_CAL_STATE_MOVE_POSE,
    IMU_CAL_STATE_SAMPLE_POSE,
    IMU_CAL_STATE_DONE,
    IMU_CAL_STATE_ABORTED,
    IMU_CAL_STATE_FAILED
} ImuCalState_e;

typedef enum {
    IMU_CAL_MODE_NONE = 0,
    IMU_CAL_MODE_QUICK,
    IMU_CAL_MODE_FULL
} ImuCalMode_e;

typedef struct {
    float roll_offset_deg;
    float pitch_offset_deg;
    float yaw_offset_deg;
    float response_deg_per_mm[3][IMU_CAL_AXIS_NUM];
    float static_std_deg[3];
    float gyro_rms_dps;
    float acc_norm_g;
    float mag_norm;
    uint16_t zero_samples;
    uint16_t pose_samples;
    uint8_t valid;
} ImuCalResult_t;

void ImuCal_Init(void);
void ImuCal_Tick(void);
bool ImuCal_StartQuick(void);
bool ImuCal_StartFull(void);
void ImuCal_Abort(void);
void ImuCal_EnableMonitor(uint16_t period_ms);
void ImuCal_DisableMonitor(void);
bool ImuCal_ApplyToAngles(const float raw_deg[3], float calibrated_deg[3]);
ImuCalState_e ImuCal_GetState(void);
ImuCalMode_e ImuCal_GetMode(void);
const char *ImuCal_GetStateName(void);
const char *ImuCal_GetLastError(void);
const ImuCalResult_t *ImuCal_GetResult(void);
void ImuCal_PrintStatus(void);
void ImuCal_PrintReport(void);

#endif
