#include "encoder.h"
#include "tim.h"

/* 每轴脉冲与位移换算比（pulse/mm） */
static const float PULSE_RATIO[ACTUATOR_NUM] = {
    33.8f, 1700.0f / 50.0f, 1700.0f / 50.0f,
    1700.0f / 50.0f, 1700.0f / 50.0f, 1700.0f / 50.0f
};

/* 轴号到编码器定时器句柄映射表 */
static TIM_HandleTypeDef *const ENCODER_TIMS[ACTUATOR_NUM] = {
    &htim1, &htim2, &htim3, &htim4, &htim5, &htim8
};

/* 启动 6 路编码器接口 */
void Encoders_Init(void) {
    for (int i = 0; i < ACTUATOR_NUM; i++) {
        HAL_TIM_Encoder_Start(ENCODER_TIMS[i], TIM_CHANNEL_ALL);
    }
}

/* 读取原始编码器计数 */
int16_t Encoder_GetRawCount(uint8_t axis_idx) {
    if (axis_idx >= ACTUATOR_NUM) {
        return 0;
    }
    return (int16_t)__HAL_TIM_GET_COUNTER(ENCODER_TIMS[axis_idx]);
}

/* 读取位置（mm） */
float Encoder_GetPos_mm(uint8_t axis_idx) {
    if (axis_idx >= ACTUATOR_NUM) {
        return 0.0f;
    }
    return (float)Encoder_GetRawCount(axis_idx) / PULSE_RATIO[axis_idx];
}

/* 清零指定轴编码器计数 */
void Encoder_ResetPos(uint8_t axis_idx) {
    if (axis_idx < ACTUATOR_NUM) {
        __HAL_TIM_SET_COUNTER(ENCODER_TIMS[axis_idx], 0);
    }
}
