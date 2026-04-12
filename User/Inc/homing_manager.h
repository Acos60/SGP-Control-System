#ifndef __HOMING_MANAGER_H
#define __HOMING_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "Actuator.h"

/* 回零管理器处理的轴数量 */
#define HOME_AXIS_NUM 6

/* 单轴回零状态机 */
typedef enum {
    HOME_IDLE = 0,      /* 空闲 */
    HOME_START,         /* 开始 */
    HOME_SEEK,          /* 搜索机械端 */
    HOME_VERIFY_STALL,  /* 停滞二次确认 */
    HOME_ZERO_SET,      /* 写入零点 */
    HOME_DONE,          /* 完成 */
    HOME_FAIL           /* 失败 */
} HomeAxisState_e;

/**
 * @brief 初始化回零状态机
 * @param 无
 * @return 无
 */
void Homing_Init(void);

/**
 * @brief 启动 6 轴并行回零
 * @param 无
 * @return 无
 */
void Homing_StartParallel(void);

/**
 * @brief 中止回零并标记失败
 * @param 无
 * @return 无
 */
void Homing_Abort(void);

/**
 * @brief 推进回零状态机（10ms 调用）
 * @param acts [in/out] 6 轴执行器对象数组
 * @return 无
 */
void Homing_Tick10ms(Actuator_t acts[HOME_AXIS_NUM]);

/**
 * @brief 查询是否正在回零
 * @param 无
 * @return true 正在回零，false 未回零
 */
bool Homing_IsBusy(void);

/**
 * @brief 查询回零是否完成
 * @param 无
 * @return true 已完成，false 未完成
 */
bool Homing_IsDone(void);

/**
 * @brief 查询回零是否失败
 * @param 无
 * @return true 失败，false 未失败
 */
bool Homing_IsFailed(void);

/**
 * @brief 获取每轴状态数组
 * @param 无
 * @return const HomeAxisState_e* 每轴状态首地址
 */
const HomeAxisState_e *Homing_GetStates(void);

#endif
