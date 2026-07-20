/*
 * step_motor.h — 双轴步进电机驱动
 *
 * 硬件:
 *   电机A: ST=PA15 (PWM_1/TIMA1), DIR=PB13, EN=PB15
 *   电机B: ST=PB14 (PWM_0/TIMA0), DIR=PB16, EN=PA12
 *   定时器预分频: ÷256 (eff_clock=125kHz), 覆盖 1.9~62500Hz
 * 依赖: ti_msp_dl_config.h (GPIO, Timer PWM)
 * 设计要点:
 *   1. 频率缓存避免冗余定时器 stop/start
 *   2. 16 位定时器溢出保护
 *   3. EN 引脚极性可配置 (active-LOW / active-HIGH)
 *
 * 已知限制:
 *   1. step_motor_move() 是阻塞调用, 仅供初始化/错误处理使用
 *   2. 16 位定时器最低频率 ≈ 1.9 Hz (eff_clock=125kHz, LOAD≤65535)
 */

#pragma once

#include "ti_msp_dl_config.h"

/*
 * EN 引脚极性配置:
 *   A4988/DRV8825/TMC2208 的 EN 都是 active-LOW (LOW=使能, HIGH=禁用)
 *   若驱动器是 active-LOW,  取消注释下面这行
 *   若驱动器是 active-HIGH, 保持注释状态
 */
/* #define STEP_MOTOR_EN_ACTIVE_LOW  1 */  /* ← 当前配置: active-HIGH 使能 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  类型定义
 * ============================================================ */

typedef enum {
    STEP_MOTOR_A = 0,   /* 电机A: ST=PA15(PWM_1), DIR=PB13, EN=PB15 */
    STEP_MOTOR_B = 1    /* 电机B: ST=PB14(PWM_0), DIR=PB16, EN=PA12 */
} StepMotorID;

typedef enum {
    STEP_DIR_CW  = 1,
    STEP_DIR_CCW = 0
} StepMotorDir;

/* ============================================================
 *  公开接口
 * ============================================================ */

/* 初始化: 配置定时器预分频 ÷256, DIR 默认 HIGH */
void step_motor_init(void);

/* 使能/禁用电机驱动器 (EN 引脚) */
void step_motor_enable(StepMotorID motor, uint8_t enable);

/* 设置电机方向 */
void step_motor_set_dir(StepMotorID motor, StepMotorDir dir);

/* 启动 PWM 输出 (指定频率, 50% 占空比) */
void step_motor_start(StepMotorID motor, uint32_t freq_hz);

/* 停止 PWM 输出 (不操作 EN 引脚) */
void step_motor_stop(StepMotorID motor);

/* 更新 PWM 频率 (频率未变时跳过寄存器操作, 频率为 0 时停止) */
void step_motor_update_freq(StepMotorID motor, uint32_t freq_hz);

/*
 * 阻塞式步进移动 — 仅供初始化/错误处理使用!
 *
 * 警告: 此函数通过 busy-wait 阻塞 CPU, 不要在增稳循环中调用.
 * 例: 10000 steps @ 2000Hz → 阻塞 5 秒
 */
void step_motor_move(StepMotorID motor, StepMotorDir dir,
                     uint32_t steps, uint32_t freq_hz);

/* 微秒级延时 (阻塞式 busy-wait) */
void delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif
