/*
 * stabilizer.h — 双轴云台增稳 (串级 PID)
 *
 * 硬件: 两轴步进电机 (A=PA15, B=PB14), MPU6050 陀螺仪
 * 依赖: step_motor.h, mpu6050.h
 * 设计要点:
 *   1. 角度外环: 互补滤波角度反馈 → 输出目标角速度 (°/s)
 *   2. 速度内环: 陀螺低通滤波角速度反馈 → 输出电机 Hz
 *   3. 双速架构: 速度环 1000Hz, 角度环 200Hz
 *   4. 死区 + 速率限制 + 积分抗饱和
 *
 * 配置选项:
 *   ROLL_INVERT:  若翻滚修正方向反了, 取消注释来翻转
 *   PITCH_INVERT: 若俯仰修正方向反了, 取消注释来翻转
 *
 * 已知限制:
 *   1. dt 硬编码, 未使用硬件定时器实测
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* #define ROLL_INVERT  1 */  /* 取消注释来翻转翻滚修正方向 */
#define  PITCH_INVERT 1       /* 翻转俯仰修正方向 */

/* ============================================================
 *  PID 参数与状态结构体
 * ============================================================ */

/* PID 参数 (可在线调整, 单位由使用场景决定) */
typedef struct {
    float kp, ki, kd;           /* 增益 */
    float integral_limit;       /* 积分限幅 (输出单位) */
    float output_limit;         /* 输出限幅 (输出单位) */
    float dead_zone;            /* 死区 (测量值单位) */
    float rate_limit;           /* 速率限制 (输出单位/周期) */
} PIDTuning;

/* PID 运行时状态 */
typedef struct {
    PIDTuning tune;
    float setpoint;             /* 目标值 */
    float integral;
    float prev_measurement;
    float prev_error;
    float prev_output;
    /* 调试用: 最近一次的 P/I/D 分量 (由 pid_compute 填充) */
    float p_term;
    float i_term;
    float d_term;
} PIDState;

/*
 * 增稳器: 封装角度环 + 速度环 (串级)
 *
 * 数据流:
 *   角度 PID (roll_pid, pitch_pid):
 *     测量 = 互补滤波角度 (°), 输出 = 目标角速度 (°/s)
 *   速度 PID (roll_vel_pid, pitch_vel_pid):
 *     测量 = 低通滤波角速度 (°/s), 输出 = 电机频率 (Hz)
 */
typedef struct {
    PIDState roll_pid;          /* 角度外环: 翻滚 */
    PIDState pitch_pid;         /* 角度外环: 俯仰 */
    PIDState roll_vel_pid;      /* 速度内环: 翻滚 */
    PIDState pitch_vel_pid;     /* 速度内环: 俯仰 */
    bool     enabled;
    uint32_t last_tick;         /* dt 测量用: 上次调用时间戳 */
    uint8_t  cycle_count;       /* 慢路径门控 (0..STAB_SLOW_DIV-1) */
} Stabilizer;

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  公开接口
 * ============================================================ */

/* PID 初始化: 加载参数并清零动态状态 */
void pid_init(PIDState *pid, const PIDTuning *tune);

/* PID 状态复位: 清零积分/误差/输出 (保留 tune 和 setpoint 不变) */
void pid_reset(PIDState *pid);

/*
 * PID 一次迭代
 *   measurement: 传感器观测值
 *   dt: 时间间隔 (秒)
 *   返回: 控制输出
 *
 * 处理流程: NaN 防护 → 误差 → 死区 → P + I(梯形积分+抗饱和) + D(测量微分) → 输出钳位 → 速率限制
 */
float pid_compute(PIDState *pid, float measurement, float dt);

/* 增稳初始化: 加载默认 PID 参数, 状态清零 */
void stabilizer_init(Stabilizer *stab);

/*
 * 增稳主循环 (每周期调用一次)
 *   快路径 (1000Hz): 陀螺读取 → 速度 PID → 电机更新
 *   慢路径 (200Hz, 每 5 周期): + 全传感器 + 互补滤波 + 角度 PID
 */
void stabilizer_update(Stabilizer *stab);

/*
 * 启用/停用增稳
 *   启用: 复位积分和历史状态, 使能电机, 锁定当前角度为目标,
 *         速度环 setpoint=0 (主动保持静止)
 *   停用: 停止 PWM, 禁用电机驱动器
 */
void stabilizer_set_enabled(Stabilizer *stab, bool en);

#ifdef __cplusplus
}
#endif
