/*
 * debug.h — 云台调试模块
 *
 * 硬件: 心跳 GPIO PB0
 * 依赖: stdint.h, mpu6050.h (GyroSample)
 * 设计要点:
 *   1. 全局 DebugSnapshot 结构体, 每个增稳周期更新一次, 可在 JLink/CCS 中实时查看
 *   2. 环形缓冲区保存最近 50 次循环的完整快照 (≈ 250ms @ 200Hz)
 *   3. 心跳使用软件计数器验证主循环不卡死
 *
 * 使用方法:
 *   - JLink: halt CPU → mem32 &g_debug 31
 *   - CCS: Variables 窗口添加 g_debug
 *   - 逻辑分析仪/示波器测 PB0 → 应看到 1Hz 方波
 *
 * 已知限制: 环形缓冲约 14KB, 无溢出保护 (覆盖写入)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 环形缓冲区大小 (200Hz 下 50 条 ≈ 250ms 历史) */
#define DEBUG_BUF_SIZE  50

/*
 * 单次循环的完整快照
 * sizeof(DebugSnapshot) ≈ 284 bytes, 环形缓冲 ≈ 14KB
 */
typedef struct {
    /* ---- 时间戳 & 状态 ---- */
    uint32_t tick;              /* 主循环计数器, 从 0 递增 */
    uint32_t i2c_errors;        /* mpu6050_get_error_count() 快照 */
    uint8_t  data_valid;        /* 本次传感器数据是否通过校验 (1=有效) */
    uint8_t  enabled;           /* 增稳是否使能 */
    uint8_t  padding[2];

    /* ---- MPU6050 原始传感器数据 ---- */
    float raw_accel_x;          /* g, 加速度计 X */
    float raw_accel_y;          /* g, 加速度计 Y */
    float raw_accel_z;          /* g, 加速度计 Z */
    float raw_gyro_x;           /* °/s, 陀螺 X (原始, 去零偏前) */
    float raw_gyro_y;           /* °/s, 陀螺 Y (原始, 去零偏前) */
    float raw_gyro_z;           /* °/s, 陀螺 Z (原始, 去零偏前, 恒为0) */

    /* ---- 互补滤波后的角度 ---- */
    float pitch;                /* 度, 俯仰角 */
    float roll;                 /* 度, 横滚角 */

    /* ---- 陀螺角速度 (去零偏) ---- */
    float debiased_gyro_x;      /* °/s, 去零偏未滤波 (供角度互补滤波) */
    float debiased_gyro_y;      /* °/s, 去零偏未滤波 */
    float filtered_gyro_x;      /* °/s, 低通滤波后 (供速度环测量值) */
    float filtered_gyro_y;      /* °/s, 低通滤波后 */

    /* ---- 翻滚 (Roll) 角度 PID (外环) ---- */
    float roll_measurement;     /* 测量值 = roll 角度 (度) */
    float roll_error;           /* 设定值 - 测量值 (度) */
    float roll_p_term;          /* 比例项 (°/s) */
    float roll_i_term;          /* 积分项 (°/s) */
    float roll_d_term;          /* 微分项 (°/s) */
    float roll_output;          /* 角度环输出 = 目标角速度 (°/s) */

    /* ---- 俯仰 (Pitch) 角度 PID (外环) ---- */
    float pitch_measurement;    /* 测量值 = pitch 角度 (度) */
    float pitch_error;          /* 设定值 - 测量值 (度) */
    float pitch_p_term;         /* 比例项 (°/s) */
    float pitch_i_term;         /* 积分项 (°/s) */
    float pitch_d_term;         /* 微分项 (°/s) */
    float pitch_output;         /* 角度环输出 = 目标角速度 (°/s) */

    /* ---- 翻滚 (Roll) 速度 PID (内环) ---- */
    float roll_vel_measurement; /* 测量值 = 滤波后角速度 (°/s) */
    float roll_vel_setpoint;    /* 设定值 = 角度环输出 (°/s) */
    float roll_vel_error;       /* 设定值 - 测量值 (°/s) */
    float roll_vel_p_term;      /* 比例项 (Hz) */
    float roll_vel_i_term;      /* 积分项 (Hz) */
    float roll_vel_d_term;      /* 微分项 (Hz) */
    float roll_vel_output;      /* 速度环输出 = 电机 Hz */

    /* ---- 俯仰 (Pitch) 速度 PID (内环) ---- */
    float pitch_vel_measurement;/* 测量值 = 滤波后角速度 (°/s) */
    float pitch_vel_setpoint;   /* 设定值 = 角度环输出 (°/s) */
    float pitch_vel_error;      /* 设定值 - 测量值 (°/s) */
    float pitch_vel_p_term;     /* 比例项 (Hz) */
    float pitch_vel_i_term;     /* 积分项 (Hz) */
    float pitch_vel_d_term;     /* 微分项 (Hz) */
    float pitch_vel_output;     /* 速度环输出 = 电机 Hz */

    /* ---- 电机指令 ---- */
    int32_t motor_a_freq;       /* Hz, 电机 A (翻滚) 当前频率 (0=停止) */
    int32_t motor_a_dir;        /* 0=CCW, 1=CW */
    int32_t motor_b_freq;       /* Hz, 电机 B (俯仰) 当前频率 (0=停止) */
    int32_t motor_b_dir;        /* 0=CCW, 1=CW */

    /* ---- 陀螺零偏 ---- */
    float gyro_bias_x;          /* LSB 原始值 (除以 131 得 °/s) */
    float gyro_bias_y;          /* LSB 原始值 */
    float gyro_bias_z;          /* LSB 原始值 */

    /* ---- 在线零偏校准 ---- */
    float gyro_ema_x;           /* °/s, 去零偏角速度 EMA (静止检测) */
    float gyro_ema_y;           /* °/s, 去零偏角速度 EMA */
} DebugSnapshot;

/* ============================================================
 *  全局调试变量
 * ============================================================ */

/* 全局最新快照 (实时, 每个循环覆盖写入) */
extern volatile DebugSnapshot g_debug;

/* 环形历史缓冲区 */
extern DebugSnapshot          g_debug_history[DEBUG_BUF_SIZE];
extern volatile uint32_t      g_debug_index;

/* 全局错误计数器 (独立于快照, 持续累加) */
extern volatile uint32_t g_debug_i2c_read_fail;     /* I2C 读取失败次数 */
extern volatile uint32_t g_debug_i2c_read_ok;       /* I2C 读取成功次数 */
extern volatile uint32_t g_debug_sensor_val_fail;   /* 传感器数据校验失败次数 */
extern volatile uint32_t g_debug_loop_count;        /* 主循环总次数 */

/* ============================================================
 *  公开接口
 * ============================================================ */

/* 初始化调试模块 (清零所有计数器/缓冲区) */
void debug_init(void);

/*
 * 在增稳循环末尾调用, 填充 g_debug 并写入环形缓冲.
 * 只传控制相关参数, 传感器数据内部调用 mpu6050 API 获取.
 * 参数精简到 11 个, 避免 Cortex-M0+ 栈溢出 (原 48 参数 ≈176B 栈).
 */
void debug_capture(
    float    pitch,   float roll,
    float    gyro_x,  float gyro_y,
    float    roll_meas, float roll_err, float roll_p, float roll_i, float roll_d, float roll_out,
    float    pitch_meas, float pitch_err, float pitch_p, float pitch_i, float pitch_d, float pitch_out,
    /* 速度 PID (滚转) */
    float    roll_vel_meas, float roll_vel_sp, float roll_vel_err,
    float    roll_vel_p, float roll_vel_i, float roll_vel_d, float roll_vel_out,
    /* 速度 PID (俯仰) */
    float    pitch_vel_meas, float pitch_vel_sp, float pitch_vel_err,
    float    pitch_vel_p, float pitch_vel_i, float pitch_vel_d, float pitch_vel_out,
    /* 电机指令 */
    int32_t  motor_a_freq, int32_t motor_a_dir,
    int32_t  motor_b_freq, int32_t motor_b_dir);

/* 心跳: 软件计数器验证主循环存活, 不依赖 GPIO */
void debug_heartbeat(void);

#ifdef __cplusplus
}
#endif
