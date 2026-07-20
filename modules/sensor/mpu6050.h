/*
 * mpu6050.h — MPU6050 陀螺仪驱动
 *
 * 硬件: I2C0 (SCL=PA11, SDA=PA10), MPU6050 地址 0x68
 * 依赖: ti_msp_dl_config.h (I2C)
 * 设计要点:
 *   1. 互补滤波融合陀螺和加速度计: α=0.95 (τ≈95ms)
 *   2. 逐字节 I2C 读取 (克隆版不支持多字节连续读)
 *   3. 传感器数据校验: 加速度合成量 [0.5, 3.0]g, 陀螺 ±300°/s
 *   4. 陀螺一阶低通滤波 (fc=50Hz): 抑制速度环 D 项噪声
 *   5. 在线陀螺零偏自适应校准: 双时间常数 EMA 静止检测
 *   6. 双缓冲 GyroSample: 防止并发撕裂读 (为未来 I2C ISR 预留)
 *
 * 已知限制:
 *   1. MPU6050 克隆版不支持多字节连续读, 每字节独立 I2C 事务
 *   2. MSPM0 I2C 硅级 bug: Controller+Target 双模下 RX 数据通路损坏
 *   3. 关闭时钟拉伸可能导致 WHO_AM_I 读到 0 (当前已注释)
 *
 * 量程: 陀螺 ±250°/s, 加速度计 ±2g
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 *  互补滤波常量
 * ============================================================ */

#define MPU6050_DT_SLOW       0.005f    /* 5ms = 200Hz, 互补滤波 dt */
#define MPU6050_DT_FAST       0.001f    /* 1ms = 1000Hz, 速度环 dt */
#define MPU6050_ALPHA         0.95f     /* 陀螺权重 → τ≈95ms, 快速响应 */
#define MPU6050_GYRO_SENS     131.0f    /* LSB/(deg/s) @ ±250°/s */
#define MPU6050_ACCEL_SENS    16384.0f  /* LSB/g @ ±2g */

/* ============================================================
 *  陀螺低通滤波常量 (一阶 IIR)
 * ============================================================ */

/*
 * 截止频率 fc=50Hz, dt=0.001s (1000Hz):
 *   β = 2π × fc × dt = 2π × 50 × 0.001 ≈ 0.314
 * 衰减 −20dB/decade, 相位滞后 45°@50Hz.
 */
#define MPU6050_GYRO_LPF_BETA  0.314f

/* ============================================================
 *  在线零偏自适应校准常量
 * ============================================================ */

#define MPU6050_BIAS_EMA_ALPHA         0.001f   /* EMA 系数 (τ≈1s @ 1000Hz) */
#define MPU6050_BIAS_ADAPT_ALPHA       0.0001f  /* 零偏修正系数 (τ≈10s) */
#define MPU6050_BIAS_MAX_LSB           500.0f   /* 零偏钳制 (LSB, ≈3.8°/s) */

/* ============================================================
 *  双缓冲陀螺数据结构
 * ============================================================ */

/*
 * 一次完整采样的陀螺衍生数据.
 * 写入者 (get_gyro_rates / get_angles) 写入 g_gyro_buf[wr],
 * 完成后原子翻转 wr. 读取者始终从 g_gyro_buf[wr ^ 1] 读取.
 */
typedef struct {
    float raw_x, raw_y;          /* 原始角速度 °/s (去零偏前) */
    float debiased_x, debiased_y;/* 去零偏后角速度 °/s (未滤波) */
    float filtered_x, filtered_y;/* 低通滤波后角速度 °/s (供速度环) */
    bool  valid;                 /* 采样是否通过校验 */
} GyroSample;

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  公开接口
 * ============================================================ */

/*
 * 初始化 MPU6050
 *   1. I2C 硬件复位 + Controller-Only 配置 (规避 MSPM0 硅级 bug)
 *   2. 验证 WHO_AM_I (0x68 或 0x70)
 *   3. 唤醒 + PLL 时钟源
 *   4. 配置 DLPF (BW=44Hz), 陀螺量程 ±250°/s, 加速度计量程 ±2g
 *
 *   返回 true 表示初始化成功
 */
bool mpu6050_init(void);

/*
 * 静态校准: 采集 N 次陀螺数据求零偏 (保持传感器静止)
 *   校验条件:
 *     1. 有效采样数 ≥ 60 (共 200 次)
 *     2. 加速度计方差 < 0.05 g² (确保传感器未移动)
 *
 *   返回 true 表示校准成功, false 表示条件未满足但已尽量计算零偏
 */
bool mpu6050_calibrate(void);

/*
 * 获取互补滤波后的欧拉角 (单位: 度) — 慢路径, ~750us
 *   若 I2C 读取失败或数据校验不通过, 返回上次有效值.
 *   内部同时更新陀螺双缓冲 + 低通滤波 + 自适应零偏.
 */
void mpu6050_get_angles(float *pitch, float *roll);

/*
 * 快速陀螺读取 — 快路径, ~300us
 *   只读 GYRO_XOUT + GYRO_YOUT (4 字节), 跳过加速度计.
 *   内部: 读寄存器 → 去零偏 → 低通滤波 → 自适应零偏更新
 *     → 写入双缓冲 → 原子提交.
 *   返回滤波后的角速度 (°/s), 供速度环 PID 使用.
 *   若 I2C 失败或数据校验不通过, 返回上次有效值, 不提交缓冲.
 */
void mpu6050_get_gyro_rates(float *gx, float *gy);

/*
 * 读取已提交缓冲区的完整陀螺采样
 *   从 g_gyro_buf[wr ^ 1] 拷贝, 用于调试/分析.
 */
void mpu6050_get_gyro_sample(GyroSample *out);

/* 获取连续 I2C 失败次数 (成功读取后清零) — 用于传感器健康监测 */
uint32_t mpu6050_get_error_count(void);

/* ============================================================
 *  调试接口
 * ============================================================ */

/*
 * 获取上一次成功读取的原始传感器数据 (去零偏前, 单位: g / °/s)
 * 若自启动后从未成功读取, 返回全零
 */
void mpu6050_get_last_raw(
    float *accel_x, float *accel_y, float *accel_z,
    float *gyro_x,  float *gyro_y,  float *gyro_z);

/*
 * 获取已提交缓冲区的去零偏未滤波角速度 (°/s)
 * 供互补滤波角度积分和 debug 使用.
 */
void mpu6050_get_debiased_gyro(float *gx, float *gy);

/* 获取/设置陀螺零偏 (LSB 原始单位) */
void mpu6050_get_gyro_biases(float *bx, float *by, float *bz);
void mpu6050_set_gyro_biases(float bx, float by, float bz);

/* 查询上一次读取的数据是否通过校验 (从已提交缓冲读取) */
bool mpu6050_get_last_data_valid(void);

/* 获取在线零偏校准的陀螺 EMA (°/s) — 用于调试静止检测 */
void mpu6050_get_gyro_ema(float *ema_x, float *ema_y);

#ifdef __cplusplus
}
#endif
