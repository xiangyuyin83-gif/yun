/*
 * debug.c — 云台调试模块实现
 *
 * 硬件: 软件心跳 (g_debug_loop_count 计数替代 GPIO)
 * 依赖: ti_msp_dl_config.h, mpu6050.h, debug.h
 * 设计要点:
 *   1. 手动清零/拷贝结构体, 不依赖 memset/memcpy, 避免 libc 链接问题
 *   2. 环形缓冲区: 写入 g_debug_history[g_debug_index % DEBUG_BUF_SIZE]
 *   3. 全局最新快照 g_debug 每周期覆盖写入 (volatile, 供 JLink/CCS 实时观察)
 *   4. 全局错误计数器独立于快照, 持续累加
 *
 * 已知限制:
 *   1. GPIO 心跳已移除, 用 g_debug_loop_count 软件方式验证主循环存活
 *   2. 环形缓冲无上溢保护 (覆盖写入)
 */
#include "ti_msp_dl_config.h"

#include "modules/debug/debug.h"

#include "modules/sensor/mpu6050.h"

/* ============================================================
 *  全局变量
 * ============================================================ */

/* 最新快照 (实时覆盖) */
volatile DebugSnapshot g_debug;

/* 环形缓冲区 */
DebugSnapshot          g_debug_history[DEBUG_BUF_SIZE];
volatile uint32_t      g_debug_index = 0;

/* 持续累加的全局计数器 */
volatile uint32_t g_debug_i2c_read_fail   = 0;
volatile uint32_t g_debug_i2c_read_ok     = 0;
volatile uint32_t g_debug_sensor_val_fail = 0;
volatile uint32_t g_debug_loop_count      = 0;

/* ============================================================
 *  内部辅助函数
 * ============================================================ */

/*
 * 手动清零结构体: 不依赖 memset, 避免 libc 链接问题
 */
static void debug_clear_snapshot(DebugSnapshot *s)
{
    volatile uint8_t *p = (volatile uint8_t *)s;
    unsigned int i;
    for (i = 0; i < sizeof(DebugSnapshot); i++) {
        p[i] = 0;
    }
}

/*
 * 手动拷贝结构体: 不依赖 memcpy, 避免 libc 链接问题
 */
static void debug_copy_snapshot(DebugSnapshot *dst, const DebugSnapshot *src)
{
    volatile uint8_t       *d = (volatile uint8_t *)dst;
    const volatile uint8_t *s = (const volatile uint8_t *)src;
    unsigned int i;
    for (i = 0; i < sizeof(DebugSnapshot); i++) {
        d[i] = s[i];
    }
}

/* ============================================================
 *  公开接口实现
 * ============================================================ */

void debug_init(void)
{
    unsigned int i;

    /* 配置心跳 GPIO: PB0, 数字输出, 初始 LOW
     * 若此行导致 crash: IOMUX_PINCM17 可能未在 SysConfig 中生成,
     * 或 PB0 已被其他外设占用. 注释掉用软件心跳替代. */
    /* DL_GPIO_initDigitalOutput(IOMUX_PINCM17); */

    /* 清零最新快照 */
    debug_clear_snapshot((DebugSnapshot *)&g_debug);

    /* 清零环形缓冲区 */
    for (i = 0; i < DEBUG_BUF_SIZE; i++) {
        debug_clear_snapshot(&g_debug_history[i]);
    }
    g_debug_index = 0;

    /* 清零计数器 */
    g_debug_i2c_read_fail   = 0;
    g_debug_i2c_read_ok     = 0;
    g_debug_sensor_val_fail = 0;
    g_debug_loop_count      = 0;
}

void debug_capture(
    float    pitch,   float roll,
    float    gyro_x,  float gyro_y,
    float    roll_meas, float roll_err, float roll_p, float roll_i, float roll_d, float roll_out,
    float    pitch_meas, float pitch_err, float pitch_p, float pitch_i, float pitch_d, float pitch_out,
    float    roll_vel_meas, float roll_vel_sp, float roll_vel_err,
    float    roll_vel_p, float roll_vel_i, float roll_vel_d, float roll_vel_out,
    float    pitch_vel_meas, float pitch_vel_sp, float pitch_vel_err,
    float    pitch_vel_p, float pitch_vel_i, float pitch_vel_d, float pitch_vel_out,
    int32_t  motor_a_freq, int32_t motor_a_dir,
    int32_t  motor_b_freq, int32_t motor_b_dir)
{
    /* 直接写全局 g_debug (BSS), 避免 204B 局部 DebugSnapshot 压栈 */
    DebugSnapshot *snap = (DebugSnapshot *)&g_debug;

    snap->tick       = g_debug_loop_count;
    snap->i2c_errors = mpu6050_get_error_count();

    /* 传感器原始 + 处理后数据: 从 MPU6050 模块内部获取 */
    snap->data_valid = (uint8_t)mpu6050_get_last_data_valid();

    float raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu6050_get_last_raw(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    snap->raw_accel_x = raw_ax;
    snap->raw_accel_y = raw_ay;
    snap->raw_accel_z = raw_az;
    snap->raw_gyro_x  = raw_gx;
    snap->raw_gyro_y  = raw_gy;
    snap->raw_gyro_z  = raw_gz;

    snap->pitch = pitch;
    snap->roll  = roll;

    /* 陀螺角速度 (滤波前后对比) */
    GyroSample gs;
    mpu6050_get_gyro_sample(&gs);
    snap->debiased_gyro_x = gs.debiased_x;
    snap->debiased_gyro_y = gs.debiased_y;
    snap->filtered_gyro_x = gs.filtered_x;
    snap->filtered_gyro_y = gs.filtered_y;

    snap->enabled = 1;

    /* 角度 PID (滚转) */
    snap->roll_measurement = roll_meas;
    snap->roll_error       = roll_err;
    snap->roll_p_term      = roll_p;
    snap->roll_i_term      = roll_i;
    snap->roll_d_term      = roll_d;
    snap->roll_output      = roll_out;

    /* 角度 PID (俯仰) */
    snap->pitch_measurement = pitch_meas;
    snap->pitch_error       = pitch_err;
    snap->pitch_p_term      = pitch_p;
    snap->pitch_i_term      = pitch_i;
    snap->pitch_d_term      = pitch_d;
    snap->pitch_output      = pitch_out;

    /* 速度 PID (滚转) */
    snap->roll_vel_measurement = roll_vel_meas;
    snap->roll_vel_setpoint    = roll_vel_sp;
    snap->roll_vel_error       = roll_vel_err;
    snap->roll_vel_p_term      = roll_vel_p;
    snap->roll_vel_i_term      = roll_vel_i;
    snap->roll_vel_d_term      = roll_vel_d;
    snap->roll_vel_output      = roll_vel_out;

    /* 速度 PID (俯仰) */
    snap->pitch_vel_measurement = pitch_vel_meas;
    snap->pitch_vel_setpoint    = pitch_vel_sp;
    snap->pitch_vel_error       = pitch_vel_err;
    snap->pitch_vel_p_term      = pitch_vel_p;
    snap->pitch_vel_i_term      = pitch_vel_i;
    snap->pitch_vel_d_term      = pitch_vel_d;
    snap->pitch_vel_output      = pitch_vel_out;

    /* 电机指令 */
    snap->motor_a_freq = motor_a_freq;
    snap->motor_a_dir  = motor_a_dir;
    snap->motor_b_freq = motor_b_freq;
    snap->motor_b_dir  = motor_b_dir;

    /* 陀螺零偏 + EMA: 从 MPU6050 模块获取 */
    float bias_x, bias_y, bias_z;
    mpu6050_get_gyro_biases(&bias_x, &bias_y, &bias_z);
    snap->gyro_bias_x = bias_x;
    snap->gyro_bias_y = bias_y;
    snap->gyro_bias_z = bias_z;

    float ema_x, ema_y;
    mpu6050_get_gyro_ema(&ema_x, &ema_y);
    snap->gyro_ema_x = ema_x;
    snap->gyro_ema_y = ema_y;

    /* 写入环形缓冲区 (从全局快照拷贝) */
    uint32_t idx = g_debug_index % DEBUG_BUF_SIZE;
    debug_copy_snapshot(&g_debug_history[idx], snap);
    g_debug_index++;
}

/*
 * 心跳 GPIO (PB0): 每 HEARTBEAT_DIV 次调用翻转一次.
 * 1000Hz 主循环 / 500 = 2Hz 翻转 → 1Hz 方波.
 * 用示波器/逻辑分析仪测 PB0: 应看到 1Hz 方波, 确认主循环存活.
 */
#define HEARTBEAT_DIV  500

void debug_heartbeat(void)
{
    static uint32_t counter = 0;
    counter++;
    if (counter >= HEARTBEAT_DIV) {
        counter = 0;
        /* DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_0);  TODO: 启用需先 init PB0 */
    }
}
