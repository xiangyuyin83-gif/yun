/*
 * stabilizer.c — 双轴云台增稳实现 (串级 PID)
 *
 * 硬件: 两轴步进电机 (A=PA15, B=PB14), MPU6050 (I2C0)
 * 依赖: step_motor.h, mpu6050.h, debug.h
 * 设计要点:
 *   1. 角度外环: 互补滤波角度 → 目标角速度 (°/s), 200Hz 慢路径
 *   2. 速度内环: 低通滤波陀螺角速度 → 电机 Hz, 1000Hz 快路径
 *   3. 双速架构: 快路径每周期运行, 慢路径每 5 周期运行一次
 *   4. 死区 + 速率限制 + 二次输出钳位 + NaN/Inf 防护
 *   5. I2C 连续故障安全机制: 暂停电机而非永久禁用
 *
 * 已知限制:
 *   1. dt 硬编码: 快路径 1ms, 慢路径 5ms
 */
#include "ti_msp_dl_config.h"

#include "modules/stabilizer/stabilizer.h"

#include "modules/motor/step_motor.h"
#include "modules/sensor/mpu6050.h"
#include "modules/debug/debug.h"

#include <math.h>

/* 速度环 1000Hz, 角度环 200Hz → 每 5 个快周期执行 1 次慢路径 */
#define STAB_SLOW_DIV           5

/* 连续 I2C 错误超过此阈值 → 暂停本周期电机, 等 I2C 恢复后自动继续 */
#define I2C_ERROR_SAFE_LIMIT    200  /* 无上拉时偶发NACK, 放宽避免误停机 */

/* ============================================================
 *  PID 核心实现
 * ============================================================ */

void pid_reset(PIDState *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_error       = 0.0f;
    pid->prev_output      = 0.0f;
}

void pid_init(PIDState *pid, const PIDTuning *tune)
{
    pid->tune     = *tune;
    pid->setpoint = 0.0f;
    pid_reset(pid);
}

float pid_compute(PIDState *pid, float measurement, float dt)
{
    const PIDTuning *t = &pid->tune;

    /* 0. NaN/Inf 防护: 若测量值异常, 直接返回上一次输出, 不做任何更新 */
    if (isnan(measurement) || isinf(measurement)) {
        return pid->prev_output;
    }

    /* 1. 误差 = 目标 - 测量 */
    float error = pid->setpoint - measurement;

    /* 2. 死区: 误差在死区内则清零, 避免噪声触发修正 */
    bool in_deadzone = (fabsf(error) < t->dead_zone);
    if (in_deadzone) {
        error = 0.0f;
    }

    /* 3. 比例项 */
    float p_term = t->kp * error;

    /* 4. 积分项 (梯形积分 + 抗饱和); 死区内不累积, 防止 prev_error 残留导致过充 */
    if (!in_deadzone) {
        pid->integral += 0.5f * (error + pid->prev_error) * dt;
        if (pid->integral >  t->integral_limit) pid->integral =  t->integral_limit;
        if (pid->integral < -t->integral_limit) pid->integral = -t->integral_limit;
    }
    float i_term = t->ki * pid->integral;

    /* 5. 微分项 (测量微分, 防设定值冲击); 死区内屏蔽, 避免噪声驱动 */
    float d_term = 0.0f;
    if (!in_deadzone) {
        d_term = -t->kd * (measurement - pid->prev_measurement) / dt;
    }

    /* 6. 合成输出 */
    float output = p_term + i_term + d_term;

    /* 7. 输出钳位 */
    if (output >  t->output_limit) output =  t->output_limit;
    if (output < -t->output_limit) output = -t->output_limit;

    /* 8. 速率限制: 限制输出变化率 */
    float delta = output - pid->prev_output;
    float rate_limited = pid->prev_output;
    if (delta >  t->rate_limit) {
        rate_limited = pid->prev_output + t->rate_limit;
    } else if (delta < -t->rate_limit) {
        rate_limited = pid->prev_output - t->rate_limit;
    } else {
        rate_limited = output;
    }

    /* 9. 输出钳位 (二次): 速率限制后再次钳位 */
    if (rate_limited >  t->output_limit) rate_limited =  t->output_limit;
    if (rate_limited < -t->output_limit) rate_limited = -t->output_limit;
    output = rate_limited;

    /* 10. NaN/Inf 最终防护: 确保输出始终有限 */
    if (isnan(output) || isinf(output)) {
        output = pid->prev_output;
    }

    /* 11. 保存 PID 调试分量 */
    pid->p_term = p_term;
    pid->i_term = i_term;
    pid->d_term = d_term;

    /* 12. 保存状态 */
    pid->prev_measurement = measurement;
    if (!in_deadzone) {
        pid->prev_error = error;
    }
    pid->prev_output = output;

    return output;
}

/* ============================================================
 *  默认 PID 参数
 * ============================================================ */

/*
 * 翻滚 (Roll) 角度外环
 *   测量值 = 互补滤波角度 (°)
 *   输出   = 目标角速度 (°/s), 供给速度内环
 */
static const PIDTuning g_default_roll_tune = {
    .kp             = 1.5f,
    .ki             = 0.02f,
    .kd             = 0.05f,
    .integral_limit = 20.0f,
    .output_limit   = 60.0f,
    .dead_zone      = 0.5f,
    .rate_limit     = 15.0f,
};

/*
 * 俯仰 (Pitch) 角度外环
 */
static const PIDTuning g_default_pitch_tune = {
    .kp             = 1.5f,
    .ki             = 0.02f,
    .kd             = 0.05f,
    .integral_limit = 20.0f,
    .output_limit   = 60.0f,
    .dead_zone      = 0.5f,
    .rate_limit     = 15.0f,
};

/*
 * 翻滚 (Roll) 速度内环
 *   测量值 = 低通滤波陀螺角速度 (°/s)
 *   设定值 = 角度环输出 (°/s)
 *   输出   = 电机频率 (Hz)
 */
static const PIDTuning g_default_roll_vel_tune = {
    .kp             = 0.6f,
    .ki             = 0.01f,
    .kd             = 0.0f,
    .integral_limit = 100.0f,
    .output_limit   = 2000.0f,
    .dead_zone      = 0.2f,
    .rate_limit     = 50.0f,
};

/*
 * 俯仰 (Pitch) 速度内环
 */
static const PIDTuning g_default_pitch_vel_tune = {
    .kp             = 0.6f,
    .ki             = 0.01f,
    .kd             = 0.0f,
    .integral_limit = 100.0f,
    .output_limit   = 2000.0f,
    .dead_zone      = 0.2f,
    .rate_limit     = 50.0f,
};

/* ============================================================
 *  增稳器生命周期管理
 * ============================================================ */

void stabilizer_init(Stabilizer *stab)
{
    pid_init(&stab->roll_pid,      &g_default_roll_tune);
    pid_init(&stab->pitch_pid,     &g_default_pitch_tune);
    pid_init(&stab->roll_vel_pid,  &g_default_roll_vel_tune);
    pid_init(&stab->pitch_vel_pid, &g_default_pitch_vel_tune);
    stab->enabled     = false;
    stab->last_tick   = 0;
    stab->cycle_count = 0;
}

void stabilizer_set_enabled(Stabilizer *stab, bool en)
{
    if (stab->enabled == en) return;

    stab->enabled = en;

    if (en) {
        /* 复位 PID 动态状态 (保留 tune, setpoint 后续预填) */
        pid_reset(&stab->roll_pid);
        pid_reset(&stab->pitch_pid);
        pid_reset(&stab->roll_vel_pid);
        pid_reset(&stab->pitch_vel_pid);

        /* 使能电机驱动器 */
        step_motor_enable(STEP_MOTOR_A, 1);   /* 两个都使能 */
        step_motor_enable(STEP_MOTOR_B, 1);

        /* 预填角度环: 锁定当前角度为目标 */
        float pitch, roll;
        mpu6050_get_angles(&pitch, &roll);
        stab->pitch_pid.setpoint         = pitch;
        stab->pitch_pid.prev_measurement = pitch;
        stab->roll_pid.setpoint          = roll;
        stab->roll_pid.prev_measurement  = roll;

        /* 预填速度环: 当前角速度作为初始测量, setpoint=0 (静止保持) */
        float gx, gy;
        mpu6050_get_gyro_rates(&gx, &gy);
        stab->roll_vel_pid.prev_measurement  = gx;
        stab->roll_vel_pid.setpoint          = 0.0f;
        stab->pitch_vel_pid.prev_measurement = gy;
        stab->pitch_vel_pid.setpoint         = 0.0f;

        stab->last_tick   = 0;
        stab->cycle_count = 0;
    } else {
        /* 停用增稳: 停止 PWM → 拉低 EN → 电机完全断电 */
        step_motor_stop(STEP_MOTOR_A);
        step_motor_stop(STEP_MOTOR_B);
        step_motor_enable(STEP_MOTOR_A, 0);
        step_motor_enable(STEP_MOTOR_B, 0);
    }
}

/* ============================================================
 *  增稳主循环 (双速串级)
 *
 *  TODO: 使用 SysTick->VAL 或 DWT->CYCCNT 实现精确 dt 测量
 * ============================================================ */

/*
 * 将 PID 输出 (Hz, 带符号表示方向) 转换为电机指令并写入硬件.
 *   motor_hz: 正=CW, 负=CCW, 幅值=目标频率
 *   out_freq/out_dir: 返回实际写入的 Hz 和方向 (供调试快照)
 */
static void motor_apply_command(StepMotorID motor, float motor_hz,
                                int32_t *out_freq, int32_t *out_dir)
{
    float        freq = fabsf(motor_hz);
    StepMotorDir dir  = (motor_hz >= 0.0f) ? STEP_DIR_CW : STEP_DIR_CCW;
    uint32_t     freq_hz;

    if (freq < 0.2f) {
        freq_hz = 0;
    } else {
        freq_hz = (uint32_t)(freq + 0.5f);
    }

    step_motor_set_dir(motor, dir);
    if (freq_hz == 0) {
        step_motor_stop(motor);
    } else {
        step_motor_update_freq(motor, freq_hz);
    }
    *out_freq = (int32_t)freq_hz;
    *out_dir  = (dir == STEP_DIR_CW) ? 1 : 0;
}

void stabilizer_update(Stabilizer *stab)
{
    if (!stab->enabled) return;

    /* --- 0. 传感器健康监测 --- */
    uint32_t i2c_errors = mpu6050_get_error_count();
    if (i2c_errors > I2C_ERROR_SAFE_LIMIT) {
        step_motor_stop(STEP_MOTOR_A);
        step_motor_stop(STEP_MOTOR_B);
        return;
    }

    /* --- 判断慢周期 --- */
    bool is_slow_cycle = (stab->cycle_count % STAB_SLOW_DIV == 0);
    stab->cycle_count++;
    if (stab->cycle_count >= STAB_SLOW_DIV) {
        stab->cycle_count = 0;
    }

    float dt_fast = MPU6050_DT_FAST;  /* 0.001f, 1000Hz */

    /* ================================================================
     *  快路径: 陀螺读取 (仅非慢周期; 慢周期复用 get_angles 缓冲数据)
     * ================================================================ */
    float gyro_x = 0.0f, gyro_y = 0.0f;

    if (!is_slow_cycle) {
        /* 只读陀螺: 读寄存器 → 去零偏 → 低通滤波 → 自适应零偏 → 双缓冲提交 */
        mpu6050_get_gyro_rates(&gyro_x, &gyro_y);
    }
    /* else: 慢周期时 gyro 数据由 get_angles 写入双缓冲, 从缓冲读取 */

    /* ================================================================
     *  慢路径 (每 5 周期, 200Hz): 全传感器 + 互补滤波 + 角度 PID
     * ================================================================ */
    float pitch = 0.0f, roll = 0.0f;
    float roll_angle_out  = 0.0f;
    float pitch_angle_out = 0.0f;

    if (is_slow_cycle) {
        /* 全传感器读取 + 互补滤波 + 双缓冲更新 */
        mpu6050_get_angles(&pitch, &roll);

        /* 角度 PID: 测量 = 互补滤波角度, dt = 5ms */
        float dt_slow = MPU6050_DT_SLOW;
        roll_angle_out  = pid_compute(&stab->roll_pid,  roll,  dt_slow);
        pitch_angle_out = pid_compute(&stab->pitch_pid, pitch, dt_slow);

        /* 串级: 角度环输出 → 速度环设定值 */
        stab->roll_vel_pid.setpoint  = roll_angle_out;
        stab->pitch_vel_pid.setpoint = pitch_angle_out;

        /* 从已提交缓冲读取 gyro 数据 (get_angles 已更新) */
        GyroSample gs;
        mpu6050_get_gyro_sample(&gs);
        gyro_x = gs.filtered_x;
        gyro_y = gs.filtered_y;
    }

    /* ================================================================
     *  速度 PID (每周期, 1000Hz): 测量 = 滤波角速度, dt = 1ms
     * ================================================================ */
    float roll_motor_hz  = pid_compute(&stab->roll_vel_pid,  gyro_x, dt_fast);
    float pitch_motor_hz = pid_compute(&stab->pitch_vel_pid, gyro_y, dt_fast);

    /* ================================================================
     *  方向翻转
     * ================================================================ */
#ifdef ROLL_INVERT
    roll_motor_hz = -roll_motor_hz;
#endif
#ifdef PITCH_INVERT
    pitch_motor_hz = -pitch_motor_hz;
#endif

    /* 电机指令: 频率→方向+Hz, 写硬件 */
    int32_t motor_a_freq = 0, motor_a_dir = 0;
    int32_t motor_b_freq = 0, motor_b_dir = 0;
    motor_apply_command(STEP_MOTOR_A, roll_motor_hz,  &motor_a_freq, &motor_a_dir);
    motor_apply_command(STEP_MOTOR_B, pitch_motor_hz, &motor_b_freq, &motor_b_dir);

    /* ================================================================
     *  调试捕获 (传感器数据由 debug_capture 内部获取, 减少栈压参)
     * ================================================================ */
    debug_capture(
        pitch, roll,
        gyro_x, gyro_y,
        /* 角度 PID (滚转) */
        roll,
        stab->roll_pid.setpoint - roll,
        stab->roll_pid.p_term,
        stab->roll_pid.i_term,
        stab->roll_pid.d_term,
        roll_angle_out,
        /* 角度 PID (俯仰) */
        pitch,
        stab->pitch_pid.setpoint - pitch,
        stab->pitch_pid.p_term,
        stab->pitch_pid.i_term,
        stab->pitch_pid.d_term,
        pitch_angle_out,
        /* 速度 PID (滚转) */
        gyro_x,
        stab->roll_vel_pid.setpoint,
        stab->roll_vel_pid.setpoint - gyro_x,
        stab->roll_vel_pid.p_term,
        stab->roll_vel_pid.i_term,
        stab->roll_vel_pid.d_term,
        roll_motor_hz,
        /* 速度 PID (俯仰) */
        gyro_y,
        stab->pitch_vel_pid.setpoint,
        stab->pitch_vel_pid.setpoint - gyro_y,
        stab->pitch_vel_pid.p_term,
        stab->pitch_vel_pid.i_term,
        stab->pitch_vel_pid.d_term,
        pitch_motor_hz,
        /* 电机指令 */
        motor_a_freq, motor_a_dir,
        motor_b_freq, motor_b_dir);
}
