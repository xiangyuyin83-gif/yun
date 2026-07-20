/*
 * step_motor.c — 双轴步进电机驱动实现
 *
 * 硬件: 电机A (PA15/PB13/PB15), 电机B (PB14/PB16/PA12)
 *       定时器 TIMA0/1, 16 位, CLK=32MHz, 预分频 ÷256
 * 依赖: ti_msp_dl_config.h (GPIO, Timer PWM), step_motor.h
 * 设计要点:
 *   1. 预分频 ÷256 固定, 运行时仅改 LOAD/COMPARE → eff_clock=125kHz
 *   2. 频率缓存 (g_last_freq_a/b) 避免冗余定时器 stop/start
 *   3. 16 位定时器溢出保护 (period 钳制在 [2, 65536])
 *   4. EN 引脚极性由 STEP_MOTOR_EN_ACTIVE_LOW 编译开关控制
 *
 * 已知限制:
 *   1. step_motor_move() 是阻塞调用, 仅供初始化/错误处理, 不在增稳循环中调用
 *   2. 最低频率 ≈ 1.9 Hz (125000/65536), 由 16 位定时器 LOAD 上限决定
 */
#include "ti_msp_dl_config.h"

#include "modules/motor/step_motor.h"

#define PWM_DEFAULT_FREQ    1000
#define PWM_DUTY_PERCENT    50
#define PWM_MIN_FREQ        2
#define PWM_MAX_FREQ        5000

/*
 * 固定预分频: ÷256
 *   eff_clock = 32MHz / 256 = 125kHz
 *   最低频率 = 125000/65536 ≈ 1.9 Hz  ✓ (覆盖 PWM_MIN_FREQ=2)
 *   最高频率 = 125000/2 = 62500 Hz   ✓ (远超 PWM_MAX_FREQ=5000)
 */
#define TIMER_PRESCALE       255U
#define TIMER_EFF_CLOCK      (CPUCLK_FREQ / (TIMER_PRESCALE + 1))  /* 125000 */

/* 缓存上次设定频率，避免稳定运行时重复修改定时器寄存器.
 * volatile: 防止编译器跨函数调用缓存该值, 同时为未来中断上下文的访问预留安全保证. */
static volatile uint32_t g_last_freq_a;
static volatile uint32_t g_last_freq_b;

/* ============================================================
 *  内部辅助函数
 * ============================================================ */

/*
 * 配置定时器周期和占空比 (停止→复位→写LOAD/COMPARE→启动).
 * 通过电机 ID 选择对应的 PWM 实例, 避免跨 SDK 版本的寄存器类型兼容问题.
 * 包含 16 位定时器溢出保护: period 被钳制在 [2, 65536].
 */
static void step_motor_configure_timer(StepMotorID motor,
                                       uint32_t period, uint32_t compare)
{
    /* 16 位定时器 LOAD 上限保护: period-1 必须 ≤ 65535 */
    if (period > 65536U) {
        period = 65536U;
        compare = period / 2;  /* 50% duty, 安全回退 */
    }
    if (period < 2U) {
        period = 2U;
    }

    if (motor == STEP_MOTOR_A) {
        DL_Timer_stopCounter(PWM_1_INST);
        DL_Timer_setTimerCount(PWM_1_INST, 0);
        DL_Timer_setLoadValue(PWM_1_INST, period - 1);
        DL_Timer_setCaptureCompareValue(PWM_1_INST, compare,
            (DL_TIMER_CC_INDEX)DL_TIMER_CC_0_INDEX);
        DL_Timer_startCounter(PWM_1_INST);
    } else {
        DL_Timer_stopCounter(PWM_0_INST);
        DL_Timer_setTimerCount(PWM_0_INST, 0);
        DL_Timer_setLoadValue(PWM_0_INST, period - 1);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, compare,
            (DL_TIMER_CC_INDEX)DL_TIMER_CC_0_INDEX);
        DL_Timer_startCounter(PWM_0_INST);
    }
}

void step_motor_init(void)
{
    /* DIR 默认 HIGH (CW). EN 保持 SysConfig 的 LOW.
     * active-HIGH 使能时 LOW=禁用, 上电安全 ✓ */
    DL_GPIO_setPins(GPIOB, STEP_MOTOR_A_DIR1_PIN);  /* PB13 */
    DL_GPIO_setPins(GPIOB, STEP_MOTOR_B_DIR2_PIN);  /* PB16 */
    g_last_freq_a = 0;
    g_last_freq_b = 0;

    /*
     * 配置定时器固定预分频 ÷256。
     * SysConfig init 已配好 PWM 模式/CC/输出方向, 此处仅改预分频。
     * 两个定时器此时处于停止状态, 安全修改时钟配置。
     */
    DL_TimerA_ClockConfig clk_cfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = TIMER_PRESCALE,
    };
    DL_TimerA_setClockConfig(PWM_0_INST, &clk_cfg);
    DL_TimerA_setClockConfig(PWM_1_INST, &clk_cfg);
}

void step_motor_enable(StepMotorID motor, uint8_t enable)
{
    /*
     * EN 引脚极性由 step_motor.h 中的 STEP_MOTOR_EN_ACTIVE_LOW 控制:
     *   已定义 → active-LOW  (A4988/DRV8825/TMC2208 默认)
     *   未定义 → active-HIGH (当前硬件配置)
     */
#ifdef STEP_MOTOR_EN_ACTIVE_LOW
    if (motor == STEP_MOTOR_A) {
        if (enable) DL_GPIO_clearPins(GPIOB, STEP_MOTOR_A_EN1_PIN);
        else        DL_GPIO_setPins(GPIOB, STEP_MOTOR_A_EN1_PIN);
    } else {
        if (enable) DL_GPIO_clearPins(GPIOA, STEP_MOTOR_B_EN2_PIN);
        else        DL_GPIO_setPins(GPIOA, STEP_MOTOR_B_EN2_PIN);
    }
#else
    /* active-HIGH: enable=1 → HIGH, enable=0 → LOW */
    if (motor == STEP_MOTOR_A) {
        if (enable) DL_GPIO_setPins(GPIOB, STEP_MOTOR_A_EN1_PIN);
        else        DL_GPIO_clearPins(GPIOB, STEP_MOTOR_A_EN1_PIN);
    } else {
        if (enable) DL_GPIO_setPins(GPIOA, STEP_MOTOR_B_EN2_PIN);
        else        DL_GPIO_clearPins(GPIOA, STEP_MOTOR_B_EN2_PIN);
    }
#endif
}

void step_motor_set_dir(StepMotorID motor, StepMotorDir dir)
{
    if (motor == STEP_MOTOR_A) {
        if (dir == STEP_DIR_CW) DL_GPIO_setPins(GPIOB, STEP_MOTOR_A_DIR1_PIN);
        else                    DL_GPIO_clearPins(GPIOB, STEP_MOTOR_A_DIR1_PIN);
    } else {
        if (dir == STEP_DIR_CW) DL_GPIO_setPins(GPIOB, STEP_MOTOR_B_DIR2_PIN);
        else                    DL_GPIO_clearPins(GPIOB, STEP_MOTOR_B_DIR2_PIN);
    }
}

void step_motor_start(StepMotorID motor, uint32_t freq_hz)
{
    /* 钳制频率 (与 update_freq 不同: freq=0 时默认启动而非停止) */
    if (freq_hz == 0) freq_hz = PWM_DEFAULT_FREQ;
    if (freq_hz < PWM_MIN_FREQ) freq_hz = PWM_MIN_FREQ;
    if (freq_hz > PWM_MAX_FREQ) freq_hz = PWM_MAX_FREQ;

    uint32_t period  = TIMER_EFF_CLOCK / freq_hz;
    uint32_t compare = (period * PWM_DUTY_PERCENT) / 100;

    step_motor_configure_timer(motor, period, compare);
    if (motor == STEP_MOTOR_A) {
        g_last_freq_a = freq_hz;  /* 存储钳制后的实际频率 */
    } else {
        g_last_freq_b = freq_hz;
    }
}

void step_motor_stop(StepMotorID motor)
{
    /*
     * 注意: 此函数只停止 PWM 脉冲, 不操作 EN 引脚.
     * 电机驱动器仍可能保持通电 (保持力矩).
     * 若需完全断电, 请额外调用 step_motor_enable(motor, 0).
     */
    if (motor == STEP_MOTOR_A) {
        DL_Timer_stopCounter(PWM_1_INST);
        g_last_freq_a = 0;
    } else {
        DL_Timer_stopCounter(PWM_0_INST);
        g_last_freq_b = 0;
    }
}

void step_motor_update_freq(StepMotorID motor, uint32_t freq_hz)
{
    if (freq_hz == 0) {
        step_motor_stop(motor);
        return;
    }

    if (freq_hz < PWM_MIN_FREQ) freq_hz = PWM_MIN_FREQ;
    if (freq_hz > PWM_MAX_FREQ) freq_hz = PWM_MAX_FREQ;

    /* 频率未变则跳过 — 避免冗余的定时器 stop/start */
    volatile uint32_t *last = (motor == STEP_MOTOR_A)
                              ? &g_last_freq_a : &g_last_freq_b;
    if (freq_hz == *last) return;

    uint32_t period  = TIMER_EFF_CLOCK / freq_hz;
    uint32_t compare = (period * PWM_DUTY_PERCENT) / 100;

    step_motor_configure_timer(motor, period, compare);
    *last = freq_hz;
}

/*
 * 阻塞式步进移动 — 仅供初始化/错误处理使用!
 *
 * 警告: 此函数通过 busy-wait 阻塞 CPU steps/freq_hz 秒.
 * 不要在增稳循环中调用, 否则会丢失传感器数据并导致云台失控.
 * 例: 10000 steps @ 2000Hz → 阻塞 5 秒.
 */
void step_motor_move(StepMotorID motor, StepMotorDir dir,
                     uint32_t steps, uint32_t freq_hz)
{
    if (freq_hz == 0) freq_hz = PWM_DEFAULT_FREQ;

    step_motor_set_dir(motor, dir);
    step_motor_start(motor, freq_hz);

    uint32_t total_us = (uint32_t)((uint64_t)steps * 1000000 / freq_hz);
    delay_us(total_us);

    step_motor_stop(motor);
}

/*
 * 微秒级延时 (阻塞式, busy-wait).
 * CPUCLK_FREQ/1000000 = 每微秒 32 个周期 @ 32MHz.
 * delay_cycles 接收 uint32_t, 因此 us 超过 ~134 秒时会被钳制,
 * 实际使用中远低于此阈值.
 */
void delay_us(uint32_t us)
{
    /* delay_cycles 接收 uint32_t, 防止溢出回绕 */
    uint64_t cycles = (uint64_t)us * (CPUCLK_FREQ / 1000000);
    if (cycles > UINT32_MAX) {
        cycles = UINT32_MAX;
    }
    delay_cycles((uint32_t)cycles);
}
