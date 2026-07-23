/*
 * empty_cpp.cpp — 双轴云台主程序
 *
 * 硬件:
 *   电机A (翻滚): ST=PA15(PWM_1/TIMA1), DIR=PB13, EN=PB15
 *   电机B (俯仰): ST=PB14(PWM_0/TIMA0), DIR=PB16, EN=PA12
 *   MPU6050:      SCL=PA11(I2C0), SDA=PA10(I2C0)
 *   EN 极性:      active-HIGH (HIGH=使能, LOW=禁用)
 *
 * 依赖: ti_msp_dl_config.h, step_motor.h
 *
 * 启动流程:
 *   1. 平台初始化 (SysConfig + 电机)
 *   2. [ENABLE_GYRO_STABILIZER] MPU6050 初始化 + 陀螺零偏校准
 *   3. [ENABLE_GYRO_STABILIZER] 增稳器初始化 + 互补滤波预热
 *   4. 进入主循环 [陀螺增稳 / 跟踪 / 自定义]
 *
 * 已知限制:
 *   1. 看门狗通过 #if 0 禁用, 待确认 MSPM0 SDK 版本后启用
 *   2. delay_us(200) 硬编码; 快周期 ~800us, 慢周期 ~1500us
 */

/* ============================================================
 *  功能开关: 改 1/0 即可开启/关闭陀螺仪增稳
 * ============================================================ */
#define ENABLE_GYRO_STABILIZER  0

#include "ti_msp_dl_config.h"
#include "modules/motor/step_motor.h"
#include "modules/tracker/uart_tracker.h"

#if ENABLE_GYRO_STABILIZER
#include "modules/sensor/mpu6050.h"
#include "modules/stabilizer/stabilizer.h"
#include "modules/debug/debug.h"
#endif

#if ENABLE_GYRO_STABILIZER
/* ============================================================
 *  看门狗 (TODO: 确认 SDK API 后启用)
 * ============================================================ */

/*
 * 配置独立看门狗 (IWDT/WWDT), 超时约 100ms.
 * 若主循环卡死 (I2C 总线锁死 / 传感器故障 / 阻塞式延时),
 * 看门狗复位系统, 防止电机在未知状态下持续运行.
 *
 * API 依赖 MSPM0 DriverLib 版本:
 *   DL_SYSCTL_enableWWD()       → 使能窗口看门狗
 *   DL_SYSCTL_resetWWD()        → 喂狗
 *   若 SDK 使用 DL_WWDT_* API, 替换为 DL_WWDT_init/WWDT0/DL_WWDT_enable 等
 */
static void watchdog_init(void)
{
    /*
     * MSPM0G3507 WWDT 通过 SYSCTL 模块配置.
     * 超时周期选择: ~100ms (在 200Hz 主循环中有 ~20 次机会喂狗).
     */
#if 0
    /* TODO: 根据你的 MSPM0 SDK 版本启用以下代码块 */
    DL_SYSCTL_setWWDPeriod(DL_SYSCTL_WWD_PERIOD_100MS);
    DL_SYSCTL_enableWWD();
#endif
}

static void watchdog_kick(void)
{
#if 0
    DL_SYSCTL_resetWWD();
#endif
}
#endif /* ENABLE_GYRO_STABILIZER */

/* ============================================================
 *  主函数
 * ============================================================ */

int main(void)
{
    /* ===== 1. 平台初始化 ===== */
    SYSCFG_DL_init();

    /*
     * 关闭 SysConfig 默认打开的 UART 回环模式 (Loopback Mode).
     *
     * 回环模式是做什么的:
     *   TX 和 RX 在芯片内部短路 —— 自己发的数据自己收, 外界信号断开.
     *   这是出厂自检用的, 正常工作必须关掉, 否则收不到 K230 发来的坐标.
     *
     * 为什么要先禁能再操作:
     *   LBE 位在 UART 运行时处于锁存状态, 直接写可能不生效,
     *   先禁能 → 改寄存器 → 重新使能 是最可靠的做法.
     */

    /* ① 暂停 UART 模块, 改配置前必须先停 */
    DL_UART_Main_disable(UART_0_INST);

    /* ② 直接操作寄存器清除 LBE (Loop Back Enable) 位, 关掉内部回环 */
    UART_0_INST->CTL0 &= ~UART_CTL0_LBE_MASK;

    /* ③ 用 TI 驱动库 API 正式禁用回环模式 (清相关状态位) */
    DL_UART_Main_disableLoopbackMode(UART_0_INST);

    /* ④ UART 重新上电运行, 此时 TX/RX 已接到外部引脚 */
    DL_UART_Main_enable(UART_0_INST);

    /* ⑤ TX 引脚默认可能是高阻输入态, 显式设为输出才能往外推信号 */
    DL_GPIO_enableOutput(GPIO_UART_0_TX_PORT, GPIO_UART_0_TX_PIN);

    step_motor_init();
#if ENABLE_GYRO_STABILIZER
    debug_init();
    watchdog_init();
#endif

#if ENABLE_GYRO_STABILIZER
    /* ===== 2. MPU6050 初始化 ===== */
    if (!mpu6050_init()) {
        /* MPU6050 初始化失败, 双电机交替闪烁提示, 死循环等待复位 */
        while (1) {
            step_motor_enable(STEP_MOTOR_A, 1);
            step_motor_enable(STEP_MOTOR_B, 1);
            step_motor_move(STEP_MOTOR_A, STEP_DIR_CW,  10, 200);
            step_motor_move(STEP_MOTOR_A, STEP_DIR_CCW, 10, 200);
            step_motor_move(STEP_MOTOR_B, STEP_DIR_CW,  10, 200);
            step_motor_move(STEP_MOTOR_B, STEP_DIR_CCW, 10, 200);
            step_motor_enable(STEP_MOTOR_A, 0);
            step_motor_enable(STEP_MOTOR_B, 0);
            delay_us(500000);
        }
    }

    /* ===== 3. 陀螺零偏校准 (跳过, 偏置已在 init 中清零) ===== */
#if 0
    if (!mpu6050_calibrate()) {
        /*
         * 校准失败: 有效采样不足或传感器在校准期间移动.
         * 闪烁错误码: 快速交替使能电机 A/B 表示校准失败.
         */
        for (int i = 0; i < 5; i++) {
            step_motor_enable(STEP_MOTOR_A, 1);
            delay_us(100000);
            step_motor_enable(STEP_MOTOR_A, 0);
            step_motor_enable(STEP_MOTOR_B, 1);
            delay_us(100000);
            step_motor_enable(STEP_MOTOR_B, 0);
        }

        /*
         * Fallback: 校准失败也确保零偏不为零.
         * 对陀螺做纯均值采样 (无方差校验), 传感器需保持静止.
         * 最多 200 次尝试, 收集 50 个有效样本.
         */
        {
            float sgx = 0, sgy = 0, sgz = 0;
            int   n = 0, target = 50;
            for (int i = 0; i < 200 && n < target; i++) {
                float p, r;
                mpu6050_get_angles(&p, &r);
                if (mpu6050_get_last_data_valid()) {
                    float rx, ry, rz;
                    mpu6050_get_last_raw(NULL, NULL, NULL, &rx, &ry, &rz);
                    sgx += rx; sgy += ry; sgz += rz;
                    n++;
                }
                delay_us(5000);
            }
            if (n > 0) {
                mpu6050_set_gyro_biases(
                    sgx / n * MPU6050_GYRO_SENS,
                    sgy / n * MPU6050_GYRO_SENS,
                    sgz / n * MPU6050_GYRO_SENS);
            }
        }
    }
#endif
#endif /* ENABLE_GYRO_STABILIZER */

#if ENABLE_GYRO_STABILIZER
    /* ===== 4. 增稳器初始化 ===== */
    Stabilizer stab;
    stabilizer_init(&stab);
#endif

    /* ===== 5. 电机测试: 验证硬件正常 ===== */
    /*step_motor_enable(STEP_MOTOR_A, 1);
    step_motor_enable(STEP_MOTOR_B, 1);
    step_motor_move(STEP_MOTOR_A, STEP_DIR_CW,  200, 800);
    step_motor_move(STEP_MOTOR_A, STEP_DIR_CCW, 200, 800);
    step_motor_move(STEP_MOTOR_B, STEP_DIR_CW,  200, 800);
    step_motor_move(STEP_MOTOR_B, STEP_DIR_CCW, 200, 800);
    step_motor_enable(STEP_MOTOR_A, 0);
    step_motor_enable(STEP_MOTOR_B, 0);*/
    /* 测试完成后 EN 拉低, tracker_init 会重新使能 */

    /* ===== 6. 跟踪模块初始化 ===== */
    {
        TrackerConfig cfg = TRACKER_DEFAULT_CONFIG;
        tracker_init(&cfg);
    }

#if ENABLE_GYRO_STABILIZER
    /*
     * ===== 6. 互补滤波预热 =====
     * 校准后 g_pitch_angle=0, 互补滤波 τ≈95ms (α=0.95), 需约 0.3s (60 次)
     * 即可 95% 收敛. 100 次 ≈ 0.5s, 足够平滑启动.
     * 预热仅服务于角度外环; 速度内环无需预热 (直接用陀螺角速度).
     */
    for (int i = 0; i < 100; i++) {
        float pitch, roll;
        mpu6050_get_angles(&pitch, &roll);
        delay_us(5000);  /* 5ms, 模拟慢路径周期 */
    }
    stabilizer_set_enabled(&stab, true);
    /* 同时使能电机 EN + 角度环 + 速度环 (setpoint=0, 主动静止保持) */
#endif

    /* ===== 7. 主循环 ===== */
    {
        bool data_confirmed = false;

        while (1) {
            tracker_update();

            /* 诊断: 首次收到有效坐标时, 电机 A 短抖 50 步作为确认 */
            if (!data_confirmed) {
                TrackerCoord c = tracker_get_coord();
                if (c.fresh) {
                    /*data_confirmed = true;
                    /*step_motor_move(STEP_MOTOR_A, STEP_DIR_CW,  50, 400);
                    step_motor_move(STEP_MOTOR_A, STEP_DIR_CCW, 50, 400);*/
                    /* 抖动后电机 A 回到 tracker 控制 */
                }
            }

#if ENABLE_GYRO_STABILIZER
            watchdog_kick();
            g_debug_loop_count++;
            stabilizer_update(&stab);
            debug_heartbeat();
#endif
            /*
             * 主循环节拍: 10µs 延时防止裸奔占满 CPU.
             * tracker_update() 实际耗时约 20~50µs (UART 轮询 + PWM 更新 + 回传),
             * 加上此延时, 每圈约 30~60µs → 实际循环频率约 15~30kHz.
             * 串口 115200 baud 每帧约 2.6ms, 循环远快于帧率, 不会丢数据.
             */
            delay_us(10);
        }
    }
}
