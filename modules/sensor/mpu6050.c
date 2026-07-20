/*
 * mpu6050.c — MPU6050 陀螺仪驱动实现
 *
 * 硬件: I2C0 (PA10=SDA, PA11=SCL), MPU6050 地址 0x68
 * 依赖: ti_msp_dl_config.h (I2C), mpu6050.h
 * 设计要点:
 *   1. I2C 逐字节读取 (MSPM0 硅级 bug 规避 + 克隆版限制)
 *   2. I2C 硬件复位 + Controller-Only 模式 (规避双模 RX 数据通路损坏)
 *   3. 互补滤波: α=0.95 (τ≈95ms), 陀螺主导+加速度计快速修正
 *   4. 双缓冲 GyroSample: 原子提交, 防止并发撕裂读
 *   5. 陀螺一阶 IIR 低通滤波 (fc=50Hz): 抑制速度环 D 项噪声
 *   6. 在线零偏自适应校准: EMA 静止检测 + 缓慢修正 (τ≈10s)
 *
 * 已知限制:
 *   1. mpu6050_read_regs() 逐字节读取, 12 字节需 24 次独立 I2C 事务
 *   2. 关闭时钟拉伸 (DL_I2C_disableControllerClockStretching) 已注释
 *   3. MPU6050 克隆版不支持多字节连续读, 校准/读取均为逐字节
 */
#include "ti_msp_dl_config.h"

#include "modules/sensor/mpu6050.h"

#include <math.h>

/* ============================================================
 *  I2C 调试结构体 (在 CCS 中查看 g_i2c_dbg)
 * ============================================================ */
typedef struct {
    uint32_t fail_step;       /* 1=Step1(写寄存器地址)失败, 2=Step2(读数据)失败 */
    uint32_t fail_reason;     /* 0=未失败, 1=超时, 2=状态错误 */
    uint32_t status_value;    /* 失败时的 DL_I2C_getControllerStatus() 返回值 */
    uint32_t stat_raw;        /* 失败时的 I2C0.STAT 原始寄存器值 */
    uint32_t addr_nack;       /* ADDR_ACK 位 (1=NACK) */
    uint32_t data_nack;       /* DATA_ACK 位 (1=NACK) */
    uint32_t error_bit;       /* ERROR 位 */
    uint32_t idle_bit;        /* IDLE 位 */
    uint32_t total_fails;     /* 累计读失败次数 */
    uint32_t last_reg;        /* 最后一次尝试读取的寄存器地址 */
    uint32_t last_len;        /* 最后一次尝试读取的字节数 */
} I2CDebug;

volatile I2CDebug g_i2c_dbg;
volatile uint32_t g_whoami_value;     /* WHO_AM_I 实际读到的值 */
volatile uint32_t g_pwrmgmt_readback; /* PWR_MGMT1 读回值 (应为 0x01) */
volatile uint32_t g_bitbang_whoami;   /* 软件 I2C 读 WHO_AM_I 结果 (0=未执行/失败) */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 寄存器地址 */
#define MPU6050_ADDR            0x68
#define REG_WHO_AM_I            0x75
#define REG_PWR_MGMT1           0x6B
#define REG_SMPLRT_DIV          0x19
#define REG_CONFIG              0x1A
#define REG_GYRO_CONFIG         0x1B
#define REG_ACCEL_CONFIG        0x1C
#define REG_ACCEL_XOUT_H        0x3B
#define REG_ACCEL_YOUT_H        0x3D
#define REG_ACCEL_ZOUT_H        0x3F
#define REG_GYRO_XOUT_H         0x43
#define REG_GYRO_YOUT_H         0x45
#define REG_GYRO_ZOUT_H         0x47

/* 校准采样数 */
#define CALIB_SAMPLES           200
/* 校准要求最低有效采样数 (放宽以适应 I2C 间歇性 NACK) */
#define CALIB_MIN_SAMPLES       60
/* 校准期间加速度计方差阈值 (放宽以适应微小振动) */
#define CALIB_ACCEL_VAR_MAX     0.05f

/* I2C 超时 (循环次数) */
#define I2C_TIMEOUT             100000

/* 传感器数据校验阈值 */
#define ACCEL_MAG_MIN           0.5f    /* g, 加速度计合成量最小值 */
#define ACCEL_MAG_MAX           3.0f    /* g, 加速/减速时允许更大的合成量 (原2.0) */
#define GYRO_RANGE_MAX          300.0f  /* °/s, 超过 ±250°/s 量程的合理上限 */

/* ============================================================
 *  状态变量
 * ============================================================ */
static float g_pitch_angle;     /* 俯仰角 (度) */
static float g_roll_angle;      /* 横滚角 (度) */

static float g_gyro_bias_x;     /* 陀螺零偏 X (LSB) */
static float g_gyro_bias_y;     /* 陀螺零偏 Y (LSB) */
static float g_gyro_bias_z;     /* 陀螺零偏 Z (LSB, 未使用) */

static uint32_t g_i2c_error_count;  /* 连续 I2C 失败次数 (成功读取后清零) */

/* 加速度计原始值 (仅 get_angles 更新, 无并发问题, 单缓冲即可) */
static float g_last_accel_x, g_last_accel_y, g_last_accel_z;
static bool  g_last_data_valid = false;   /* 兼容旧接口 */

/* ============================================================
 *  双缓冲陀螺数据结构
 * ============================================================ */
static GyroSample g_gyro_buf[2];       /* 双缓冲 */
static volatile uint8_t g_gyro_wr = 0; /* 写入索引 (0 或 1) */

/* ============================================================
 *  低通滤波状态 (递推, 跨周期保持)
 * ============================================================ */
static float g_lpf_state_x = 0.0f;     /* X 轴低通滤波上一周期输出 */
static float g_lpf_state_y = 0.0f;     /* Y 轴低通滤波上一周期输出 */

/* ============================================================
 *  在线自适应零偏校准状态
 * ============================================================ */
static float g_gyro_ema_x = 0.0f;      /* 去零偏角速度 EMA (τ≈1s) */
static float g_gyro_ema_y = 0.0f;      /* 用于静止检测 */

/* ============================================================
 *  I2C 底层读写
 * ============================================================ */

/*
 * I2C 状态位说明 (MSPM0G CTRLSTATUS 寄存器):
 *   DL_I2C_CONTROLLER_STATUS_ADDR_ACK → 对应 TRM 的 TX_ADR_NACK 位
 *   DL_I2C_CONTROLLER_STATUS_DATA_ACK → 对应 TRM 的 TX_DAT_NACK 位
 * 虽然 DriverLib 命名含 "ACK", 但实际比特位语义:
 *   1 = NACK (从机未应答) = 错误
 *   0 = ACK  (从机已应答) = 正常
 * 因此代码中将这些位作为错误标志检查是正确的.
 */

/* 写单个寄存器: start + dev(W) + reg + data + stop */
static bool mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, buf, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    /* 等待传输完成 */
    uint32_t timeout = I2C_TIMEOUT;
    uint32_t status;
    do {
        status = DL_I2C_getControllerStatus(I2C_0_INST);
        if (--timeout == 0) return false;
    } while ((status & DL_I2C_CONTROLLER_STATUS_IDLE) == 0);

    /* ADDR_ACK/DATA_ACK bit = 1 表示 NACK (见文件头注释) */
    if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                  DL_I2C_CONTROLLER_STATUS_ADDR_ACK  |
                  DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
        return false;
    }
    return true;
}

/*
 * 读寄存器 — 极简版: 只用 Advanced API, 只等 IDLE, 不轮询 FIFO.
 *   因为 MSPM0 I2C RX + 这个 MPU6050 模块在 FIFO 轮询和
 *   DL_I2C_startControllerTransfer(RX) 下均随机卡死 BUSY.
 *   只能退回到最接近 TX (已经证明 100% 可靠) 的方式:
 *   每个字节独立 Advanced API 事务 + 等 IDLE. */
static bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    for (uint8_t n = 0; n < len; n++) {
        uint32_t status;
        uint32_t timeout;

        g_i2c_dbg.last_reg = reg + n;
        g_i2c_dbg.last_len = 1;

        /*
         * Step 1: 写当前寄存器地址 (TX, Advanced API, START+STOP)
         * 完全复制 mpu6050_write_reg 的成功模式, 只发 1 字节.
         */
        DL_I2C_flushControllerTXFIFO(I2C_0_INST);
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
        DL_I2C_startControllerTransferAdvanced(I2C_0_INST, MPU6050_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_TX, 1,
            DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_ENABLE);

        timeout = I2C_TIMEOUT;
        do {
            status = DL_I2C_getControllerStatus(I2C_0_INST);
            if (--timeout == 0) {
                g_i2c_dbg.fail_step = 1; g_i2c_dbg.fail_reason = 1;
                g_i2c_dbg.total_fails++; return false;
            }
        } while (!(status & DL_I2C_CONTROLLER_STATUS_IDLE));

        if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                      DL_I2C_CONTROLLER_STATUS_ADDR_ACK |
                      DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
            g_i2c_dbg.fail_step = 1; g_i2c_dbg.fail_reason = 2;
            g_i2c_dbg.status_value = status; g_i2c_dbg.total_fails++;
            return false;
        }

        /* 寄存器地址自增 (下一个字节) */
        reg++;

        /*
         * Step 2: 读 1 字节 (RX, Advanced API, START+STOP, ACK_DISABLE)
         * ACK_DISABLE → 1 字节读完后立即 NACK → 从机释放总线 → STOP.
         */
        DL_I2C_startControllerTransferAdvanced(I2C_0_INST, MPU6050_ADDR,
            DL_I2C_CONTROLLER_DIRECTION_RX, 1,
            DL_I2C_CONTROLLER_START_ENABLE,
            DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);

        timeout = I2C_TIMEOUT;
        do {
            status = DL_I2C_getControllerStatus(I2C_0_INST);
            if (--timeout == 0) {
                g_i2c_dbg.fail_step = 2; g_i2c_dbg.fail_reason = 1;
                g_i2c_dbg.total_fails++; return false;
            }
        } while (!(status & DL_I2C_CONTROLLER_STATUS_IDLE));

        if (status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                      DL_I2C_CONTROLLER_STATUS_ADDR_ACK |
                      DL_I2C_CONTROLLER_STATUS_DATA_ACK)) {
            g_i2c_dbg.fail_step = 2; g_i2c_dbg.fail_reason = 2;
            g_i2c_dbg.status_value = status; g_i2c_dbg.total_fails++;
            return false;
        }

        buf[n] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    return true;
}

/* ============================================================
 *  传感器数据校验
 * ============================================================ */

/*
 * 校验加速度计和陀螺仪原始数据是否在合理范围内.
 * 返回 true 表示数据可信, false 表示数据异常 (传感器故障/总线噪声).
 */
static bool mpu6050_validate_sensor_data(
    int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
    int16_t gx_raw, int16_t gy_raw, int16_t gz_raw)
{
    float ax = (float)ax_raw / MPU6050_ACCEL_SENS;
    float ay = (float)ay_raw / MPU6050_ACCEL_SENS;
    float az = (float)az_raw / MPU6050_ACCEL_SENS;

    /* 加速度计合成量应在 0.5g ~ 2.0g 之间 (静止时 ≈1g) */
    float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_mag < ACCEL_MAG_MIN || accel_mag > ACCEL_MAG_MAX) {
        return false;
    }

    /* 陀螺值应在 ±300°/s 以内 (±250°/s 量程 + 余量) */
    float gx = (float)gx_raw / MPU6050_GYRO_SENS;
    float gy = (float)gy_raw / MPU6050_GYRO_SENS;
    float gz = (float)gz_raw / MPU6050_GYRO_SENS;
    if (fabsf(gx) > GYRO_RANGE_MAX || fabsf(gy) > GYRO_RANGE_MAX
        || fabsf(gz) > GYRO_RANGE_MAX) {
        return false;
    }

    return true;
}

/* ============================================================
 *  软件 I2C (bit-banging) 诊断
 *
 *  绕开 MSPM0 I2C 硬件模块, 用 GPIO 直接控制 SDA/SCL 引脚.
 *    若此函数读到正确值 → MSPM0 I2C 模块有硅级 bug
 *    若此函数也读到 0x00   → MPU6050 芯片/总线有问题
 * ============================================================ */
#define BB_SCL_PORT  GPIOA
#define BB_SCL_PIN   DL_GPIO_PIN_11   /* PA11 = SCL */
#define BB_SDA_PORT  GPIOA
#define BB_SDA_PIN   DL_GPIO_PIN_10   /* PA10 = SDA */

/* IOMUX PINCM 寄存器: PA10=PINCM21, PA11=PINCM22 */
#define PINCM_SDA    0x40408054UL
#define PINCM_SCL    0x40408058UL

/* GPIO 模式: PF[3:0]=1, PC[4]=1(输入使能) */
#define PINCM_GPIO   0x00000011UL

static void bb_delay(void) {
    for (volatile int i = 0; i < 120; i++) { __asm("nop"); }  /* ~3.75us */
}
static void bb_scl_h(void) { DL_GPIO_setPins(BB_SCL_PORT, BB_SCL_PIN);    bb_delay(); }
static void bb_scl_l(void) { DL_GPIO_clearPins(BB_SCL_PORT, BB_SCL_PIN);  bb_delay(); }
static void bb_sda_h(void) { DL_GPIO_setPins(BB_SCL_PORT, BB_SCL_PIN);    /* SDA high = set output HIGH (float via pull-up if open-drain) */ }
/* Actually, for SDA we need to switch between output LOW and input (float HIGH).
 * Use direct register manipulation for SDA. */
static void bb_sda_1(void) { DL_GPIO_setPins(BB_SDA_PORT, BB_SDA_PIN); }   /* output HIGH */
static void bb_sda_0(void) { DL_GPIO_clearPins(BB_SDA_PORT, BB_SDA_PIN); } /* output LOW */
static int  bb_sda_in(void){ return DL_GPIO_readPins(BB_SDA_PORT, BB_SDA_PIN) ? 1 : 0; }

static void bb_start(void) {
    bb_sda_1(); bb_scl_h();
    bb_sda_0(); bb_scl_l();
}
static void bb_stop(void) {
    bb_sda_0(); bb_scl_h();
    bb_sda_1();
}
static int bb_tx(uint8_t d) {
    for (int i = 7; i >= 0; i--) {
        if (d & (1 << i)) bb_sda_1(); else bb_sda_0();
        bb_scl_h(); bb_scl_l();
    }
    bb_sda_1(); bb_scl_h();
    int ack = bb_sda_in();
    bb_scl_l();
    return ack;  /* 0=ACK */
}
static uint8_t bb_rx(int last) {
    uint8_t d = 0;
    bb_sda_1();
    for (int i = 7; i >= 0; i--) {
        bb_scl_h();
        if (bb_sda_in()) d |= (1 << i);
        bb_scl_l();
    }
    if (last) bb_sda_1(); else bb_sda_0();
    bb_scl_h(); bb_scl_l();
    bb_sda_1();
    return d;
}

static void bitbang_read_reg(uint8_t reg)
{
    uint32_t save_sda, save_scl;
    int ack;

    /* 保存 IOMUX */
    save_sda = *(volatile uint32_t *)PINCM_SDA;
    save_scl = *(volatile uint32_t *)PINCM_SCL;

    /* 切换为 GPIO 模式 */
    *(volatile uint32_t *)PINCM_SDA = PINCM_GPIO;
    *(volatile uint32_t *)PINCM_SCL = PINCM_GPIO;

    /* 初始: SCL=SDA=HIGH (空闲) */
    DL_GPIO_setPins(BB_SCL_PORT, BB_SCL_PIN);
    DL_GPIO_setPins(BB_SDA_PORT, BB_SDA_PIN);
    for (volatile int i = 0; i < 320; i++) { __asm("nop"); }  /* ~10us */

    /* 写寄存器地址 */
    bb_start();
    ack = bb_tx(0xD0);
    if (ack) { g_bitbang_whoami = 0xFE; goto out; }
    ack = bb_tx(reg);
    if (ack) { g_bitbang_whoami = 0xFD; goto out; }
    bb_stop();
    for (volatile int i = 0; i < 960; i++) { __asm("nop"); }

    /* 读数据 */
    bb_start();
    ack = bb_tx(0xD1);
    if (ack) { g_bitbang_whoami = 0xFC; goto out; }
    g_bitbang_whoami = bb_rx(1);  /* last byte → NACK */
    bb_stop();

out:
    /* 恢复 IOMUX */
    *(volatile uint32_t *)PINCM_SDA = save_sda;
    *(volatile uint32_t *)PINCM_SCL = save_scl;
}

/* ============================================================
 *  初始化
 * ============================================================ */

bool mpu6050_init(void)
{
    /*
     * 轻量 I2C 复位: 不碰 Target (SysConfig 已配 Controller-Only),
     * 只保证每次上电/复位后 I2C 状态机干净.
     */
    {
        DL_I2C_ClockConfig clk_cfg;
        DL_I2C_getClockConfig(I2C_0_INST, &clk_cfg);
        DL_I2C_reset(I2C_0_INST);
        DL_I2C_enablePower(I2C_0_INST);
        delay_cycles(16);
        DL_I2C_setClockConfig(I2C_0_INST, &clk_cfg);
        DL_I2C_setTimerPeriod(I2C_0_INST, 15);  /* ~200kHz */
        DL_I2C_disableControllerClockStretching(I2C_0_INST);
        DL_I2C_enableController(I2C_0_INST);
    }

    /* MPU6050 热复位后内部 PLL 需要稳定时间, 等 50ms */
    for (volatile int i = 0; i < 400000; i++) { __asm("nop"); }  /* ~50ms */

    uint8_t whoami = 0;
    for (int retry = 0; retry < 10; retry++) {
        if (mpu6050_read_regs(REG_WHO_AM_I, &whoami, 1)) {
            g_whoami_value = whoami;
            if (whoami == 0x68 || whoami == 0x70) break;
        }
        for (volatile int i = 0; i < 80000; i++) { __asm("nop"); }  /* ~10ms */
    }
    if (whoami != 0x68 && whoami != 0x70) return false;

    /* 强制偏置清零 — 跳过校准, 直接零偏置 */
    g_gyro_bias_x = 0.0f; g_gyro_bias_y = 0.0f; g_gyro_bias_z = 0.0f;
    g_gyro_ema_x = 0.0f; g_gyro_ema_y = 0.0f;

    /* 2. 唤醒 + 选择 PLL 时钟源 (CLKSEL=1, X轴陀螺参考) */
    if (!mpu6050_write_reg(REG_PWR_MGMT1, 0x01)) return false;

    /* 等待稳定 */
    for (volatile int i = 0; i < 320000; i++) { __asm("nop"); }  /* ~40ms */

    /* 2b. 读回 PWR_MGMT1 验证 I2C 读功能 (延时后, MPU6050 已稳定) */
    {
        uint8_t pwrmgmt = 0;
        if (mpu6050_read_regs(REG_PWR_MGMT1, &pwrmgmt, 1)) {
            g_pwrmgmt_readback = pwrmgmt;
        } else {
            g_pwrmgmt_readback = 0xFF;  /* 读失败标志 */
        }
    }

    /* 3. 采样率分频: 0 → 1kHz (陀螺), DLPF 开启后自动匹配 */
    if (!mpu6050_write_reg(REG_SMPLRT_DIV, 0x00)) return false;

    /* 4. DLPF: BW=44Hz (accel+gyro), 1kHz internal sample rate */
    if (!mpu6050_write_reg(REG_CONFIG, 0x03)) return false;

    /* 5. 陀螺量程: ±250°/s */
    if (!mpu6050_write_reg(REG_GYRO_CONFIG, 0x00)) return false;

    /* 6. 加速度计量程: ±2g */
    if (!mpu6050_write_reg(REG_ACCEL_CONFIG, 0x00)) return false;

    return true;
}

/* ============================================================
 *  传感器读取辅助
 * ============================================================ */

/*
 * 读取单个 16 位寄存器 (MSB 先, LSB 后).
 * MPU6050 克隆版不支持多字节连续读, 每个字节独立 I2C 事务.
 * 返回 false 表示 I2C 读取失败.
 */
static bool mpu6050_read_i16(uint8_t reg, int16_t *out)
{
    uint8_t hi, lo;
    if (!mpu6050_read_regs(reg, &hi, 1)) return false;
    if (!mpu6050_read_regs(reg + 1, &lo, 1)) return false;
    *out = (int16_t)((hi << 8) | lo);
    return true;
}

/* ============================================================
 *  陀螺零偏校准
 * ============================================================ */

/*
 * 静态校准: 采集 N 次陀螺数据求零偏.
 * 返回 true 表示校准成功, false 表示采样不足或传感器在移动.
 * 调用时需保持传感器完全静止!
 */
bool mpu6050_calibrate(void)
{
    float    sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float    sum_ax = 0, sum_ay = 0, sum_az = 0;
    float    sum_ax2 = 0, sum_ay2 = 0, sum_az2 = 0;
    uint32_t valid_samples = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        /* 逐寄存器单字节读 — MPU6050 克隆版不支持多字节连续读 */
        int16_t ax, ay, az, gx, gy;
        if (!mpu6050_read_i16(REG_ACCEL_XOUT_H, &ax)) continue;
        if (!mpu6050_read_i16(REG_ACCEL_YOUT_H, &ay)) continue;
        if (!mpu6050_read_i16(REG_ACCEL_ZOUT_H, &az)) continue;
        if (!mpu6050_read_i16(REG_GYRO_XOUT_H, &gx)) continue;
        if (!mpu6050_read_i16(REG_GYRO_YOUT_H, &gy)) continue;
        /* 跳过 GYRO_ZOUT: Z轴不再使用 */

        /* 累积加速度计数据用于方差校验 */
        float fax = (float)ax / MPU6050_ACCEL_SENS;
        float fay = (float)ay / MPU6050_ACCEL_SENS;
        float faz = (float)az / MPU6050_ACCEL_SENS;
        sum_ax  += fax;
        sum_ay  += fay;
        sum_az  += faz;
        sum_ax2 += fax * fax;
        sum_ay2 += fay * fay;
        sum_az2 += faz * faz;

        sum_gx += (float)gx;
        sum_gy += (float)gy;
        sum_gz += 0.0f;  /* Z轴不再使用 */
        valid_samples++;

        /* 简单延迟 ~1ms */
        for (volatile int j = 0; j < 32000; j++) { __asm("nop"); }
    }

    /* 校验 1: 最低有效采样数 */
    if (valid_samples < CALIB_MIN_SAMPLES) {
        g_gyro_bias_x = 0.0f;
        g_gyro_bias_y = 0.0f;
        g_gyro_bias_z = 0.0f;
        return false;
    }

    /* 校验 2: 加速度计方差 — 若传感器在校准期间移动, 拒绝校准 */
    float mean_ax  = sum_ax  / (float)valid_samples;
    float mean_ay  = sum_ay  / (float)valid_samples;
    float mean_az  = sum_az  / (float)valid_samples;
    float var_ax   = sum_ax2 / (float)valid_samples - mean_ax  * mean_ax;
    float var_ay   = sum_ay2 / (float)valid_samples - mean_ay  * mean_ay;
    float var_az   = sum_az2 / (float)valid_samples - mean_az  * mean_az;
    float var_total = var_ax + var_ay + var_az;

    if (var_total > CALIB_ACCEL_VAR_MAX) {
        /* 传感器在校准期间移动 — 零偏不可信 */
        g_gyro_bias_x = 0.0f;
        g_gyro_bias_y = 0.0f;
        g_gyro_bias_z = 0.0f;
        return false;
    }

    /* 计算零偏均值 */
    g_gyro_bias_x = sum_gx / (float)valid_samples;
    g_gyro_bias_y = sum_gy / (float)valid_samples;
    g_gyro_bias_z = sum_gz / (float)valid_samples;

    /* 清零角度初始值 */
    g_pitch_angle = 0.0f;
    g_roll_angle  = 0.0f;

    return true;
}

/* ============================================================
 *  陀螺数据校验 (仅角速度, 不含加速度计)
 * ============================================================ */

/*
 * 校验陀螺仪原始数据是否在合理范围内.
 * 返回 true 表示数据可信.
 */
static bool mpu6050_validate_gyro_data(int16_t gx_raw, int16_t gy_raw)
{
    float gx = (float)gx_raw / MPU6050_GYRO_SENS;
    float gy = (float)gy_raw / MPU6050_GYRO_SENS;
    if (fabsf(gx) > GYRO_RANGE_MAX || fabsf(gy) > GYRO_RANGE_MAX) {
        return false;
    }
    return true;
}

/* ============================================================
 *  双缓冲辅助: 提交陀螺数据到写入缓冲, 翻转索引
 * ============================================================ */

/*
 * 将处理完成的陀螺数据提交到双缓冲.
 * 调用前: 已填充 g_gyro_buf[g_gyro_wr] 的所有字段.
 * 调用后: g_gyro_wr 翻转, 新数据对读者可见.
 */
static void gyro_commit_buffer(void)
{
    g_gyro_wr ^= 1;  /* 单字节, Cortex-M0+ 上原子 */
}

/* ============================================================
 *  在线自适应零偏校准
 * ============================================================ */

/*
 * 每个快周期调用一次 (在去零偏之后, 滤波之前).
 * 使用去零偏但未滤波的角速度做静止检测和零偏修正.
 * 双时间常数: EMA τ≈1s (静止检测), 零偏修正 τ≈10s.
 */
static void mpu6050_adaptive_bias_update(float debiased_x, float debiased_y)
{
    /* NaN/Inf 防护 */
    if (isnan(debiased_x) || isinf(debiased_x) ||
        isnan(debiased_y) || isinf(debiased_y)) {
        return;
    }

    /* 更新去零偏角速度的慢速 EMA (τ≈1s) */
    g_gyro_ema_x = MPU6050_BIAS_EMA_ALPHA * debiased_x
                 + (1.0f - MPU6050_BIAS_EMA_ALPHA) * g_gyro_ema_x;
    g_gyro_ema_y = MPU6050_BIAS_EMA_ALPHA * debiased_y
                 + (1.0f - MPU6050_BIAS_EMA_ALPHA) * g_gyro_ema_y;

    /* 自适应零偏校准已禁用. 同时硬钳 EMA 到 0, 防止旧偏置残留. */
    g_gyro_ema_x = 0.0f;
    g_gyro_ema_y = 0.0f;

    /* 钳制零偏范围 */
    if (g_gyro_bias_x >  MPU6050_BIAS_MAX_LSB) g_gyro_bias_x =  MPU6050_BIAS_MAX_LSB;
    if (g_gyro_bias_x < -MPU6050_BIAS_MAX_LSB) g_gyro_bias_x = -MPU6050_BIAS_MAX_LSB;
    if (g_gyro_bias_y >  MPU6050_BIAS_MAX_LSB) g_gyro_bias_y =  MPU6050_BIAS_MAX_LSB;
    if (g_gyro_bias_y < -MPU6050_BIAS_MAX_LSB) g_gyro_bias_y = -MPU6050_BIAS_MAX_LSB;
}

/* ============================================================
 *  快速陀螺读取 (快路径, ~300us)
 * ============================================================ */

void mpu6050_get_gyro_rates(float *gx, float *gy)
{
    int16_t  gx_raw, gy_raw;
    int      retry;

    /* 只读陀螺 X, Y (4 字节), 跳过加速度计 */
    for (retry = 0; retry < 3; retry++) {
        if (!mpu6050_read_i16(REG_GYRO_XOUT_H, &gx_raw)) continue;
        if (!mpu6050_read_i16(REG_GYRO_YOUT_H, &gy_raw)) continue;
        break;
    }
    if (retry == 3) {
        g_i2c_error_count++;
        /* 返回已提交缓冲区的滤波后角速度 (上次有效值) */
        GyroSample *rd = &g_gyro_buf[g_gyro_wr ^ 1];
        *gx = rd->filtered_x;
        *gy = rd->filtered_y;
        return;
    }

    /* 陀螺数据校验 */
    if (!mpu6050_validate_gyro_data(gx_raw, gy_raw)) {
        GyroSample *rd = &g_gyro_buf[g_gyro_wr ^ 1];
        *gx = rd->filtered_x;
        *gy = rd->filtered_y;
        return;
    }

    /* 读取成功, 清零连续错误计数 */
    g_i2c_error_count = 0;

    /* 转物理单位 (去零偏前) */
    float rgx = (float)gx_raw / MPU6050_GYRO_SENS;
    float rgy = (float)gy_raw / MPU6050_GYRO_SENS;

    /* 写入缓冲: 原始值 */
    GyroSample *wr = &g_gyro_buf[g_gyro_wr];
    wr->raw_x = rgx;
    wr->raw_y = rgy;

    /* 去零偏 */
    float db_x = rgx - g_gyro_bias_x / MPU6050_GYRO_SENS;
    float db_y = rgy - g_gyro_bias_y / MPU6050_GYRO_SENS;
    wr->debiased_x = db_x;
    wr->debiased_y = db_y;

    /* 在线自适应零偏校准 (使用未滤波的去零偏值) */
    mpu6050_adaptive_bias_update(db_x, db_y);

    /* 一阶 IIR 低通滤波 (fc=50Hz) */
    float beta = MPU6050_GYRO_LPF_BETA;
    g_lpf_state_x = beta * db_x + (1.0f - beta) * g_lpf_state_x;
    g_lpf_state_y = beta * db_y + (1.0f - beta) * g_lpf_state_y;
    wr->filtered_x = g_lpf_state_x;
    wr->filtered_y = g_lpf_state_y;

    /* 采样有效 */
    wr->valid = true;

    /* 原子提交 */
    gyro_commit_buffer();

    /* 返回滤波后角速度 */
    *gx = wr->filtered_x;
    *gy = wr->filtered_y;
}

/* ============================================================
 *  读取已提交缓冲
 * ============================================================ */

void mpu6050_get_gyro_sample(GyroSample *out)
{
    *out = g_gyro_buf[g_gyro_wr ^ 1];
}

void mpu6050_get_debiased_gyro(float *gx, float *gy)
{
    GyroSample *rd = &g_gyro_buf[g_gyro_wr ^ 1];
    *gx = rd->debiased_x;
    *gy = rd->debiased_y;
}

/* ============================================================
 *  互补滤波 + 角度输出
 * ============================================================ */

void mpu6050_get_angles(float *pitch, float *roll)
{
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    int     retry;

    /*
     * 逐寄存器单字节读 — MPU6050 克隆版不支持多字节连续读.
     * 每个 16 位寄存器分两次读 (H 然后 L), 由 mpu6050_read_i16() 封装.
     */

    /* 加速度计 X, Y, Z */
    for (retry = 0; retry < 3; retry++) {
        if (!mpu6050_read_i16(REG_ACCEL_XOUT_H, &ax_raw)) continue;
        if (!mpu6050_read_i16(REG_ACCEL_YOUT_H, &ay_raw)) continue;
        if (!mpu6050_read_i16(REG_ACCEL_ZOUT_H, &az_raw)) continue;
        break;
    }
    if (retry == 3) {
        g_i2c_error_count++;
        g_last_data_valid = false;
        *pitch = g_pitch_angle;
        *roll  = g_roll_angle;
        return;
    }

    /* 陀螺 X, Y */
    for (retry = 0; retry < 3; retry++) {
        if (!mpu6050_read_i16(REG_GYRO_XOUT_H, &gx_raw)) continue;
        if (!mpu6050_read_i16(REG_GYRO_YOUT_H, &gy_raw)) continue;
        gz_raw = 0;  /* Z轴不再使用 */
        break;
    }
    if (retry == 3) {
        g_i2c_error_count++;
        g_last_data_valid = false;
        *pitch = g_pitch_angle;
        *roll  = g_roll_angle;
        return;
    }

    /* 传感器数据校验 */
    if (!mpu6050_validate_sensor_data(ax_raw, ay_raw, az_raw,
                                       gx_raw, gy_raw, gz_raw)) {
        g_last_data_valid = false;
        *pitch = g_pitch_angle;
        *roll  = g_roll_angle;
        return;
    }

    /* 读取成功且数据有效, 清零连续错误计数 */
    g_i2c_error_count = 0;
    g_last_data_valid = true;

    /* 保存加速度计原始值 (调试用) */
    float accel_x = (float)ax_raw / MPU6050_ACCEL_SENS;
    float accel_y = (float)ay_raw / MPU6050_ACCEL_SENS;
    float accel_z = (float)az_raw / MPU6050_ACCEL_SENS;
    g_last_accel_x = accel_x;
    g_last_accel_y = accel_y;
    g_last_accel_z = accel_z;

    /* 陀螺原始值 → 写入双缓冲 */
    GyroSample *wr = &g_gyro_buf[g_gyro_wr];
    wr->raw_x = (float)gx_raw / MPU6050_GYRO_SENS;
    wr->raw_y = (float)gy_raw / MPU6050_GYRO_SENS;

    /* 去零偏 (存入 debiased, 未滤波) */
    float db_x = wr->raw_x - g_gyro_bias_x / MPU6050_GYRO_SENS;
    float db_y = wr->raw_y - g_gyro_bias_y / MPU6050_GYRO_SENS;
    wr->debiased_x = db_x;
    wr->debiased_y = db_y;

    /* 在线自适应零偏校准 (使用未滤波的值) */
    mpu6050_adaptive_bias_update(db_x, db_y);

    /* 一阶 IIR 低通滤波 (fc=50Hz) */
    float beta = MPU6050_GYRO_LPF_BETA;
    g_lpf_state_x = beta * db_x + (1.0f - beta) * g_lpf_state_x;
    g_lpf_state_y = beta * db_y + (1.0f - beta) * g_lpf_state_y;
    wr->filtered_x = g_lpf_state_x;
    wr->filtered_y = g_lpf_state_y;
    wr->valid = true;

    /* 原子提交 — 速度环/调试从此缓冲读取 */
    gyro_commit_buffer();

    /* 加速度计角度 (度) */
    float accel_pitch = atan2f(accel_x,
        sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;
    float accel_roll  = atan2f(accel_y, accel_z) * 180.0f / M_PI;

    /* 互补滤波: α=0.95, dt=5ms → τ≈95ms.
     * 使用去零偏但未滤波的角速度 (低延迟积分). */
    const float alpha = MPU6050_ALPHA;
    const float dt    = MPU6050_DT_SLOW;

    /* 轴映射: pitch(绕Y轴) 用 gyro_y, roll(绕X轴) 用 gyro_x */
    g_pitch_angle = alpha * (g_pitch_angle + db_y * dt)
                    + (1.0f - alpha) * accel_pitch;
    g_roll_angle  = alpha * (g_roll_angle  + db_x * dt)
                    + (1.0f - alpha) * accel_roll;

    *pitch = g_pitch_angle;
    *roll  = g_roll_angle;
}

uint32_t mpu6050_get_error_count(void)
{
    return g_i2c_error_count;
}

/* ============================================================
 *  调试接口
 * ============================================================ */

void mpu6050_get_last_raw(
    float *accel_x, float *accel_y, float *accel_z,
    float *gyro_x,  float *gyro_y,  float *gyro_z)
{
    GyroSample *rd = &g_gyro_buf[g_gyro_wr ^ 1];
    *accel_x = g_last_accel_x;
    *accel_y = g_last_accel_y;
    *accel_z = g_last_accel_z;
    *gyro_x  = rd->raw_x;
    *gyro_y  = rd->raw_y;
    *gyro_z  = 0.0f;  /* Z轴不再使用 */
}

void mpu6050_get_gyro_biases(float *bx, float *by, float *bz)
{
    *bx = g_gyro_bias_x;
    *by = g_gyro_bias_y;
    *bz = g_gyro_bias_z;
}

void mpu6050_set_gyro_biases(float bx, float by, float bz)
{
    g_gyro_bias_x = bx;
    g_gyro_bias_y = by;
    g_gyro_bias_z = bz;
}

void mpu6050_get_gyro_ema(float *ema_x, float *ema_y)
{
    *ema_x = g_gyro_ema_x;
    *ema_y = g_gyro_ema_y;
}

bool mpu6050_get_last_data_valid(void)
{
    GyroSample *rd = &g_gyro_buf[g_gyro_wr ^ 1];
    return rd->valid && g_last_data_valid;
}
