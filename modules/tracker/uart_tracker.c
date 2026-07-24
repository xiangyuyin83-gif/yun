/*
 * uart_tracker.c — UART 坐标跟踪模块实现
 *
 * 功能: 接收 K230 坐标, 死区+比例控制双轴步进电机跟踪目标
 * 协议: t,target_x,target_y,frame_cx,frame_cy,error_x,error_y\n
 *       f,key_id\n
 * 硬件: UART0 RX=PA28, TX=PB1, 115200 8N1 (由 SysConfig 配置)
 * 依赖: ti_msp_dl_config.h, step_motor.h
 *
 * 集成: 在 main() 中调用 tracker_init(&cfg) 和 tracker_update()
 */
#include "ti_msp_dl_config.h"

#include "modules/tracker/uart_tracker.h"
#include "modules/motor/step_motor.h"

#include <limits.h>

/* 接收缓冲 */
#define RX_BUF_SIZE  128

/* 坐标数值字段数 (t 标识后共 6 个整数) */
#define COORD_FIELDS  6

/* ============================================================
 *  模块内部状态
 * ============================================================ */

static TrackerConfig g_cfg;
static bool          g_enabled;

/* UART 接收 */
static char   g_rx_buf[RX_BUF_SIZE];
static uint8_t g_rx_idx;

/* 解析结果 */
static volatile TrackerCoord g_coord;
static volatile TrackerKey   g_key;
static volatile bool    g_need_echo;      /* 待回传 */
static volatile char    g_echo_char;      /* 消息类型 'i'/'t'/'f'/'?'/'e' */
static volatile int32_t g_echo_len;       /* 原始消息字节数 */
static volatile int32_t g_echo_v1;        /* 附加信息 (t:error_x, e:fail_field) */

/* 超时检测 (tick 每 tracker_update 调用 +1, 约 1ms/tick) */
static uint32_t g_last_coord_tick;
static uint32_t g_tick;

/* 电机缓存 (避免冗余 stop/start/dir 切换) */
static int32_t g_motor_a_dir;     /* -1=停止, 0=CCW, 1=CW */
static int32_t g_motor_b_dir;
static int32_t g_motor_a_freq;
static int32_t g_motor_b_freq;
static bool    g_new_data;        /* 本周期收到新坐标 */
static volatile bool g_frame_ready; /* ISR 通知主循环: 有一帧待解析 */

/* ============================================================
 *  辅助: 手动解析整数 (避免 sscanf/libc 依赖)
 * ============================================================ */

/*
 * 从字符串当前位置解析一个有符号整数, 解析后 *str 指向逗号/换行/结束.
 * 返回 true=成功, false=格式错误.
 */
static bool parse_int_field(const char **str, int32_t *value)
{
    bool    neg = false;
    int32_t v   = 0;

    while (**str && **str != '-' && (**str < '0' || **str > '9')) {
        (*str)++;
    }
    if (**str == '\0') return false;

    if (**str == '-') { neg = true; (*str)++; }
    if (**str < '0' || **str > '9') return false;

    while (**str >= '0' && **str <= '9') {
        v = v * 10 + (**str - '0');
        (*str)++;
    }
    *value = neg ? -v : v;
    return true;
}

/* ============================================================
 *  UART TX 辅助 (应答回传)
 * ============================================================ */

/*
 * 发送一个字节 (忙等 TX FIFO 有空位, 超时 2000 次循环后放弃).
 *
 * 耗时分析:
 *   115200 baud → 每个字节需约 87µs 才能从移位寄存器发出.
 *   FIFO 满 (硬件 FIFO 通常 8~16 字节深) 时, 需等约 87µs 腾出一个空位.
 *   2000 次循环 @32MHz (约 3~10 周期/次) → ~60~200µs → 足以等 1~2 字节.
 *
 * 隐患:
 *   若 FIFO 持续满 (如对端不接收), 2000 次后静默丢字节, 无错误日志.
 *   对于回传应答 (辅助功能), 偶尔丢一个字节不影响跟踪, 可接受.
 */
static bool uart_tx_byte_wait(char c)
{
    for (uint32_t t = 0; t < 2000; t++) {
        if (!DL_UART_isTXFIFOFull(UART_0_INST)) {
            DL_UART_transmitData(UART_0_INST, (uint8_t)c);
            return true;
        }
    }
    return false;
}

/* 发送字符串 (非阻塞, 任一字节能发送失败则停止) */
static void uart_tx_str(const char *s)
{
    while (*s) {
        if (!uart_tx_byte_wait(*s)) return;
        s++;
    }
}

/* 发送有符号整数 (非阻塞) */
static void uart_tx_int(int32_t v)
{
    if (v < 0) {
        if (!uart_tx_byte_wait('-')) return;
        v = -v;
    }

    char buf[12];
    int  pos = 0;

    if (v == 0) {
        buf[pos++] = '0';
    } else {
        while (v > 0) {
            buf[pos++] = (char)('0' + (v % 10));
            v /= 10;
        }
    }

    while (pos > 0) {
        if (!uart_tx_byte_wait(buf[--pos])) return;
    }
}

/*
 * 极简回传: <type><len>,<v1>\r\n  例: t22,200\r\n 或 i1,0\r\n
 */
static void uart_tx_echo_min(char type, int32_t len, int32_t v1)
{
    if (!uart_tx_byte_wait(type)) return;
    uart_tx_int(len);
    if (!uart_tx_byte_wait(',')) return;
    uart_tx_int(v1);
    if (!uart_tx_byte_wait('\r')) return;
    uart_tx_byte_wait('\n');
}

/* ============================================================
 *  协议解析
 * ============================================================ */

static void parse_message(void)
{
    if (g_rx_idx == 0) return;
    g_rx_buf[g_rx_idx] = '\0';

    const char *p = g_rx_buf;

    if (g_rx_buf[0] == 'i') {
        g_echo_char = 'i'; g_echo_len = g_rx_idx; g_echo_v1 = 0;
        g_need_echo = true;
    } else if (g_rx_buf[0] == 't') {
        /* 格式: t,target_x,target_y,frame_cx,frame_cy,error_x,error_y */
        int32_t vals[COORD_FIELDS];
        int32_t fail_field = -1;

        for (int i = 0; i < COORD_FIELDS; i++) {
            /* 跳到逗号 (i=0 跳过 't' 本身) */
            while (*p && *p != ',') p++;
            if (*p == ',') p++;

            if (!parse_int_field(&p, &vals[i])) {
                fail_field = i;
                break;
            }
        }

        if (fail_field < 0) {
            g_coord.target_x  = vals[0];
            g_coord.target_y  = vals[1];
            g_coord.frame_cx  = vals[2];
            g_coord.frame_cy  = vals[3];
            g_coord.error_x   = vals[4];
            g_coord.error_y   = vals[5];
            g_coord.fresh     = true;
            g_last_coord_tick = g_tick;
            g_new_data        = true;

            /* 应答: t,raw_len,error_x */
            g_echo_char = 't'; g_echo_len = g_rx_idx; g_echo_v1 = g_coord.error_x;
            g_need_echo = true;
        } else {
            /* 解析失败: e,raw_len,fail_field */
            g_echo_char = 'e'; g_echo_len = g_rx_idx; g_echo_v1 = fail_field;
            g_need_echo = true;
        }
    } else if (g_rx_buf[0] == 'f') {
        /* 格式: f,key_id */
        p++;  /* 跳过 'f' */
        while (*p && *p != ',') p++;
        if (*p == ',') p++;

        int32_t kv = 0;
        if (parse_int_field(&p, &kv)) {
            g_key.key_id = (uint8_t)kv;
            g_key.fresh  = true;

            /* 应答: f,key_id,0 (延迟发送) */
            g_echo_char = 'f'; g_echo_len = g_rx_idx; g_echo_v1 = g_key.key_id;
            g_need_echo = true;
        }
    } else {
        /* 未识别协议 → ?,raw_len,0 (延迟发送) */
        g_echo_char = '?'; g_echo_len = g_rx_idx; g_echo_v1 = 0;
        g_need_echo = true;
    }
}

/* ============================================================
 *  UART RX 中断服务例程 (替代轮询)
 * ============================================================ */
/*
 * 接收延时分析:
 *
 *   硬件层:
 *     115200 baud → 每字节 87µs (1/115200 × 10bit, 含起始+停止位).
 *     一帧约 30 字节 → 约 2.6ms 全部到达.
 *
 *   中断触发:
 *     RX FIFO ≥ 1 字节时硬件自动触发 ISR, 无需 CPU 轮询.
 *     ISR 只负责收字节 + 拼帧, 设 g_frame_ready 通知主循环,
 *     不做协议解析 (耗时操作留给主循环).
 *
 *   帧级延时:
 *     \n 到达硬件 → ISR 立即读出 (<1µs) → 设 g_frame_ready
 *     → 主循环下次迭代 (最多 30~60µs) → parse_message().
 *     从 \n 到电机 PWM 更新, 软件延迟 ≤ 60µs (与轮询相同).
 *
 *   并发保护:
 *     g_frame_ready==true 期间 ISR 丢弃新字节 (不写 g_rx_buf).
 *     主循环处理完一帧 (<20µs) 后才清标志, 期间新字节暂留
 *     硬件 FIFO (8 字节). 115200 baud 下 20µs 无法填满 FIFO,
 *     不会溢出.
 */
void UART_0_INST_IRQHandler(void)
{
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t byte = DL_UART_receiveData(UART_0_INST);

        /*
         * 上一帧主循环还没消费 (g_frame_ready 仍为 true):
         *   丢弃当前字节, 避免覆盖 g_rx_buf.
         *   最多丢一个帧头字节 ('t'), 下一帧 \n 对齐后恢复.
         */
        if (g_frame_ready) {
            continue;
        }

        if (byte == '\n' || byte == '\r') {
            /* 换行符触发帧完成: 终止字符串, 设标志 */
            if (g_rx_idx > 0) {
                g_rx_buf[g_rx_idx] = '\0';
                g_frame_ready = true;
            }
        } else if (g_rx_idx < RX_BUF_SIZE - 1) {
            g_rx_buf[g_rx_idx++] = (char)byte;
        } else {
            /* 溢出: 丢弃当前帧, 等下一个 \n 重新对齐 */
            g_rx_idx = 0;
        }
    }
}

/* ============================================================
 *  死区 + 比例控制 → 电机指令
 * ============================================================ */

/*
 * motor_compute() —— 单轴死区 + 比例控制
 *
 * 输入一个像素偏差, 输出一个电机指令 (方向 + 频率).
 * 每个轴独立调用, X 轴和 Y 轴互不影响.
 *
 * 参数说明:
 *   error     : 像素偏差. 正 = 目标偏右/下, 负 = 目标偏左/上.
 *               例: error=30 → 目标在中心偏右 30 像素
 *   dead_zone : 死区 (像素). |error| <= 此值 → 电机不动.
 *               作用: 目标接近中心时避免微小抖动导致电机反复启停.
 *   kp        : 比例系数 (Hz/px). 有效偏差每多 1 像素就多加这么多 Hz.
 *               例: kp=0.4 → 偏差 10px 多加 4Hz
 *   min_freq  : 电机最低运行频率 (Hz). 低于此频率电机可能扭矩不够、堵转.
 *   max_freq  : 电机最高运行频率 (Hz). 限制最大速度, 防止丢步或飞车.
 *   out_dir   : 输出方向. -1=停止, 0=CCW(逆时针), 1=CW(顺时针)
 *   out_freq  : 输出频率 (Hz). 0 表示不转.
 *
 * 计算公式:
 *   abs_err   = |error|                          ← 取绝对值
 *   effective = abs_err - dead_zone              ← 超出死区的部分
 *   freq      = min_freq + kp × effective        ← 死区外比例加速
 *
 * 计算示例 (error=30, dead_zone=15, kp=0.4, min=0, max=100):
 *   abs_err   = 30
 *   30 > 15   → 超出死区, 继续
 *   effective = 30 - 15 = 15
 *   freq      = 0 + 0.4 × 15 = 6 Hz
 *   6 在 [0, 100] 内, 不限幅
 *   dir       = 1 (CW, 因为 30 > 0)
 *
 * 计算示例 (error=-10, dead_zone=15):
 *   abs_err   = 10
 *   10 <= 15  → 在死区内 → dir=-1, freq=0 → 电机停
 */
static void motor_compute(int32_t error, int32_t dead_zone,
                          float kp, int32_t min_freq, int32_t max_freq,
                          int32_t *out_dir, int32_t *out_freq)
{
    /* ① 取绝对值: 不管目标偏左还是偏右, 要转的频率只看距离 */
    int32_t abs_err = (error < 0) ? -error : error;

    /* ② 死区判断: 目标离中心够近了, 不需要动
     *    例: 死区=15, error=10 → abs_err=10 ≤ 15 → 停
     *    避免目标在中心附近微小漂移时电机反复抖 */
    if (abs_err <= dead_zone) {
        *out_dir  = -1;          /* -1 = 停止 */
        *out_freq = 0;           /* 无脉冲 */
        return;
    }

    /* ③ 超出死区的部分 = 真正需要转动的距离 */
    int32_t effective = abs_err - dead_zone;
    if (effective <= 0) {
        *out_dir  = -1;
        *out_freq = 0;
        return;
    }

    /* ④ 比例公式: 从 0 开始线性加速, 不加 min_freq.
     *    +0.5f 四舍五入, 消除整数截断造成的"第二死区" */
    int32_t freq = min_freq + (int32_t)(kp * abs_err);

    /* ⑤ 限幅: 上限 max_freq, 下限 0 (死区内已直接返回 0) */
    if (freq > max_freq) freq = max_freq;
    if (freq <= min_freq)   freq = 0;

    *out_freq = freq;

    /* ⑥ 方向: 利用 error 原本的正负号
     *    error > 0 → 目标偏右/下 → 1 (CW 顺时针)
     *    error < 0 → 目标偏左/上 → 0 (CCW 逆时针)
     *    注意: 这里的 CW/CCW 是逻辑方向, 实际云台往哪边转
     *    还取决于电机安装方向 (Motor B 就在下面翻转了) */
    *out_dir = (error > 0) ? 1:0 ;
}

/*
 * motor_apply() —— 把电机指令写入硬件，带状态缓存
 *
 * 不是每次都直接写寄存器，而是先跟上次写入的值比较：
 *   方向没变 → 跳过 set_dir
 *   频率没变 → 跳过 start
 *   什么都没变 → 整个函数什么都不干，直接返回
 *
 * 四个参数：
 *   motor     : 哪个电机 (STEP_MOTOR_A / STEP_MOTOR_B)
 *   dir       : 方向，-1=停止, 0=CCW, 1=CW
 *   freq      : 频率 (Hz)，0=停止
 *   prev_dir  : 上次写入这个电机的方向（既是输入也是输出，调用后会更新）
 *   prev_freq : 上次写入这个电机的频率（同上）
 *
 * 停止条件 (dir<0 或 freq==0):
 *   只要满足一个就停止 PWM 脉冲。
 *   dir<0 是 motor_compute 返回的"死区内"信号（dir=-1）
 *   freq==0 是频率被截断为 0 的情况（kp太小或超出死区但有效偏差不足1px）
 *
 * 方向变更:
 *   dir 从 0→1 或 1→0 时调用 step_motor_set_dir 切换 DIR 引脚电平
 *   dir 从 -1→0 或 -1→1 时也要设（从停止到启动，必须先设方向）
 *   写完后更新 prev_dir 缓存
 *
 * 频率变更:
 *   频率跟上次不一样时才调 step_motor_start 更新 PWM 周期
 *   0→N 是冷启动，N→M 是变速，都会触发
 *   写完后更新 prev_freq 缓存
 *
 * 设计意图:
 *   tracker_update 每 10us 左右调一次，绝大多数时候误差没变化或变化很小，
 *   缓存机制让电机在稳定跟踪时几乎不产生任何硬件寄存器操作，
 *   只在方向或频率真正改变时才动硬件。
 */
static void motor_apply(StepMotorID motor, int32_t dir, int32_t freq,
                        int32_t *prev_dir, int32_t *prev_freq)
{
    /*
     * ① 停止判断:
     *    dir=-1 → motor_compute 判定在死区内
     *    freq=0 → 频率为 0（死区内或 kp 太小被截断）
     *    两种情况都意味着"这轴不该转"
     */
    if (dir < 0 || freq == 0) {
        /*
         * 上次还在转 (*prev_dir >= 0) 才需要真正 stop，
         * 如果上次已经停了 (*prev_dir == -1)，跳过以避免重复操作硬件
         */
        if (*prev_dir >= 0) {
            step_motor_stop(motor);       /* 停止 PWM 输出，电机断电不转 */
        }
        *prev_dir  = -1;                  /* 缓存更新为"已停止" */
        *prev_freq = 0;
        return;
    }

    /*
     * ② 方向变更:
     *    新方向和缓存不同 → DIR 引脚需要翻转
     *    例: 上一帧目标偏左（dir=0,CCW），这一帧偏右（dir=1,CW）
     */
    if (dir != *prev_dir) {
        step_motor_set_dir(motor,
            (dir == 1) ? STEP_DIR_CW : STEP_DIR_CCW);
        *prev_dir = dir;                  /* 缓存新方向 */
    }

    /*
     * ③ 频率变更:
     *    新频率和缓存不同 → 更新 PWM 周期寄存器
     *    例: 上一帧偏差大（freq=50Hz），这一帧接近中心（freq=6Hz）
     *    注意: 频率没变时跳过，这是最常见的路径 ——
     *    目标匀速移动时连续多帧误差一样，不需要重复写定时器
     */
    if (freq != *prev_freq) {
        step_motor_start(motor, (uint32_t)freq);
        *prev_freq = freq;                /* 缓存新频率 */
    }
}

/* ============================================================
 *  公开接口
 * ============================================================ */

void tracker_init(const TrackerConfig *cfg)
{
    g_cfg = *cfg;

    /* UART 硬件由 SysConfig 的 SYSCFG_DL_UART_0_init() 初始化,
     * 已在 main() 的 SYSCFG_DL_init() 中调用. */

    /* 状态清零 */
    g_enabled         = false;
    g_rx_idx          = 0;
    g_tick            = 0;
    g_last_coord_tick = 0;
    g_motor_a_dir     = -1;
    g_motor_b_dir     = -1;
    g_motor_a_freq    = 0;
    g_motor_b_freq    = 0;
    g_new_data        = false;
    g_frame_ready     = false;

    g_coord.target_x  = 0; g_coord.target_y = 0;
    g_coord.frame_cx  = 0; g_coord.frame_cy = 0;
    g_coord.error_x   = 0; g_coord.error_y  = 0;
    g_coord.fresh     = false;
    g_key.key_id      = 0;
    g_key.fresh       = false;
    g_need_echo       = false;

    /* 使能电机 + 验证脉冲 */
    step_motor_enable(STEP_MOTOR_A, 1);
    step_motor_enable(STEP_MOTOR_B, 1);

    /* 诊断: 用 step_motor_move 发 50 步 200Hz 脉冲, 验证硬件通路 */
#if 0
    step_motor_set_dir(STEP_MOTOR_A, STEP_DIR_CW);
    step_motor_start(STEP_MOTOR_A, 200);
    delay_us(250000);  /* 50/200*1e6 = 250000us */
    step_motor_stop(STEP_MOTOR_A);
#endif

    /* 上电自检: 发送初始化消息, 确认 TX 通路正常 */
    uart_tx_str("init\r\n");

    /*
     * 使能 UART RX 中断 (替代轮询).
     *
     * 1. RX FIFO 阈值设为 1 字节 → 每收到一个字节就触发 ISR,
     *    保证最低延迟.
     * 2. 使能 RX 中断源 (UART 外设侧).
     * 3. 使能 NVIC 中断线 (CPU 侧).
     *
     * 注意: 主循环中 SYSCFG_DL_init() 已完成 UART 时钟和引脚配置,
     *       此处只操作中断相关寄存器, 不与 SysConfig 冲突.
     */
    DL_UART_setRXFIFOThreshold(UART_0_INST,
        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_RX);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    g_enabled = true;
}

/*
 * tracker_update() —— 主循环每周期调用一次的核心函数
 *
 * 调用频率: 建议 500~1000 Hz (由主循环 delay_us(10) 决定, 约 100kHz 理论值,
 *           但实际受 UART 接收和电机 PWM 更新耗时限制, 有效频率约 1kHz).
 *
 * 每次调用做五件事:
 *   ① 检查 ISR 设的 g_frame_ready 标志, 若有完整帧则解析 error_x / error_y
 *   ② 超时检测: 太久没收到坐标 → 停电机
 *   ③ 没收到过坐标 → 只回应答, 不动电机
 *   ④ 有新的坐标数据 → 死区+比例公式算方向和频率 → 写硬件 PWM
 *   ⑤ 回传给 K230 确认
 *
 * 与 ISR 的协作:
 *   UART0_IRQHandler (ISR) 负责收字节拼帧, 在 \n 到达时设 g_frame_ready=true.
 *   tracker_update() 检测到标志后关中断 → parse_message() 解析 → 复位索引 →
 *   清标志 → 开中断. parse_message() 填充 g_coord 并设 g_new_data=true,
 *   随后 tracker_update() 消费 g_new_data 并更新电机.
 */
void tracker_update(void)
{
    /* 未使能: tracker_enable(false) 之后, 整个模块静默 */
    if (!g_enabled) return;

    /* 心跳计数器 +1:
     *   与 g_last_coord_tick 配合, 用于计算"距离上次收坐标过了多久".
     *   g_tick 一直递增, g_last_coord_tick 在每次收到 t 帧时更新为 g_tick 的值.
     *   两者差值就是"多少周期没收坐标了". */
    g_tick++;

    /* ---- 1. 检查中断收到的帧 ----
     *   ISR 在 \n 到达时设 g_frame_ready=true.
     *   此处关 UART RX 中断 → 解析 → 复位索引 → 清标志 → 开中断.
     *   关中断期间字节暂留硬件 FIFO (8 字节), 115200 baud 下 20µs 不会满. */
    if (g_frame_ready) {
        NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
        parse_message();
        g_rx_idx       = 0;
        g_frame_ready  = false;
        NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    }

    /* ---- 2. 超时检测 ----
     *   has_data:  是否曾经收到过至少一次有效的 t 帧?
     *              上电后第一帧到达前为 false, 此后永远为 true.
     *   timed_out: 距离上次收到 t 帧是否超过了配置的 timeout_ms? */
    bool has_data  = (g_last_coord_tick != 0);
    bool timed_out = has_data
                  && ((g_tick - g_last_coord_tick) > g_cfg.timeout_ms);

    if (timed_out) {
        /* 超时 → 立即停止两个电机, 防止目标丢失后云台继续空转 */
        motor_apply(STEP_MOTOR_A, -1, 0, &g_motor_a_dir, &g_motor_a_freq);
        motor_apply(STEP_MOTOR_B, -1, 0, &g_motor_b_dir, &g_motor_b_freq);

        /* 如果有还没发出的应答 (边缘情况), 补发一个再走 */
        if (g_need_echo) {
            g_need_echo = false;
            uart_tx_echo_min(g_echo_char, g_echo_len, g_echo_v1);
        }
        return;
    }

    if (!has_data) {
        /* 还没收到过任何 t 帧:
         *   电机保持初始停止状态不动.
         *   但可能收到了 f (按键) 或 i (心跳) 等非坐标消息,
         *   这些消息的应答在这里发出. */
        if (g_need_echo) {
            g_need_echo = false;
            uart_tx_echo_min(g_echo_char, g_echo_len, g_echo_v1);
        }
        return;
    }

    /* ---- 3. 有新坐标数据 → 死区 + 比例控制 ----
     *   只在 g_new_data==true 时进入, 处理完立刻消费掉.
     *   这样避免同一帧数据被重复处理, 也避免旧误差重复驱动电机. */
    if (g_new_data) {
        g_new_data = false;            /* 消费标志位 */

        int32_t dir_a, freq_a;         /* Motor A 的方向和频率 */
        int32_t dir_b, freq_b;         /* Motor B 的方向和频率 */

        /* 3a. Motor B (先算)
         *     输入: error_x (如 30 = 目标偏右 30px)
         *     输出: dir_b, freq_b */
        motor_compute(g_coord.error_x, g_cfg.dead_zone_y,
                      g_cfg.kp_y, g_cfg.min_freq, g_cfg.max_freq,
                      &dir_b, &freq_b);

        /* 3b. Motor A (后算)
         *     输入: error_y (如 -20 = 目标偏上 20px)
         *     输出: dir_a (-1=停, 0=CCW, 1=CW), freq_a (Hz) */
        motor_compute(g_coord.error_y, g_cfg.dead_zone_x,
                      g_cfg.kp_x, g_cfg.min_freq, g_cfg.max_freq,
                      &dir_a, &freq_a);

        /* 3c. Motor A 翻转, Motor B 不翻转 */
        if (dir_a == 0) {
            dir_a = 1;
        } else if (dir_a == 1) {
            dir_a = 0;
        }

        /* ---- 4. 写入电机硬件 ----
         *   motor_apply 内部有状态缓存:
         *     方向或频率和上次一样 → 什么都不做
         *     变了 → 调 step_motor_set_dir / step_motor_start / step_motor_stop */
        motor_apply(STEP_MOTOR_A, dir_a, freq_a, &g_motor_a_dir, &g_motor_a_freq);
        motor_apply(STEP_MOTOR_B, dir_b, freq_b, &g_motor_b_dir, &g_motor_b_freq);

        /* 4a. 回传给 K230: 确认收到了坐标, 并附上当前电机 A 的状态
         *     格式: t<帧长>,<error_x>,<dir_a>,<freq_a>\r\n
         *     例:   t26,30,1,6\r\n
         *     含义: 收到你 26 字节的帧, error_x=30, Motor A 方向 CW, 频率 6Hz */
        if (g_need_echo) {
            g_need_echo = false;
            uart_tx_echo_min(g_echo_char, g_echo_len, g_echo_v1);
            uart_tx_byte_wait(',');
            uart_tx_int(dir_a);
            uart_tx_byte_wait(',');
            uart_tx_int(freq_a);
            uart_tx_byte_wait('\r');
            uart_tx_byte_wait('\n');
        }
    }

    /*
     * ---- 5. PWM 硬件已由电机驱动层写完, 此处 5µs 延时仅为
     *    给下次 tracker_update 入口留一个极短间隔, 避免在主循环
     *    无其他耗时逻辑时空转. 无功能意义, 可删.
     */
    delay_us(5);
}

void tracker_enable(bool en)
{
    if (g_enabled == en) return;
    g_enabled = en;

    if (en) {
        g_last_coord_tick = 0;
        g_rx_idx          = 0;
        g_frame_ready     = false;
        g_coord.fresh     = false;
        g_key.fresh       = false;

        step_motor_enable(STEP_MOTOR_A, 1);
        step_motor_enable(STEP_MOTOR_B, 1);
    } else {
        step_motor_stop(STEP_MOTOR_A);
        step_motor_stop(STEP_MOTOR_B);
        step_motor_enable(STEP_MOTOR_A, 0);
        step_motor_enable(STEP_MOTOR_B, 0);

        g_motor_a_dir  = -1;
        g_motor_b_dir  = -1;
        g_motor_a_freq = 0;
        g_motor_b_freq = 0;
    }
}

TrackerCoord tracker_get_coord(void)
{
    TrackerCoord c = g_coord;
    g_coord.fresh = false;
    return c;
}

TrackerKey tracker_get_key(void)
{
    TrackerKey k = g_key;
    g_key.fresh = false;
    return k;
}

TrackerState tracker_get_state(void)
{
    if (!g_enabled) return TRACKER_IDLE;
    if (g_last_coord_tick == 0) return TRACKER_ACTIVE;  /* 等待第一帧 */
    if ((g_tick - g_last_coord_tick) > g_cfg.timeout_ms) return TRACKER_LOST;
    return TRACKER_ACTIVE;
}

uint32_t tracker_coord_age_ms(void)
{
    if (g_last_coord_tick == 0) return UINT32_MAX;
    return g_tick - g_last_coord_tick;
}
