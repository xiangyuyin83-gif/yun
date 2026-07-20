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

/* 发送一个字节 (等待 TX FIFO 有空位, 短超时) */
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
 *  UART 轮询 (在 tracker_update 中每周期调用)
 * ============================================================ */

static void uart_poll(void)
{
    /* 轮询 RX FIFO, 逐字节读出直到 FIFO 空 */
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        uint8_t byte = DL_UART_receiveData(UART_0_INST);

        if (byte == '\n' || byte == '\r') {
            /* \n 或 \r 都触发解析 (兼容不同串口助手的换行风格) */
            if (g_rx_idx > 0) {
                parse_message();
                g_rx_idx = 0;
            }
        } else if (g_rx_idx < RX_BUF_SIZE - 1) {
            g_rx_buf[g_rx_idx++] = (char)byte;
        } else {
            /* 溢出: 丢弃当前帧 */
            g_rx_idx = 0;
        }
    }
}

/* ============================================================
 *  死区 + 比例控制 → 电机指令
 * ============================================================ */

/*
 * 计算单个轴的电机方向和频率.
 *   error: 像素偏差 (正=目标偏右/下, 负=偏左/上)
 *   dead_zone: 死区 (像素)
 *   kp: 比例系数 (Hz/px)
 *   min_freq / max_freq: 频率边界
 *   out_dir: 输出方向 (-1=停止, 0=CCW, 1=CW)
 *   out_freq: 输出频率
 */
static void motor_compute(int32_t error, int32_t dead_zone,
                          float kp, int32_t min_freq, int32_t max_freq,
                          int32_t *out_dir, int32_t *out_freq)
{
    int32_t abs_err = (error < 0) ? -error : error;

    /* 死区内 → 停止 */
    if (abs_err <= dead_zone) {
        *out_dir  = -1;
        *out_freq = 0;
        return;
    }

    int32_t effective = abs_err - dead_zone;
    int32_t freq      = min_freq + (int32_t)(kp * (float)effective);

    if (freq > max_freq) freq = max_freq;
    if (freq < min_freq) freq = min_freq;

    *out_freq = freq;
    *out_dir  = (error > 0) ? 1 : 0;   /* 1=CW, 0=CCW */
}

/* 将计算结果写入硬件, 只在方向/频率变化时操作.
 * EN 引脚在 tracker_init 中已使能, 此处不再反复切换. */
static void motor_apply(StepMotorID motor, int32_t dir, int32_t freq,
                        int32_t *prev_dir, int32_t *prev_freq)
{
    /* 停止 */
    if (dir < 0 || freq == 0) {
        if (*prev_dir >= 0) {
            step_motor_stop(motor);
        }
        *prev_dir  = -1;
        *prev_freq = 0;
        return;
    }

    /* 方向变更 */
    if (dir != *prev_dir) {
        step_motor_set_dir(motor, (dir == 1) ? STEP_DIR_CW : STEP_DIR_CCW);
        *prev_dir = dir;
    }

    /* 频率变更: 直接 step_motor_start (与电机测试代码一致, 已验证可行) */
    if (freq != *prev_freq) {
        step_motor_start(motor, (uint32_t)freq);
        *prev_freq = freq;
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
    step_motor_set_dir(STEP_MOTOR_A, STEP_DIR_CW);
    step_motor_start(STEP_MOTOR_A, 200);
    delay_us(250000);  /* 50/200*1e6 = 250000us */
    step_motor_stop(STEP_MOTOR_A);

    /* 上电自检: 发送初始化消息, 确认 TX 通路正常 */
    uart_tx_str("init\r\n");

    g_enabled = true;
}

void tracker_update(void)
{
    if (!g_enabled) return;

    g_tick++;

    /* ---- 1. UART 轮询: 逐字节接收 + 帧解析 ---- */
    uart_poll();

    /* ---- 2. 超时检测 ---- */
    bool has_data  = (g_last_coord_tick != 0);
    bool timed_out = has_data
                  && ((g_tick - g_last_coord_tick) > g_cfg.timeout_ms);

    if (timed_out) {
        motor_apply(STEP_MOTOR_A, -1, 0, &g_motor_a_dir, &g_motor_a_freq);
        motor_apply(STEP_MOTOR_B, -1, 0, &g_motor_b_dir, &g_motor_b_freq);
        /* 超时时也发送解析应答 (如果有) */
        if (g_need_echo) {
            g_need_echo = false;
            uart_tx_echo_min(g_echo_char, g_echo_len, g_echo_v1);
        }
        return;
    }

    if (!has_data) {
        /* 非坐标消息的回传在解析后立即发送 (无电机计算) */
        if (g_need_echo) {
            g_need_echo = false;
            uart_tx_echo_min(g_echo_char, g_echo_len, g_echo_v1);
        }
        return;
    }

    /* ---- 3. 死区 + 比例控制 (仅在新数据到达时更新, 避免旧误差重复驱动) ---- */
    if (g_new_data) {
        g_new_data = false;

        int32_t dir_a, freq_a, dir_b, freq_b;

        motor_compute(g_coord.error_x, g_cfg.dead_zone_x,
                      g_cfg.kp_x, g_cfg.min_freq, g_cfg.max_freq,
                      &dir_a, &freq_a);
        motor_compute(g_coord.error_y, g_cfg.dead_zone_y,
                      g_cfg.kp_y, g_cfg.min_freq, g_cfg.max_freq,
                      &dir_b, &freq_b);

        /* 电机 B 机械方向与电机 A 相反, 翻转方向 */
        if (dir_b == 0) {
            dir_b = 1;
        } else if (dir_b == 1) {
            dir_b = 0;
        }

        /* ---- 4. 写入电机 ---- */
        motor_apply(STEP_MOTOR_A, dir_a, freq_a, &g_motor_a_dir, &g_motor_a_freq);
        motor_apply(STEP_MOTOR_B, dir_b, freq_b, &g_motor_b_dir, &g_motor_b_freq);

        /* 坐标消息: 回传解析+电机状态 (t<len>,<err_x>,<dir_a>,<freq_a>) */
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

    /* ---- 5. 延时: 确保电机 PWM 稳定启动 ---- */
    delay_us(5);
    /* 注意: 此处不再次轮询 RX, 避免干扰电机启动 */
}

void tracker_enable(bool en)
{
    if (g_enabled == en) return;
    g_enabled = en;

    if (en) {
        g_last_coord_tick = 0;
        g_rx_idx          = 0;
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
