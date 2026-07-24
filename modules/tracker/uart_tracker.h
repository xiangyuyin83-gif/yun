/*
 * uart_tracker.h — UART 坐标跟踪模块
 *
 * 功能: 接收 K230 摄像头发送的坐标偏差 (UART 115200 8N1),
 *       根据 error_x/error_y 控制双轴步进电机实现目标跟踪.
 *
 * 硬件: UART0 RX=PA28, TX=PB1 (TX 仅用于调试, 可悬空)
 *       电机A (X轴): ST=PA15(PWM_1), DIR=PB13, EN=PB15
 *       电机B (Y轴): ST=PB14(PWM_0), DIR=PB16, EN=PA12
 *
 * 协议:
 *   t,target_x,target_y,frame_cx,frame_cy,error_x,error_y\n   — 坐标数据
 *   f,key_id\n                                                  — 按键状态
 *
 * 集成方式 (不动原有代码):
 *   1. 在 main() 初始化后调用 tracker_init(&cfg)
 *   2. 在主循环中调用 tracker_update()
 *
 * 依赖: step_motor.h
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *  配置结构体
 * ============================================================ */

typedef struct {
    int32_t dead_zone_x;     /* X轴死区 (像素), |error|<=此值不动作 */
    int32_t dead_zone_y;     /* Y轴死区 (像素) */
    float   kp_x;            /* X轴比例系数 (Hz/px) */
    float   kp_y;            /* Y轴比例系数 (Hz/px) */
    int32_t min_freq;        /* 电机最低频率 (Hz) */
    int32_t max_freq;        /* 电机最高频率 (Hz) */
    uint32_t timeout_ms;     /* 数据超时 (ms), 超时后停止电机 */
} TrackerConfig;

/* 默认配置 */
#define TRACKER_DEFAULT_CONFIG {  \
    .dead_zone_x = 15,             \
    .dead_zone_y = 15,             \
    .kp_x        = 0.1f,           \
    .kp_y        = 0.1f,     /*下*/   \  
    .min_freq    = 20,             \
    .max_freq    = 100,          \
    .timeout_ms  = 1000000         \
}

/* ============================================================
 *  数据输出结构体
 * ============================================================ */

/* 解析后的坐标消息 */
typedef struct {
    int32_t target_x;       /* 目标中心 X (像素) */
    int32_t target_y;       /* 目标中心 Y (像素) */
    int32_t frame_cx;       /* 画面中心 X (例: 400) */
    int32_t frame_cy;       /* 画面中心 Y (例: 240) */
    int32_t error_x;        /* 偏差 X (像素), 目标-画面中心 */
    int32_t error_y;        /* 偏差 Y (像素) */
    bool    fresh;           /* 有新数据 (读取后由 get 函数清除) */
} TrackerCoord;

/* 解析后的按键消息 */
typedef struct {
    uint8_t key_id;         /* 键值 */
    bool    fresh;
} TrackerKey;

/* 跟踪器运行状态 */
typedef enum {
    TRACKER_IDLE  = 0,      /* 未使能 */
    TRACKER_ACTIVE,          /* 跟踪中 */
    TRACKER_LOST             /* 目标丢失 (超时) */
} TrackerState;

/* ============================================================
 *  公开接口
 * ============================================================ */

/*
 * 初始化跟踪模块
 *   1. UART 由 SysConfig 配置 (RX=PA18/TX=PA17, 115200 8N1)
 *   2. 加载配置参数
 *   3. 使能电机驱动器
 *
 * 调用时机: main() 初始化阶段, 在 SYSCFG_DL_init() 之后.
 */
void tracker_init(const TrackerConfig *cfg);

/*
 * 主循环更新 (每周期调用一次, 建议 500Hz~1000Hz)
 *   内部完成: UART 轮询 → 协议解析 → 坐标/按键更新
 *            → 死区+比例控制 → 电机 PWM 更新 → 超时检测
 */
void tracker_update(void);

/*
 * 使能/禁用跟踪
 *   禁用时停止 PWM 并拉低 EN (电机断电)
 *   使能时复位状态, 重新使能电机
 */
void tracker_enable(bool en);

/* 获取最新坐标 (读取后 fresh 标志清零, 可据此判断是否新数据) */
TrackerCoord tracker_get_coord(void);

/* 获取最新按键事件 (读取后 fresh 清零) */
TrackerKey tracker_get_key(void);

/* 获取跟踪器当前状态 */
TrackerState tracker_get_state(void);

/* 获取距上次收到有效坐标的毫秒数 (用于外部超时判断) */
uint32_t tracker_coord_age_ms(void);

#ifdef __cplusplus
}
#endif
