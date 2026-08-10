#ifndef FLASH_PARAMS_H
#define FLASH_PARAMS_H

/**
 * @file flash_params.h
 * @brief 机器人参数管理：RAM 运行配置 + W25Q128 双备份持久化 + 范围限制
 *
 * 分层:
 *   算法层 imu_algo.c  →  参数层 flash_params  →  通信层 upper_protocol
 *                                                   ↕
 *                                                存储层 W25Q128
 *
 * 工作方式:
 *   开机  : FlashParams_Init() → 从 W25Q128 双 slot 中选有效且较新者加载到 RAM
 *   运行  : 所有模块通过 Config_Get() 只读访问 RAM 配置
 *   调参  : 上位机 PID_WRITE/PARAM_WRITE → Config_SetPID/SetParam (带范围限制) → RAM 立即生效
 *   保存  : 上位机 PID_SAVE → FlashParams_Save() → 写 W25Q128 (双备份，只有此步碰 Flash)
 *   恢复  : 上位机 CONFIG_RESET → FlashParams_ResetDefaults() → 恢复编译时默认值
 */

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================ 常量 ============================ */
#define CONFIG_MAGIC    0x524F424FU   /* "ROBO" ASCII，合法十六进制 */
#define CONFIG_VERSION  1U

/* W25Q128 双备份 slot (各占一个 4KB 扇区) */
#define FLASH_SLOT_A_ADDR   0U          /* 扇区 0 */
#define FLASH_SLOT_B_ADDR   4096U       /* 扇区 1 */

/* ============================ PID 子结构 ============================ */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_max;     /* 积分限幅 */
    float out_max;   /* 输出限幅 */
} PID_Config_t;

/* ============================ 主配置结构 ============================ */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;              /* 写入时的 sizeof(Robot_Config_t)，用于以后迁移 */

    /* ---- 航向 PID ---- */
    PID_Config_t yaw_track;     /* 行驶中航向跟踪 */
    PID_Config_t yaw_hold;      /* 静止航向锁定 */
    float yaw_gyro_k;           /* 陀螺阻尼增益 */
    float yaw_deadzone_deg;     /* 航向死区 */
    float yaw_cmd_threshold;    /* 手动旋转判断阈值 */

    /* ---- 底盘 ---- */
    float wheel_radius;         /* 轮半径 m (磨损可调) */
    float half_width_scale;     /* 旋转标定(半轮距) */
    float half_length_scale;    /* 旋转标定(半轴距) */
    float max_rpm;              /* 最大轮速限幅 */

    /* ---- 融合 ---- */
    float fusion_w_max;         /* 编码器权重上限 */
    float fusion_w_ramp;        /* 权重爬升速率 */

    /* ---- IMU 算法调参 ---- */
    float zupt_gyro_threshold;  /* ZUPT 角速度门限 */
    float zupt_acc_std_threshold; /* ZUPT 加速度标准差门限 */
    float kalman_q;             /* 偏航卡尔曼过程方差 Q */
    float kalman_r;             /* 偏航卡尔曼测量方差 R */

    /* ---- 预留(以后加参数，不改 layout) ---- */
    uint8_t reserved[32];

    uint32_t crc32;             /* CRC32(范围 = 从 magic 到 reserved 末尾，不含本字段) */
} Robot_Config_t;

/* ============================ 参数 ID(上位机 PARAM_READ/WRITE 用) ============================ */
typedef enum
{
    PARAM_INVALID = 0,
    PARAM_WHEEL_RADIUS,
    PARAM_HALF_WIDTH_SCALE,
    PARAM_HALF_LENGTH_SCALE,
    PARAM_MAX_RPM,
    PARAM_FUSION_W_MAX,
    PARAM_FUSION_W_RAMP,
    PARAM_ZUPT_GYRO,
    PARAM_ZUPT_ACC_STD,
    PARAM_KALMAN_Q,
    PARAM_KALMAN_R,
    PARAM_YAW_GYRO_K,
    PARAM_YAW_DEADZONE,
    PARAM_YAW_CMD_THRESHOLD,
    PARAM_COUNT
} Param_Id_t;

/* ============================ PID 通道(上位机 PID_READ/WRITE 用) ============================ */
#define PID_CH_YAW_TRACK  0x10U   /* 行驶中航向 PID */
#define PID_CH_YAW_HOLD   0x11U   /* 静止航向 PID */

/* ============================ API ============================ */

/** 开机初始化: 从 Flash 加载到 RAM (SPI 未开则用默认值) */
void FlashParams_Init(void);

/** 绑定 W25Q128 驱动(SPI 开启后调用; 传入 SPI 句柄 + CS 端口/引脚) */
void FlashParams_AttachFlash(void *spi_handle, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/** 从 Flash 加载到 RAM (双备份, 自动选较新有效 slot) */
bool FlashParams_Load(void);

/** 把 RAM 配置写入 Flash (双备份, 交替写) */
bool FlashParams_Save(void);

/** 恢复编译时默认值到 RAM (不写 Flash) */
void FlashParams_ResetDefaults(void);

/** 获取只读配置指针 (所有模块通过此接口读参数) */
const Robot_Config_t *Config_Get(void);

/* ---- 上位机调参接口(带范围限制) ---- */

/** 设置单个标量参数(自动 clamp 到合法范围) */
void Config_SetParam(Param_Id_t id, float value);

/** 获取单个标量参数 */
float Config_GetParam(Param_Id_t id);

/** 设置一组 PID 参数(5 个 float: kp ki kd i_max out_max, 自动 clamp) */
bool Config_SetPID(uint8_t channel, const float params[5]);

/** 获取一组 PID 参数 */
bool Config_GetPID(uint8_t channel, float params[5]);

#endif /* FLASH_PARAMS_H */
