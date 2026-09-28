/**
 *******************************************************************************
 * @file      :ControlTask.cpp
 * @brief     : GM6020 正弦轨迹跟踪 + 分层看门狗保护
 *
 *  用 GM6020「分别」跟随一条以正弦波变化的
 *      - 速度曲线   (g_mode = kModeVelocity)
 *      - 位置曲线   (g_mode = kModePosition)
 *  曲线参数与增益全部集中在下面的「曲线自定义 / 控制器」两节, 改完重新编译即可;
 *  运行中也可以在 Ozone / JLink 里直接改变量 g_mode 切换两种曲线。
 *
 *  控制结构(1kHz):
 *      位置模式:  位置环(P) --速度给定--> 速度环(PI) --> 电压
 *                 并在两处加入解析前馈 (速度前馈 / 加速度前馈)
 *      速度模式:  速度环(PI) --> 电压, 同样带前馈
 *
 *  保护(三层, 各自职责不同, 不能互相替代):
 *      1. CAN 反馈超时  kCanTimeoutMs  -> 立即输出置零并退回 IDLE (可自恢复)
 *      2. 过温          kOverTempC     -> 锁存 FAULT, 需降温后自动解除
 *      3. IWDG          ≈0.8s(见 iwdg.c) -> 本次 1kHz 控制周期完整跑完才喂狗
 *
 * @attention : 首次上电务必分阶段调试, 不要直接带机构跑:
 *      只接收 -> 开环小电压 -> 速度环 -> 位置环 -> 加前馈 -> 故障注入
 *      任一阶段异常都要先停下来确认, 尤其是方向与限位。
 *******************************************************************************
 */

#include "ControlTask.h"

#include "HW_can.hpp"
#include "Gm6020.h"
#include "Pid.h"
#include "system_user.hpp"

#include <cmath>

/* ===========================================================================
 *  曲线自定义
 * =========================================================================*/

namespace cfg {

// ---- 电机与总线 ----
constexpr uint8_t        kMotorId = 1;       // GM6020 拨码 ID (1~7)
CAN_HandleTypeDef *const kCan     = &hcan1;  // 电机挂在哪条 CAN 上

// ---- 两种曲线各自的参数(自定义曲线就是改这里) ----
// 速度曲线:  v_ref(t) = kVelAmp * sin(2*pi*kVelFreq*t)
constexpr float kVelAmp  = 3.0f;    // rad/s   幅值
constexpr float kVelFreq = 0.5f;    // Hz      频率

// 位置曲线:  theta_ref(t) = kPosAmp * sin(2*pi*kPosFreq*t)
constexpr float kPosAmp  = 1.57f;   // rad     幅值 (1.57 ≈ 90°)
constexpr float kPosFreq = 0.5f;    // Hz      频率

// 上电默认使用哪条曲线
constexpr uint8_t kDefaultMode = 0;  // 0 = 速度曲线, 1 = 位置曲线

// ---- 控制器增益 ----
//
// ⚠️ 量纲很重要: 输出 u 的单位是 GM6020 的"电压给定"(-25000~25000), 不是伏特,
//    也不是转矩。所以 Kp 的量级取决于电机增益 K = 稳态转速/电压给定。
//    若 K ≈ 0.0012 [rad/s per 单位] (即 25000 对应空载约 30 rad/s),
//    则速度环 Kp 要在 1e4 量级; 给 200 会慢到几乎不响应(见验证报告 T6)。
//    真机请先按 "测 K" 的步骤实测, 再整定。
constexpr float kVelKp = 3000.0f;   // 保守初值, 实测后应升到 1e4~3e4 量级
constexpr float kVelKi = 300.0f;
constexpr float kVelKd = 0.0f;

constexpr float kPosKp = 20.0f;     // 位置环输出是速度给定(rad/s), 量纲正常
constexpr float kPosKi = 0.0f;
constexpr float kPosKd = 0.0f;

// ---- 前馈增益 ----
// 先定义两个需要实测的电机参数:
//     K   = 稳态增益 [rad/s per 电压单位] ≈ 空载最高转速 / 25000
//     tau = 机电时间常数 [s]
// 若 K 未知: 速度前馈保持 0 (只用反馈, 精度差但安全), 测出 K 后再打开。
//
// ⚠️ 下面四个名字相似但量纲不同, 不要混用:
//   速度模式:  u     += kVelFfV * v_ref + kVelFfA * a_ref
//              kVelFfV [电压/(rad/s)]   ≈ 1/K    抵消维持该转速所需电压
//              kVelFfA [电压/(rad/s²)]  ≈ tau/K  抵消加速所需额外电压
//   位置模式:  v_cmd += kPosFfV * v_ref
//              u     += kPosFfA * a_ref
//              kPosFfV [无量纲]         ≈ 1.0    直接把参考速度当前馈(安全, 建议开)
//              kPosFfA [电压/(rad/s²)]  ≈ tau/K
//
// 默认按 K≈0.0012、tau≈0.1s 估计: 1/K≈833, tau/K≈83
constexpr float kVelFfV = 0.0f;    // ≈ 833, 实测 K 后打开
constexpr float kVelFfA = 0.0f;    // ≈ 83
constexpr float kPosFfV = 1.0f;    // 无量纲, 建议保持 1.0
constexpr float kPosFfA = 0.0f;    // ≈ 83, 实测 tau 后打开

// ---- 限幅 ----
constexpr float kVoltMax   = 25000.0f;  // 电压给定上限(GM6020: -25000~25000)
constexpr float kMaxVelCmd = 20.0f;     // 位置环输出(即速度给定)上限 rad/s

// ---- 保护阈值 ----
constexpr uint32_t kCanTimeoutMs = 20;    // 反馈超时 -> 断输出
constexpr float    kOverTempC    = 70.0f; // 过温
constexpr float    kOverTempHys  = 5.0f;  // 过温恢复迟滞
constexpr uint32_t kArmDelayMs   = 2000;  // 上电后自动使能延时

// ---- 派生量 ----
constexpr float kTwoPi = 6.283185307179586f;
// 采样周期。必须与 1kHz 控制定时器(以及 system_user.hpp 的 kCtrlPeriod)一致。
// 这里不能写成 kCtrlPeriod: 它是 const float 而非 constexpr, 无法用于常量表达式。
constexpr float kDt    = 0.001f;

// 当前生效的曲线参数
constexpr uint8_t kModeVelocity = 0;
constexpr uint8_t kModePosition = 1;

inline float Amp(uint8_t mode)  { return (mode == kModeVelocity) ? kVelAmp  : kPosAmp;  }
inline float Freq(uint8_t mode) { return (mode == kModeVelocity) ? kVelFreq : kPosFreq; }

}  // namespace cfg

/* ===========================================================================
 *  状态机 与 可观测变量
 *
 *  这些变量刻意不加 static, 方便在 Ozone / JLink 里直接观察或修改。
 *  tick 的定义在 Tasks/main_task.cpp (与 system_user.hpp 的 extern 对应)。
 * =========================================================================*/

enum TaskState : uint8_t {
    kStateIdle    = 0,   // 输出 0, 等待使能
    kStateRunning = 1,   // 正在跟踪曲线
    kStateFault   = 2,   // 故障锁存, 输出 0
};

volatile uint8_t  g_mode       = cfg::kDefaultMode;  // 可运行中修改: 0速度/1位置
volatile uint8_t  g_state      = kStateIdle;
volatile uint32_t g_fault_code = 0;   // 0正常 1CAN超时 2过温
volatile uint32_t g_rx_count   = 0;   // 收到的反馈帧数
volatile uint32_t g_tx_count   = 0;   // 发出的控制帧数

// 调试/示波用信号(可送 VOFA+)
volatile float g_dbg_ref   = 0.0f;   // 当前参考(速度模式: rad/s, 位置模式: rad)
volatile float g_dbg_fdb   = 0.0f;   // 当前反馈(同上量纲)
volatile float g_dbg_phase = 0.0f;   // 轨迹相位 rad
volatile float g_dbg_u     = 0.0f;   // 电压给定

/* ===========================================================================
 *  内部状态
 * =========================================================================*/

static Gm6020 g_motor(cfg::kMotorId);

static Pid g_vel_pid(cfg::kVelKp, cfg::kVelKi, cfg::kVelKd,
                     cfg::kVoltMax, -cfg::kVoltMax);
static Pid g_pos_pid(cfg::kPosKp, cfg::kPosKi, cfg::kPosKd,
                     cfg::kMaxVelCmd, -cfg::kMaxVelCmd);

static uint32_t g_last_rx_ms = 0;   // 最近一次收到反馈的时刻
static uint32_t g_boot_ms    = 0;   // 上电时刻
static float    g_phase      = 0.0f; // 轨迹相位累加器

/* ===========================================================================
 *  内部函数
 * =========================================================================*/

static inline float clampf(float x, float lo, float hi) {
    return (x > hi) ? hi : ((x < lo) ? lo : x);
}

// 输出置零(同时把积分清掉, 避免下次使能时残留)
static void OutputZero(void) {
    g_motor.setVoltage(0);
    g_dbg_u = 0.0f;
}

// 进入 FAULT 并锁存
static void EnterFault(uint32_t code) {
    if (g_state != kStateFault) {
        g_state      = kStateFault;
        g_fault_code = code;
        g_pos_pid.reset();
        g_vel_pid.reset();
        OutputZero();
    }
}

// 进入 RUNNING: 对齐相位, 保证使能瞬间参考值等于当前状态, 避免阶跃冲击
static void EnterRunning(void) {
    g_pos_pid.reset();
    g_vel_pid.reset();

    if (g_mode == cfg::kModeVelocity) {
        // 速度模式: sin(0) = 0 -> 参考速度从 0 平滑起步
        g_phase = 0.0f;
    } else {
        // 位置模式: 取 asin(当前角度/幅值), 结果落在 [-pi/2, pi/2],
        // 保证 cos >= 0, 即从当前位置沿正方向起步, 参考角无阶跃
        const float s = clampf(g_motor.angle() / cfg::kPosAmp, -1.0f, 1.0f);
        g_phase = std::asinf(s);
    }
    g_state = kStateRunning;
}

// 轨迹推进 + 控制律, 每个 1kHz 节拍调用一次
static void Step(void) {
    const uint8_t mode = g_mode;
    const float   w    = cfg::kTwoPi * cfg::Freq(mode);

    // ---- 相位累加(用固定 dt, 与硬件定时器一致) ----
    g_phase += w * cfg::kDt;
    if (g_phase >= cfg::kTwoPi) {
        g_phase -= cfg::kTwoPi;
    }

    const float s = std::sinf(g_phase);
    const float c = std::cosf(g_phase);

    float u = 0.0f;

    if (mode == cfg::kModeVelocity) {
        // ---- 速度曲线跟踪: 单速度环 ----
        const float v_ref = cfg::kVelAmp * s;            // 参考速度
        const float a_ff  = cfg::kVelAmp * w * c;        // 解析求导得到的加速度

        u = g_vel_pid.calc(v_ref, g_motor.vel())
          + cfg::kVelFfV * v_ref                         // 速度前馈 [电压/(rad/s)]
          + cfg::kVelFfA * a_ff;                         // 加速度前馈 [电压/(rad/s²)]

        g_dbg_ref = v_ref;
        g_dbg_fdb = g_motor.vel();
    } else {
        // ---- 位置曲线跟踪: 位置环 + 速度环 串级 ----
        const float theta_ref = cfg::kPosAmp * s;        // 参考位置
        const float v_ff      = cfg::kPosAmp * w * c;    // 参考速度(解析求导)
        const float a_ff      = -cfg::kPosAmp * w * w * s; // 参考加速度(二阶导)

        // 位置环输出速度给定, 再叠加速度前馈(无量纲)
        const float v_cmd = g_pos_pid.calc(theta_ref, g_motor.angle())
                          + cfg::kPosFfV * v_ff;

        // 速度环输出电压, 再叠加加速度前馈
        u = g_vel_pid.calc(v_cmd, g_motor.vel())
          + cfg::kPosFfA * a_ff;    // [电压/(rad/s²)]

        g_dbg_ref = theta_ref;
        g_dbg_fdb = g_motor.angle();
    }

    // ---- 输出限幅 ----
    u = clampf(u, -cfg::kVoltMax, cfg::kVoltMax);
    g_motor.setVoltage(static_cast<int16_t>(u));
    g_dbg_u = u;
}

// 把本电机电压打包进 8 字节控制帧并发出
// 注意: 0x1FF 一帧可带 4 个电机, 0x2FF 可带 3 个; 本工程只用一个,
//       其余槽位保持 0 (即 0V), 多电机时应共用一个帧再发一次。
static void SendControlFrame(void) {
    uint8_t frame[8] = {0};
    g_motor.encode(frame);
    CAN_Send_Msg(cfg::kCan, frame, g_motor.txId(), 8);
    g_tx_count = g_tx_count + 1u;   // 不用 ++: C++20 对 volatile 的 ++ 已弃用
}

/* ===========================================================================
 *  对外接口
 * =========================================================================*/

extern "C" void ControlTaskInit(void) {
    // ---- CAN 过滤器 + 启动 + 打开接收中断 ----
    CanFilter_Init(cfg::kCan);
    HAL_CAN_Start(cfg::kCan);
    HAL_CAN_ActivateNotification(cfg::kCan, CAN_IT_RX_FIFO0_MSG_PENDING);

    // ---- 状态复位 ----
    g_boot_ms    = HAL_GetTick();
    g_last_rx_ms = g_boot_ms;
    g_phase      = 0.0f;
    g_rx_count   = 0;
    g_tx_count   = 0;
    g_fault_code = 0;
    g_state      = kStateIdle;
    g_mode       = cfg::kDefaultMode;

    OutputZero();

    // ---- 启动 1kHz 控制定时器 ----
    HAL_TIM_Base_Start_IT(&htim6);
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    // 只认本电机的反馈帧 (0x205 ~ 0x20B)
    if (std_id == g_motor.rxId()) {
        g_motor.decode(data);
        g_last_rx_ms = HAL_GetTick();
        g_rx_count = g_rx_count + 1u;   // 不用 ++: C++20 对 volatile 的 ++ 已弃用
    }
}

extern "C" void MainTask(void) {
    tick++;

    const uint32_t now = HAL_GetTick();

    // ---------------- 第 1 层保护: CAN 反馈超时 ----------------
    // 反馈断了就立刻断输出, 并退回 IDLE 等待恢复(可自恢复, 因为输出一直是 0)
    const bool rx_alive = (uint32_t)(now - g_last_rx_ms) <= cfg::kCanTimeoutMs;

    // ---------------- 第 2 层保护: 过温 ----------------
    // 过温锁存 FAULT, 降温到 kOverTempC - kOverTempHys 以下才允许恢复
    if (g_motor.temp() >= cfg::kOverTempC) {
        EnterFault(2);
    }

    if (g_state == kStateFault) {
        // 过温恢复判据
        if (g_fault_code == 2 && g_motor.temp() < (cfg::kOverTempC - cfg::kOverTempHys)) {
            g_fault_code = 0;
            g_state      = kStateIdle;
            g_boot_ms    = now;      // 重新计时使能延时
        }
        OutputZero();
    } else if (!rx_alive) {
        // 反馈超时: 退到 IDLE, 输出 0
        if (g_state != kStateIdle) {
            g_pos_pid.reset();
            g_vel_pid.reset();
        }
        g_fault_code = 1;
        g_state      = kStateIdle;
        g_boot_ms    = now;          // 恢复后重新等待使能延时
        OutputZero();
    } else {
        g_fault_code = 0;

        if (g_state == kStateIdle) {
            // 反馈正常且过了使能延时, 才允许进入跟踪
            if ((uint32_t)(now - g_boot_ms) >= cfg::kArmDelayMs) {
                EnterRunning();
            } else {
                OutputZero();
            }
        }

        if (g_state == kStateRunning) {
            Step();
        }
    }

    // 无论什么状态都持续发控制帧: 让电调知道主控还活着, 且给定为 0V
    SendControlFrame();

    // ---------------- 第 3 层保护: IWDG ----------------
    // 放在 MainTask 末尾: 只有本次 1kHz 控制周期完整跑完才喂狗。
    // 若控制任务卡死/TIM6 停摆, 约 0.8s 后 IWDG 复位 MCU, 电机随之失去控制帧。
    //
    // 局限: 只覆盖了控制任务本身, 没覆盖主循环。要做成严格的多任务监督,
    //       应把喂狗移到主循环, 并检查本函数的计数是否在推进。
    HAL_IWDG_Refresh(&hiwdg);
}
