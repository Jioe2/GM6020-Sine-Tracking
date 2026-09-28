# firmware —— 真机工程

基于 **STM32F407IGH6（RoboMaster 开发板 C 型）** 的 GM6020 正弦曲线跟随工程。

## 关键文件

| 文件 | 作用 |
|------|------|
| `Tasks/ControlTask.cpp` | **任务核心**：正弦轨迹跟踪 + 分层看门狗保护 |
| `Resources/Src/Gm6020.cpp` · `Resources/Inc/Gm6020.h` | GM6020 CAN 驱动（控制帧打包 / 反馈帧解析） |
| `Resources/Src/HW_can.cpp` · `Resources/Inc/HW_can.hpp` | CAN 收发封装 |
| `components/pid/Pid.cpp` · `Pid.h` | PID 组件（位置环 / 速度环） |
| `Core/Src/iwdg.c` | 看门狗初始化（Prescaler 64, Reload 400 → 约 0.64s） |
| `Core/Src/can.c` | CAN 外设初始化 |
| `C_project.ioc` | CubeMX 工程配置（**看门狗在此默认开启**） |

## 实现要点

**两种曲线（在 `ControlTask.cpp` 的 `namespace cfg` 里自定义）**

- 速度曲线：`v_ref(t) = kVelAmp · sin(2π·kVelFreq·t)`
- 位置曲线：`θ_ref(t) = kPosAmp · sin(2π·kPosFreq·t)`
- `g_mode`：0 = 速度曲线，1 = 位置曲线

**控制结构（1 kHz，TIM6 触发）**

```
速度模式：速度环(PI) → 电压（叠加解析前馈）
位置模式：位置环(P) → 速度环(PI) → 电压（叠加速度/加速度前馈）
```

**三层保护（各自独立）**

1. CAN 反馈超时（20 ms）→ 输出置零，可自恢复
2. 过温（70℃）→ 锁存 FAULT，降温后解除
3. IWDG（约 0.64 s）→ 1 kHz 控制周期完整跑完才喂狗

## 编译与烧录

工具链：**ARM GNU Toolchain + CMake + Ninja**，烧录用 **OpenOCD + J-Link**。

用 VS Code 打开本文件夹，`Ctrl+Shift+P` → `Run Task`，依次运行：

1. `CMake: 配置 (Debug)`
2. `生成` → 产出 `build/project.elf`
3. `烧录 (OpenOCD + J-Link)`

> 若烧录报 `No J-Link device found`，先运行 `J-Link 状态` 检查探针。

## 参数调整

见 `Tasks/ControlTask.cpp` 顶部 `namespace cfg`：

- 曲线：`kVelAmp` / `kVelFreq` / `kPosAmp` / `kPosFreq`
- 增益：`kVelKp/Ki/Kd`、`kPosKp/Ki/Kd`
- 前馈：`kVelFfV/FfA`、`kPosFfV/FfA`
- 限幅与保护阈值：`kVoltMax`、`kCanTimeoutMs`、`kOverTempC` 等

> ⚠️ 电压给定（-25000~25000）量纲特殊，速度环增益需真机实测整定。
