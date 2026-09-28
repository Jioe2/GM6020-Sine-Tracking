#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <stdint.h>

// 供 C 中断处理(stm32f4xx_it.c 或 HAL 弱回调)调用的接口
//
// 注意: 本头文件会被 C 代码(main.c)包含, 所以只能用 <stdint.h>,
//       不能用 <cstdint> —— 后者是 C++ 专有头文件。
#ifdef __cplusplus
extern "C" {
#endif

// 上电初始化: 配置 CAN 过滤器、启动 CAN 与 1kHz 控制定时器
// 在 main.c 中 MX_XXX_Init() 全部执行完后调用一次
void ControlTaskInit(void);

// 1kHz 控制入口, 在 TIM6 周期中断(HAL_TIM_PeriodElapsedCallback)中调用
void MainTask(void);

// CAN 接收回调, 在 HAL_CAN_RxFifo0MsgPendingCallback 中调用
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // CONTROLTASK_H
