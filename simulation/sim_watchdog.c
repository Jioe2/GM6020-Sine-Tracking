/* =============================================================
 * 看门狗（Watchdog）仿真演示 —— 不需要任何硬件
 *
 * 作用：在电脑上模拟"看门狗保护"的完整过程：
 *       正常运行 -> 喂狗 -> 程序卡死 -> 看门狗超时 -> 自动复位 -> 恢复
 *
 * 运行：打开 https://www.onlinegdb.com/ ，整段粘贴，点 Run
 *
 * 可改参数：
 *   WDG_TIMEOUT ：看门狗超时时间（多少个 tick 不喂就复位）
 *   LOOP_PERIOD ：主循环周期（正常情况每多少 tick 喂一次狗）
 *   fault_tick[]：在哪些时刻注入"卡死"故障
 * ============================================================= */

#include <stdio.h>

/* ---------------- 可调参数 ---------------- */
#define WDG_TIMEOUT  100     /* 超时：100 个 tick 不喂狗就复位 */
#define LOOP_PERIOD   20     /* 主循环：每 20 个 tick 跑一圈（正常时喂狗） */
#define TOTAL_TICKS 1200     /* 仿真总时长 */

/* 在哪些 tick 注入"卡死"故障（可改） */
static int fault_tick[] = { 300, 800 };
static int fault_num    = 2;

int main(void)
{
    int wdg_count  = WDG_TIMEOUT;  /* 看门狗计数器（每个 tick 递减） */
    int last_feed  = 0;            /* 上次喂狗的时刻 */
    int stuck      = 0;            /* 是否处于"卡死"状态 */
    int fault_idx  = 0;            /* 已注入了几个故障 */
    int reset_count = 0;           /* 复位次数 */

    printf("================================================\n");
    printf("  看门狗（Watchdog）仿真演示\n");
    printf("  超时 = %d tick ，主循环周期 = %d tick\n", WDG_TIMEOUT, LOOP_PERIOD);
    printf("  故障注入时刻: ");
    for (int i = 0; i < fault_num; i++) printf("%d ", fault_tick[i]);
    printf("\n================================================\n\n");
    printf("[系统启动]\n");

    for (int tick = 0; tick <= TOTAL_TICKS; tick++) {

        /* ---------- 主循环 ---------- */
        if (tick - last_feed >= LOOP_PERIOD) {
            last_feed = tick;

            if (!stuck) {
                /* 正常：喂狗 */
                wdg_count = WDG_TIMEOUT;
                printf("tick %4d : 主循环正常 -> 喂狗\n", tick);

                /* 到点就注入一次故障 */
                if (fault_idx < fault_num && tick >= fault_tick[fault_idx]) {
                    fault_idx++;
                    stuck = 1;
                    printf("tick %4d : >>>>>>>>>> 程序卡死！喂不了狗 <<<<<<<<<<\n", tick);
                }
            } else {
                /* 卡死：想喂也喂不了 */
                printf("tick %4d : 程序卡死中，无法喂狗\n", tick);
            }
        }

        /* ---------- 看门狗每个 tick 递减 ---------- */
        if (wdg_count > 0) wdg_count--;

        /* ---------- 减到 0 -> 复位 ---------- */
        if (wdg_count <= 0) {
            printf("\ntick %4d : !!!! 看门狗超时 -> 系统复位 !!!!\n\n", tick);
            reset_count++;
            wdg_count = WDG_TIMEOUT;   /* 复位后重新装载 */
            stuck     = 0;             /* 故障清除（重启后程序是干净的） */
            last_feed = tick;
            printf("[第 %d 次重启]\n", reset_count + 1);
        }
    }

    printf("\n================================================\n");
    printf("仿真结束：共发生 %d 次看门狗复位\n", reset_count);
    printf("结论：程序卡死后，看门狗自动复位，系统恢复运行\n");
    printf("================================================\n");

    return 0;
}
