/* =============================================================
 * GM6020 正弦跟随 —— 电脑仿真版（不需要任何硬件）
 *
 * 作用：在电脑上模拟"电机跟随目标"的完整控制过程，
 *       让你在没硬件的时候也能学会 PID + 正弦跟随。
 *
 * 运行方式（二选一）：
 *   1) 打开 https://www.onlinegdb.com/ ，把这整个文件粘进去，点 Run
 *   2) 本地 gcc：  gcc sim_gm6020.c -o sim -lm  &&  ./sim
 *
 * 输出：一张"示波器"曲线
 *   R = 目标
 *   A = 实际
 *   X = 两者重合
 *   横轴 = 时间，纵轴 = 数值
 *
 * 三种模式（改 MODE）：
 *   1 = 速度正弦跟随
 *   2 = 位置正弦跟随（串级 PID）
 *   3 = 阶跃响应测试（看超调 / 震荡，"暴露参数是否太激进"）
 *
 * 参数说明（都改最上面的 #define）：
 *   FREQ        正弦频率(Hz)  —— 难度：越高越难跟
 *   AMP_SPEED   速度幅值      —— 难度
 *   AMP_POS     位置幅值      —— 难度
 *   V_KP/KI/KD  速度环 PID    —— 能力：真正决定跟得好不好
 *   P_KP/KI/KD  位置环 PID    —— 能力
 *   SIM_TIME    仿真时长(秒)  —— 只影响画面，不影响物理规律
 * ============================================================= */

#include <stdio.h>
#include <math.h>

#define PI   3.14159265f

/* ------------------ 可调参数 ------------------ */
#define MODE       1          /* 1=速度跟随  2=位置跟随  3=阶跃测试 */

#define DT         0.001f     /* 控制周期 1ms（真机上也用这个） */
#define SIM_TIME   6.0f       /* 仿真时长(秒)，仅 MODE 1/2 用 */
#define FREQ       0.5f       /* 正弦频率(Hz) */

#define AMP_SPEED  300.0f     /* 速度正弦幅值(度/秒) */
#define AMP_POS    90.0f      /* 位置正弦幅值(度) */

/* 速度环 PID */
#define V_KP  6.0f
#define V_KI  40.0f
#define V_KD  0.0f

/* 位置环 PID（仅 MODE=2 使用） */
#define P_KP  6.0f
#define P_KI  0.0f
#define P_KD  0.0f

/* 速度前馈（仅 MODE=2 使用）：0=关闭  1=开启
 * 前馈：既然目标是一条已知的正弦，就"提前"把需要的速度告诉电机，
 *       而不是等误差出现才反应。这是改善正弦跟随最有效的手段。 */
#define FF_VEL  0.0f   /* 速度前馈强度（0~1） */

/* 电机模型：一阶惯性，越大响应越慢 */
#define MOTOR_TAU  0.05f

/* 阶跃测试（仅 MODE=3 使用） */
#define STEP_TIME  0.2f       /* 何时施加阶跃(秒) */
#define STEP_REF   200.0f     /* 阶跃目标值 */

/* 输出方式：1=画曲线（直观）  0=输出 CSV（可复制到 Excel 画图） */
#define ASCII_PLOT 1

/* 曲线尺寸 */
#define PLOT_COLS  100        /* 横轴采样点数（时间） */
#define PLOT_ROWS  21         /* 纵轴行数（数值） */

/* ------------------ PID 结构 ------------------ */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_err;
    float out_limit;   /* 输出限幅 */
    float i_limit;     /* 积分限幅 */
} PID;

/* 一步 PID 计算 */
static float pid_step(PID *p, float target, float actual, float dt)
{
    float err = target - actual;

    /* 积分（带限幅，防止积分饱和） */
    p->integral += err * dt;
    if (p->integral >  p->i_limit) p->integral =  p->i_limit;
    if (p->integral < -p->i_limit) p->integral = -p->i_limit;

    /* 微分 */
    float d = (err - p->prev_err) / dt;
    p->prev_err = err;

    /* 输出（带限幅） */
    float out = p->kp * err + p->ki * p->integral + p->kd * d;
    if (out >  p->out_limit) out =  p->out_limit;
    if (out < -p->out_limit) out = -p->out_limit;
    return out;
}

/* 采样缓存 */
static float g_ref[PLOT_COLS];
static float g_act[PLOT_COLS];

/* 用字符画一张曲线：横轴=时间，纵轴=数值 */
static void draw_plot(float range)
{
    for (int r = 0; r < PLOT_ROWS; r++) {
        /* 本行代表的数值（上大下小） */
        float level = range - (2.0f * range) * r / (PLOT_ROWS - 1);
        /* 半个行高，作为判定带 */
        float band  = range / (PLOT_ROWS - 1);

        for (int c = 0; c < PLOT_COLS; c++) {
            int isR = fabsf(g_ref[c] - level) <= band;
            int isA = fabsf(g_act[c] - level) <= band;
            if (isR && isA)      putchar('X');   /* 重合 */
            else if (isR)        putchar('R');   /* 只有目标 */
            else if (isA)        putchar('A');   /* 只有实际 */
            else                 putchar(' ');
        }
        putchar('\n');
    }
    for (int c = 0; c < PLOT_COLS; c++) putchar('-');
    putchar('\n');
    printf("+%.0f\n", range);
}

int main(void)
{
    float motor_speed = 0.0f;   /* 当前转速（度/秒） */
    float motor_angle = 0.0f;   /* 当前角度（度）   */

    PID vpid = { V_KP, V_KI, V_KD, 0.0f, 0.0f, 2000.0f, 800.0f };
    PID ppid = { P_KP, P_KI, P_KD, 0.0f, 0.0f, 2000.0f,   0.0f };

    /* 仿真时长：阶跃测试固定 1 秒 */
    float sim_time = (MODE == 3) ? 1.0f : SIM_TIME;

    /* 画图纵轴范围 */
    float range = (MODE == 1) ? AMP_SPEED
                : (MODE == 2) ? AMP_POS
                : STEP_REF * 1.5f;

    int steps = (int)(sim_time / DT);
    int sample_every = steps / (PLOT_COLS - 1);
    if (sample_every < 1) sample_every = 1;

    float max_actual = -1e9f;   /* 用于统计超调 */
    double err_sum = 0.0;       /* 用于统计平均跟随误差 */
    int    err_n   = 0;

    int col = 0;
    for (int i = 0; i <= steps && col < PLOT_COLS; i++) {
        float t = (float)i * DT;
        float ref, actual;

#if (MODE == 1)
        /* ===== 速度正弦跟随 ===== */
        ref = AMP_SPEED * sinf(2.0f * PI * FREQ * t);

        float u = pid_step(&vpid, ref, motor_speed, DT);      /* PID：目标速度 → 电压 */
        motor_speed += (u - motor_speed) * (DT / MOTOR_TAU);  /* 电机响应 */
        motor_angle += motor_speed * DT;

        actual = motor_speed;
#elif (MODE == 2)
        /* ===== 位置正弦跟随（串级：位置环 + 速度环 + 前馈）===== */
        float w = 2.0f * PI * FREQ;
        ref           = AMP_POS * sinf(w * t);          /* 目标角度 */
        float ref_vel = AMP_POS * w * cosf(w * t);      /* 目标速度（前馈用） */

        float pos_out   = pid_step(&ppid, ref, motor_angle, DT);  /* 位置环反馈 */
        float speed_ref = pos_out + FF_VEL * ref_vel;             /* 反馈 + 速度前馈 */
        float u         = pid_step(&vpid, speed_ref, motor_speed, DT);
        motor_speed += (u - motor_speed) * (DT / MOTOR_TAU);
        motor_angle += motor_speed * DT;

        actual = motor_angle;
#else
        /* ===== 阶跃响应测试 ===== */
        ref = (t < STEP_TIME) ? 0.0f : STEP_REF;

        float u = pid_step(&vpid, ref, motor_speed, DT);
        motor_speed += (u - motor_speed) * (DT / MOTOR_TAU);

        actual = motor_speed;
#endif

        if (actual > max_actual) max_actual = actual;

        /* 统计稳态后的平均跟随误差（跳过第一个周期，避免启动瞬态干扰） */
        if (MODE != 3 && FREQ > 0.0f && t > 1.0f / FREQ) {
            err_sum += fabsf(ref - actual);
            err_n++;
        }

        if (i % sample_every == 0 && col < PLOT_COLS) {
            g_ref[col] = ref;
            g_act[col] = actual;
            col++;
        }
    }
    /* 若没填满，用最后一个值补上 */
    while (col < PLOT_COLS) {
        g_ref[col] = g_ref[col - 1];
        g_act[col] = g_act[col - 1];
        col++;
    }

#if ASCII_PLOT
    printf("\n模式: ");
#if (MODE == 1)
    printf("速度正弦跟随   频率: %.2f Hz\n", (double)FREQ);
#elif (MODE == 2)
    printf("位置正弦跟随   频率: %.2f Hz\n", (double)FREQ);
#else
    printf("阶跃响应测试   阶跃值: %.0f\n", (double)STEP_REF);
#endif
    printf("R = 目标    A = 实际    X = 重合\n\n");
    draw_plot(range);

#if (MODE != 3)
    if (err_n > 0)
        printf("\n>>> 平均跟随误差: %.2f   (越小越贴合)\n",
               err_sum / (double)err_n);
#else
    {
        float overshoot = (max_actual - STEP_REF) / STEP_REF * 100.0f;
        if (overshoot < 0.0f) overshoot = 0.0f;
        printf("\n>>> 实际峰值: %.1f   超调: %.1f%%\n",
               (double)max_actual, (double)overshoot);
        printf(">>> 超调越小越稳；超调大 = 参数太激进\n");
    }
#endif
#else
    printf("t,ref,actual\n");
    for (int c = 0; c < PLOT_COLS; c++) {
        printf("%.4f,%.3f,%.3f\n",
               c * sim_time / (PLOT_COLS - 1),
               g_ref[c], g_act[c]);
    }
#endif

    return 0;
}
