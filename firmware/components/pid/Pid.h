#ifndef PID_H
#define PID_H

// 位置式 PID 控制器
//
// 离散形式:
//   u(k) = Kp*e(k) + Ki*sum(e(j))*T + Kd*(e(k)-e(k-1))/T
// 其中 T 为采样周期(本工程固定 1kHz -> 0.001s)
//
// 特性: 输出限幅 + 积分抗饱和(条件积分)
class Pid {
public:
    Pid(float kp, float ki, float kd, float out_max, float out_min);
    ~Pid() = default;

    void setParams(float kp, float ki, float kd);
    void setOutputLimit(float out_max, float out_min);
    void reset();

    // 计算一次控制量, ref 目标值, fdb 反馈值, 返回限幅后的输出
    float calc(float ref, float fdb);

    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }

private:
    float kp_, ki_, kd_;
    float out_max_, out_min_;
    float integ_;      // 误差累积 sum(e(j))
    float last_err_;   // 上一次误差 e(k-1)
};

#endif // PID_H
