#include "Gm6020.h"

namespace {
constexpr float    kPi         = 3.14159265358979323846f;
constexpr uint16_t kAngleRange = 8192;   // 机械角度一圈 8192
constexpr float    kRpmToRadS  = (2.0f * kPi) / 60.0f;
constexpr float    kRawToAmp   = 3.0f / 16384.0f; // 电流标定: 16384 对应 约3A(近似)
}

Gm6020::Gm6020(uint8_t id)
    : id_(id), voltage_(0),
      raw_angle_(0), raw_vel_rpm_(0), raw_current_(0), raw_temp_(0),
      full_turns_(0), last_raw_angle_(0), initialized_(false) {}

uint32_t Gm6020::rxId() const {
    return 0x204u + id_;   // 0x205 ~ 0x20B
}

uint32_t Gm6020::txId() const {
    return (id_ <= 4) ? 0x1FFu : 0x2FFu;   // 电压控制帧
}

void Gm6020::decode(const uint8_t *data) {
    uint16_t raw = static_cast<uint16_t>((data[0] << 8) | data[1]);

    // 跨零检测: 一圈 8192, 单拍跳变不超过半圈(4096)才认为是连续转动。
    //
    // 前提: 单拍角变化必须 < 180°。1kHz 反馈下 GM6020 满速也只有约 44 计数/拍,
    //       所以这个前提在真实运行中总是成立。
    //
    // 首帧必须特殊处理: 构造函数把 last_raw_angle_ 置 0, 若电机上电时恰好停在
    // 180°~360° 之间(如 raw=6000), 直接比较会算出 diff=6000 > 4096, 被误判成
    // "反转跨零"而使 full_turns_ 变成 -1, 累计角度整体偏移 -360°。
    // 所以第一帧只用来建立基准, 不改变 full_turns_。
    if (!initialized_) {
        initialized_    = true;
        full_turns_     = 0;
        last_raw_angle_ = raw;
    } else {
        int16_t diff = static_cast<int16_t>(raw - last_raw_angle_);
        if (diff > 4096)        full_turns_--;   // 8191 -> 0, 实际上是正转了一圈
        else if (diff < -4096)  full_turns_++;   // 0 -> 8191, 实际上是反转了一圈
        last_raw_angle_ = raw;
    }

    raw_angle_   = raw;
    raw_vel_rpm_ = static_cast<int16_t>((data[2] << 8) | data[3]);
    raw_current_ = static_cast<int16_t>((data[4] << 8) | data[5]);
    raw_temp_    = data[6];
}

float Gm6020::angle() const {
    // 累计角度 = 整圈数 * 一圈 + 当前原始角度, 再换算成弧度
    return (static_cast<float>(full_turns_) * kAngleRange + raw_angle_)
           * (2.0f * kPi / kAngleRange);
}

float Gm6020::vel() const {
    return raw_vel_rpm_ * kRpmToRadS;   // rpm -> rad/s
}

float Gm6020::current() const {
    return raw_current_ * kRawToAmp;
}

float Gm6020::temp() const {
    return static_cast<float>(raw_temp_);
}

int16_t Gm6020::velRpm() const {
    return raw_vel_rpm_;
}

void Gm6020::setVoltage(int16_t v) {
    voltage_ = v;
}

int16_t Gm6020::voltage() const {
    return voltage_;
}

void Gm6020::encode(uint8_t *data) const {
    // 电压控制帧数据布局:
    //   0x1FF: ID 1->[0,1]  2->[2,3]  3->[4,5]  4->[6,7]
    //   0x2FF: ID 5->[0,1]  6->[2,3]  7->[4,5]
    uint8_t index = (id_ <= 4) ? static_cast<uint8_t>((id_ - 1) * 2)
                               : static_cast<uint8_t>((id_ - 5) * 2);
    data[index]     = static_cast<uint8_t>((voltage_ >> 8) & 0xFF);   // 大端
    data[index + 1] = static_cast<uint8_t>(voltage_ & 0xFF);
}
