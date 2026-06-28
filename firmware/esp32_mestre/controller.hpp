#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include "config.hpp"

namespace eb15 {
class PidTanhController {
public:
    float update(std::size_t axis,float target,float measured) {
        if(axis>=3) return 0;
        const float error=target-measured;
        if(std::fabs(error)<=PID_DEADBAND_DEG){integral_[axis]=0;last_[axis]=error;return 0;}
        constexpr float dt=1.0F/CONTROL_HZ;
        integral_[axis]=std::clamp(integral_[axis]+error*dt,-PID_I_LIMIT,PID_I_LIMIT);
        const float derivative=(error-last_[axis])/dt; last_[axis]=error;
        const float raw=PID_KP[axis]*error+PID_KI[axis]*integral_[axis]+PID_KD[axis]*derivative;
        return std::clamp(raw*std::tanh(PID_GAMMA*std::fabs(error)),
                          -MAX_SPEED_DEG_S[axis],MAX_SPEED_DEG_S[axis]);
    }
    void reset(){integral_.fill(0);last_.fill(0);}
private: std::array<float,3> integral_{};std::array<float,3> last_{};
};
}

