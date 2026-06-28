#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "config.hpp"

namespace eb15 {
struct MotionState { float position; float velocity; float acceleration; float jerk; };
struct SCurveProfile {
    float distance{}, jerk{};
    std::array<float,7> phase{};
    float duration{};
    bool valid{};
};

inline SCurveProfile plan_scurve(float distance, std::size_t axis, float speed_fraction=1.0F) {
    SCurveProfile p{}; p.distance=distance;
    const float d=std::fabs(distance);
    if(d<1e-7F || axis>=6) return p;
    const float scale=std::clamp(speed_fraction,0.05F,1.0F);
    const float vmax=MAX_SPEED_DEG_S[axis]*scale;
    const float amax=MAX_ACCEL_DEG_S2[axis]*scale;
    p.jerk=MAX_JERK_DEG_S3[axis];
    float tj=std::min(amax/p.jerk,std::sqrt(vmax/p.jerk));
    float ta=std::max(0.0F,vmax/amax-tj);
    float peak=p.jerk*tj*(tj+ta);
    const float ramp=peak*(2*tj+ta);
    float cruise=0;
    if(d<ramp) {
        const float threshold=2*p.jerk*std::pow(amax/p.jerk,3);
        if(d<=threshold){tj=std::cbrt(d/(2*p.jerk));ta=0;}
        else {tj=amax/p.jerk;ta=(-3*tj+std::sqrt(tj*tj+4*d/amax))/2;}
        peak=p.jerk*tj*(tj+ta);
    } else cruise=(d-ramp)/peak;
    p.phase={tj,ta,tj,cruise,tj,ta,tj};
    for(float dt:p.phase) p.duration+=dt;
    p.valid=p.duration>0 && std::isfinite(p.duration);
    return p;
}

inline MotionState sample_scurve(const SCurveProfile &p,float time) {
    if(!p.valid) return {};
    const float sign=p.distance<0?-1.0F:1.0F;
    const std::array<float,7> j={p.jerk,0,-p.jerk,0,-p.jerk,0,p.jerk};
    float x=0,v=0,a=0,remaining=std::clamp(time,0.0F,p.duration);
    for(std::size_t i=0;i<7 && remaining>0;++i){
        const float dt=std::min(remaining,p.phase[i]);
        x+=v*dt+0.5F*a*dt*dt+j[i]*dt*dt*dt/6.0F;
        v+=a*dt+0.5F*j[i]*dt*dt; a+=j[i]*dt; remaining-=dt;
    }
    float active_jerk=0,elapsed=0;
    for(std::size_t i=0;i<7;++i){elapsed+=p.phase[i];if(time<elapsed){active_jerk=j[i];break;}}
    if(time>=p.duration){x=std::fabs(p.distance);v=0;a=0;active_jerk=0;}
    return {sign*x,sign*v,sign*a,sign*active_jerk};
}

struct TrajectorySegment { std::array<float,6> target_deg{}; uint16_t duration_ms{}; };
template<std::size_t Capacity> class SegmentQueue {
public:
    bool push(const TrajectorySegment &s){const auto n=(head_+1)%Capacity;if(n==tail_)return false;data_[head_]=s;head_=n;return true;}
    bool pop(TrajectorySegment &s){if(empty())return false;s=data_[tail_];tail_=(tail_+1)%Capacity;return true;}
    bool empty()const{return head_==tail_;}
    void clear(){tail_=head_;}
private: std::array<TrajectorySegment,Capacity> data_{};std::size_t head_{},tail_{};
};
}

