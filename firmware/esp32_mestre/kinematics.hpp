#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include "config.hpp"

namespace eb15 {
constexpr float PI = 3.14159265358979323846F;
using JointVector = std::array<float, 6>;
using Vec3 = std::array<float, 3>;
using Mat3 = std::array<std::array<float, 3>, 3>;
struct Pose { Vec3 position{}; Mat3 rotation{}; };
enum class IkStatus { Ok, Unreachable, Singular };

inline Mat3 multiply(const Mat3 &a, const Mat3 &b) {
    Mat3 c{};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j)
        for (int k=0;k<3;++k) c[i][j] += a[i][k]*b[k][j];
    return c;
}
inline Mat3 transpose(const Mat3 &a) {
    Mat3 t{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j) t[i][j]=a[j][i]; return t;
}
inline Mat3 rz(float q) { const float c=std::cos(q),s=std::sin(q); return {{{c,-s,0},{s,c,0},{0,0,1}}}; }
inline Mat3 ry(float q) { const float c=std::cos(q),s=std::sin(q); return {{{c,0,s},{0,1,0},{-s,0,c}}}; }

inline Pose forward_kinematics(const JointVector &q) {
    const float q23=q[1]+q[2];
    const float radial=LINK_L2*std::cos(q[1])+LINK_L3*std::cos(q23);
    const Vec3 wc={std::cos(q[0])*radial,std::sin(q[0])*radial,
                   LINK_L1+LINK_L2*std::sin(q[1])+LINK_L3*std::sin(q23)};
    const Mat3 r03=multiply(rz(q[0]),ry(q23));
    const Mat3 r36=multiply(multiply(rz(q[3]),ry(q[4])),rz(q[5]));
    const Mat3 r=multiply(r03,r36);
    return {{wc[0]+LINK_L6*r[0][2],wc[1]+LINK_L6*r[1][2],wc[2]+LINK_L6*r[2][2]},r};
}

inline IkStatus inverse_kinematics(const Pose &pose, JointVector &q) {
    const float wx=pose.position[0]-LINK_L6*pose.rotation[0][2];
    const float wy=pose.position[1]-LINK_L6*pose.rotation[1][2];
    const float wz=pose.position[2]-LINK_L6*pose.rotation[2][2];
    const float radial=std::hypot(wx,wy), z=wz-LINK_L1;
    if (radial < 1e-6F) return IkStatus::Singular;
    const float raw=(radial*radial+z*z-LINK_L2*LINK_L2-LINK_L3*LINK_L3)/(2*LINK_L2*LINK_L3);
    if (raw < -1.00001F || raw > 1.00001F) return IkStatus::Unreachable;
    const float c3=std::clamp(raw,-1.0F,1.0F);
    q[0]=std::atan2(wy,wx); q[2]=-std::acos(c3);
    q[1]=std::atan2(z,radial)-std::atan2(LINK_L3*std::sin(q[2]),LINK_L2+LINK_L3*std::cos(q[2]));
    const Mat3 r03=multiply(rz(q[0]),ry(q[1]+q[2]));
    const Mat3 r36=multiply(transpose(r03),pose.rotation);
    q[4]=std::acos(std::clamp(r36[2][2],-1.0F,1.0F));
    if (std::fabs(std::sin(q[4])) <= 1e-5F) {
        q[3]=0; q[5]=std::atan2(r36[1][0],r36[0][0]);
        return IkStatus::Singular;
    }
    q[3]=std::atan2(r36[1][2],r36[0][2]);
    q[5]=std::atan2(r36[2][1],-r36[2][0]);
    return IkStatus::Ok;
}

inline bool within_soft_limits_deg(const JointVector &q) {
    for(std::size_t i=0;i<q.size();++i)
        if(q[i]<LIMIT_MIN_DEG[i] || q[i]>LIMIT_MAX_DEG[i]) return false;
    return true;
}
}

