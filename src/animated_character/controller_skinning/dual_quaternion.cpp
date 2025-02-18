#include "dual_quaternion.hpp"

using namespace cgp;

quaternion_dual::quaternion_dual()
    :q(), qe()
{}
quaternion_dual::quaternion_dual(quaternion const& q_arg, quaternion const qe_arg)
    :q(q_arg),qe(qe_arg)
{}

quaternion_dual::quaternion_dual(quaternion const& q_arg, vec3 const& t)
    :q(q_arg), qe()
{
    qe = 0.5f * quaternion(t.x, t.y, t.z, 0.0f) * q_arg;
//    qe.x = 0.5f * ( t.x*q.w + t.y*q.z - t.z*q.y);
//    qe.y = 0.5f * (-t.x*q.z + t.y*q.w + t.z*q.x);
//    qe.z = 0.5f * ( t.x*q.y - t.y*q.x + t.z*q.w);
//    qe.w = 0.5f * (-t.x*q.x - t.y*q.y - t.z*q.z);
}

vec3 quaternion_dual::translation() const
{
    quaternion q_t = 2 * qe * conjugate(q);
    return {q_t.x, q_t.y, q_t.z};

//    float const tx = 2.0f * ( -qe.w*q.x + qe.x*q.w - qe.y*q.z + qe.z*q.y );
//    float const ty = 2.0f * ( -qe.w*q.y + qe.x*q.z + qe.y*q.w - qe.z*q.x );
//    float const tz = 2.0f * ( -qe.w*q.z - qe.x*q.y + qe.y*q.x + qe.z*q.w );
//    return {tx,ty,tz};
}

quaternion_dual& operator+=(quaternion_dual& a, quaternion_dual const& b)
{
    a.q += b.q;
    a.qe += b.qe;
    return a;
}
quaternion_dual operator+(quaternion_dual const& a, quaternion_dual const& b)
{
    return {a.q+b.q, a.qe+b.qe};
}
quaternion_dual operator*(float s, quaternion_dual const& d)
{
    return {s*d.q, s*d.qe};
}
quaternion_dual operator/(quaternion_dual const& d, float s)
{
    return {d.q/s, d.qe/s};
}

