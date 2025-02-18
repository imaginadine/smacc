
#pragma once

#include "cgp/cgp.hpp"

struct quaternion_dual
{
    cgp::quaternion q;
    cgp::quaternion qe;

    quaternion_dual();
    quaternion_dual(cgp::quaternion const& q, cgp::quaternion const qe);
    quaternion_dual(cgp::quaternion const& q, cgp::vec3 const& tr);

    cgp::vec3 translation() const;
};

quaternion_dual& operator+=(quaternion_dual& a, quaternion_dual const& b);
quaternion_dual operator+(quaternion_dual const& a, quaternion_dual const& b);
quaternion_dual operator*(float s, quaternion_dual const& d);
quaternion_dual operator/(quaternion_dual const& d, float s);