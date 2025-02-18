#pragma once

#include "cgp/cgp.hpp"
#include <iostream>

// Include Eigen
#define EIGEN_NO_DEBUG
#include "../../third_party/eigen/Eigen/Sparse"

struct circle_2D {
    float radius;
    cgp::vec2 center;
};

circle_2D find_circle(cgp::numarray<cgp::vec2> points);