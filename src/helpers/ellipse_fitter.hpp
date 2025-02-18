#pragma once

#include "cgp/cgp.hpp"
#include <iostream>

// Include Eigen
#define EIGEN_NO_DEBUG
#include "../../third_party/eigen/Eigen/Dense"

void computeGradientAndHessian(const cgp::vec2& p, float a, float b, float theta, float& gradient, float& hessian);
cgp::vec2 find_nearest_point_ellipse(float a, float b, cgp::vec2 p);