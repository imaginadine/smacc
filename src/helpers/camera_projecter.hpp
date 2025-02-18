#pragma once

#include "cgp/cgp.hpp"

cgp::vec3 unproject(cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view_inverse, cgp::vec2 const& p_screen, float depth);
cgp::vec3 unproject(cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view_inverse, cgp::vec2 const& p_screen);