#pragma once

#include "cgp/cgp.hpp"

/*
    Tells if a ray starting from p and going to the right crosses the segment ab
    p : the point where the ray starts
    a : one point of the segment
    b : the other point of the segment
    Returns true if it crosses, else false
*/
bool cross_segment(cgp::vec2 p, cgp::vec2 a, cgp::vec2 b);

/*
    Verify if a point is inside a polygon defined by points
    p : the point to apply on the inside outside test
    samples: the points of the polygon
*/
bool is_inside_polygon(cgp::vec2 p, cgp::numarray<cgp::vec2> samples);