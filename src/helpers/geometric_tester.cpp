#include "geometric_tester.hpp"

using namespace cgp;

bool cross_segment(vec2 p, vec2 a, vec2 b)
{
    bool res;

    // Ensure A is below B (swap if necessary)
    if (a.y > b.y) {
        vec2 tmp = a;
        a = b;
        b = tmp;
    }

    // Check if point is outside the vertical range of the edge
    if (p.y < a.y || p.y >= b.y) 
    {
        res = false;
    } else {
        // Calculate the x-coordinate of the intersection point
        float xIntersect = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);

        // Check if the intersection is to the right of the point
        res = (xIntersect >= p.x);
    }
    
    return res;
    
}

bool is_inside_polygon(cgp::vec2 p, cgp::numarray<vec2> samples)
{
    bool res;

    int intersection_count = 0;
    int n = samples.size();

    // verify that their ends are close
    if (norm(samples[0]-samples[samples.size()-1]) > 0.1f){
        res = false;
    } else {
        for (int i = 0; i < n; i++) {
            // Get the current edge
            vec2 a = samples[i];
            vec2 b = samples[(i + 1) % n]; // Wrap around to the first vertex

            // Check if the ray intersects this edge
            if (cross_segment(p, a, b)) {
                intersection_count ++;
            }
        }

        // If the number of intersections is odd, the point is inside
        res = (intersection_count % 2 == 1);
    }


    return res;
}