#include "line.hpp"

using namespace cgp;


void line_structure::init_line(numarray<vec3> positions, numarray<vec2> positions_2d, float depth, camera_projection_perspective const& P, mat4 const& camera_view_inverse)
{
    projected_samples = positions_2d;
    samples = positions;
    depth_2D = depth;

    thickness = 1.5f;

    for(int i=0; i<projected_samples.size()-1;i++)
    {
        vec2 p1 = projected_samples[i];
        vec2 p2 = projected_samples[i+1];

        // Line direction
        vec2 direction = normalize(p2 - p1);
        
        // Perpendicular vector in the XY plane
        vec2 normal = vec3(-direction.y, direction.x, 0.0f);
        normal = normalize(normal) * (thickness / 800.0f);

        // Quad vertices
        vec3 v1 = unproject(P, camera_view_inverse, p1 + normal, depth); // Top-left
        vec3 v2 = unproject(P, camera_view_inverse, p1 - normal, depth); // Bottom-left
        vec3 v3 = unproject(P, camera_view_inverse, p2 + normal, depth); // Top-right
        vec3 v4 = unproject(P, camera_view_inverse, p2 - normal, depth); // Bottom-right

        // Triangle vertices - Add to the drawable object
        triangle_points.push_back(v1);
        triangle_points.push_back(v2);
        triangle_points.push_back(v3);

        triangle_points.push_back(v3);
        triangle_points.push_back(v2);
        triangle_points.push_back(v4);

    }

    triangle_normals.resize(triangle_points.size());
    vec3 to_front = vec3(0.f,0.f,1.f);
    triangle_normals.fill(to_front);

    triangle_colors.resize(triangle_points.size());
    vec3 black = vec3(0.f,0.f,0.f);
    triangle_colors.fill(black);

    line_tris.clear();
    line_tris.initialize_data_on_gpu(triangle_points, triangle_normals, triangle_colors);
}


void line_structure::set_color(vec3 color)
{
    triangle_colors.resize_clear(triangle_points.size());
    triangle_colors.fill(color);

    line_tris.clear();
    line_tris.initialize_data_on_gpu(triangle_points, triangle_normals, triangle_colors);
}

vec3 line_structure::get_color()
{
    return triangle_colors[0];
}


void line_structure::clear()
{
    line_tris.clear();
}

float line_structure::get_thickness()
{
    return thickness;
}

void line_structure::set_thickness(float value)
{
    thickness = value;
}

float line_structure::get_length()
{
    float total_length = 0.0f;
    for(int i=0; i<samples.size()-1;i++)
    {
        total_length += norm(samples[i] - samples[i+1]);
    }
    return total_length;
}

bool line_structure::contains_changes(float threshold)
{
    bool discontinued = false;
    // look for sharp turns
    for(int i=0; i<samples.size()-2;i++)
    {
        vec3 a = samples[i];
        vec3 b = samples[i+1];
        vec3 c = samples[i+2];

        float cosTheta = dot(b-a,c-b) / ( norm(b-a) * norm(c-b) );
        if (cosTheta > (1.f + threshold) || cosTheta < (1.f - threshold)) {
            // abrupt change
            discontinued = true;
            break;
        }
    }
    return discontinued;
}

bool line_structure::is_random(float radius, vec2 center)
{
    bool point_too_far = false;
    // look for sharp turns
    for(int i=0; i<projected_samples.size();i++)
    {
        vec2 p = projected_samples[i];

        if (norm(center - p) > (1.2f * radius) || norm(center - p) < (0.8f * radius)) {
            // abrupt change
            point_too_far = true;
            break;
        }
    }
    return point_too_far;
}

vec3 line_structure::angular_velocity(vec3& mean_axis, float& mean_magnitude)
{
    // compute axis, mean of cross products of tangents
    /*mean_axis = vec3(0.0f,0.0f,0.0f);
    mean_magnitude = 0.0f;

    for(int i=0; i<samples.size()-2;i++)
    {
        vec3 t0 = normalize(samples[i+1] - samples[i]);
        vec3 t1 = normalize(samples[i+2] - samples[i+1]);

        mean_axis += cross(t0,t1);

        float cosTheta = dot(t0,t1);
        cosTheta = clamp(cosTheta, -1.0f, 1.0f); // Clamp for numerical stability
        mean_magnitude += acos(cosTheta);
    }*/

    int start_point = 0;
    int end_point = samples.size()-1;
    int mid_point = samples.size()/2 - 1;

    vec3 t0 = normalize(samples[mid_point] - samples[start_point]);
    vec3 t1 = normalize(samples[end_point] - samples[mid_point]);
    mean_axis = cross(t0,t1);

    float cosTheta = dot(t0,t1);
    cosTheta = clamp(cosTheta, -1.0f, 1.0f); // Clamp for numerical stability
    mean_magnitude = acos(cosTheta);

    //mean_axis = mean_axis/(samples.size()-2);
    if(norm(mean_axis)>1e-1){
        mean_axis = normalize(mean_axis);
    } else {
        mean_axis = vec3(0.0f,0.0f,0.0f); // no rotation
    }
    //mean_magnitude = mean_magnitude/(samples.size()-2);

    return mean_magnitude * mean_axis;
}

numarray<line_structure> remove_at_index(numarray<line_structure> lines, int id)
{
    numarray<line_structure> new_lines;
    for(int k=0; k<lines.size();k++) {
        if(k != id) new_lines.push_back(lines[k]);
    }
    return new_lines;
}

bool line_structure::equals(line_structure other_line)
{
    bool res = true;
    
    if(other_line.samples.size()==samples.size()){
        for(int k=0; k<samples.size();k++){
            if(!is_equal(other_line.samples[k], samples[k])) {
                res = false;
            }
        }
    } else {
        res = false;
    }

    return res;
}