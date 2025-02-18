#pragma once

#include "cgp/cgp.hpp"
#include "../../helpers/camera_projecter.hpp"

enum class Line_type { DirT, CueT, ImpT, GlobT };

struct line_structure {
	
    int joint_id;
    cgp::vec3 pos_impact; // impact_line
    Line_type type_motion;

	cgp::numarray<cgp::vec3> samples;
    float thickness;

    cgp::numarray<cgp::vec3> triangle_points;
    cgp::numarray<cgp::vec3> triangle_normals;
    cgp::numarray<cgp::vec3> triangle_colors;
    cgp::triangles_drawable line_tris;

    cgp::numarray<cgp::vec2> projected_samples;
    float depth_2D;
	
    void init_line(cgp::numarray<cgp::vec3> positions, cgp::numarray<cgp::vec2> positions_2d, float depth, cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view_inverse);
    void clear();
    float get_thickness();
    void set_thickness(float value);
    float get_length();
    cgp::vec3 angular_velocity(cgp::vec3& mean_axis, float& mean_magnitude);
    bool contains_changes(float threshold);
    bool is_random(float radius, cgp::vec2 center);

    void set_color(cgp::vec3 color);
    cgp::vec3 get_color();

    bool equals(line_structure other_line);
};

template <typename SCENE>
void draw(line_structure line, SCENE const& scene);

template <typename SCENE>
void draw(line_structure line, SCENE const& scene)
{
    //draw
    draw(line.line_tris, scene);
}

cgp::numarray<line_structure> remove_at_index(cgp::numarray<line_structure> lines, int id);