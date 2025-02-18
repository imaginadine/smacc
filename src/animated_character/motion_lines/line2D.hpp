#pragma once

#include "cgp/cgp.hpp"
#include "../animated_model/animated_model.hpp"
#include "../controller_skinning/controller_skinning.hpp"
#include "../../helpers/geometric_tester.hpp"
#include "line.hpp"


float cross2D (cgp::vec2 v0, cgp::vec2 v1);
bool in_list (int nb, cgp::numarray<int> list);
int most_dependant_joint(int vertex_id, controller_skinning_structure controller_skinning);
int find_joint_from_2D_line(cgp::numarray<cgp::vec2> projected_positions, animated_model_structure model, cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view, float& depth_to_find);

bool is_wrapping_object(cgp::numarray<cgp::vec2> projected_positions, animated_model_structure animated_model, cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view);

struct distance_2D {
	int vertex_id;
	float dist;
};

int get_closest_line_id_2D(cgp::vec2 pos, cgp::numarray<line_structure> lines, cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view);