#pragma once

#include "cgp/cgp.hpp"

#include "../line.hpp"
#include "../../../helpers/least_squarer.hpp"
#include "../../../helpers/camera_projecter.hpp"
#include "../../skeleton_structure/skeleton_structure.hpp"

#include <memory>

class Motion {

	protected:

		float v;
		float a = 0.0f;
		
		cgp::vec3 vel;
		float dt = 1.0f;

	public:
		int joint_id;
		
		cgp::numarray<cgp::vec3> positions_to_follow;
		int N_pos_before; // number of positions we know, defined by the drawn line

		cgp::numarray<cgp::mat4> joints;
		cgp::numarray<float> times;
		float t_start = 0.0f;
		float t_end = 10000.0f;
		cgp::numarray<float> distances;
		float dist_total;
		int joint_root_ik = 0;
		cgp::numarray<line_structure> lines;

		cgp::numarray<cgp::numarray<cgp::mat4>> all_local_joints_after;

		std::map<int, cgp::vec3> impacts; // id joint impacted | pos the impact joint
		bool is_constrained = false;

		cgp::numarray<int> chain;

		int method = 1; // 0, 1, 2
		int old_method = 1;

		float speed(int step, cgp::numarray<float> old_times);
		float get_a();

		//static std::unique_ptr<Motion> find_type_line(line_structure line, skeleton_structure& skeleton, int id, cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view_inverse, int method_to_give);

		Motion() = default;
		Motion(line_structure line, int id, int method);
		Motion(line_structure line, skeleton_structure skeleton, int method);
		virtual ~Motion() = default;

		void clear();

		virtual void find_positions(skeleton_structure skeleton, cgp::vec3 t_source);
		void find_distances();

		void set_joint_root_ik(int id_root);
		int get_joint_root_ik();

		static void find_roots_ik(cgp::numarray<std::shared_ptr<Motion>>& motions, skeleton_structure skeleton);

		void animate_motion_to_joint(skeleton_structure& skeleton);

		cgp::mat4 evaluate(float t);
		cgp::mat4 evaluate_end(int id_joint_in_chain, float t);

		void find_chain(skeleton_structure skeleton);
		void update_skeleton(skeleton_structure& skeleton);

		line_structure get_closest_line(skeleton_structure skeleton);
		line_structure get_median_line(skeleton_structure skeleton);
		void calculate_speed();
		void add_lines(cgp::numarray<line_structure> lines_to_add);

		int get_step_from_time(float t);
		void insert_position(cgp::vec3 pos_to_insert, int step, skeleton_structure skeleton);
};

int find_motion_id(line_structure line, skeleton_structure& skeleton);
bool is_joint_parent(int child, int parent, skeleton_structure skeleton);
float sigmoid(float x, float k);
