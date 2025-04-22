#pragma once


#include "cgp/cgp.hpp"
#include "environment.hpp"

#include "animated_character/animated_character.hpp"
#include "effects/effects.hpp"

#include "animated_character/motion_lines/motion/motion.hpp"
#include "animated_character/motion_lines/motion/direction.hpp"
#include "animated_character/motion_lines/motion/cue.hpp"
#include "animated_character/motion_lines/line.hpp"
#include "animated_character/motion_lines/line2D.hpp"

#include "helpers/camera_projecter.hpp"
#include "helpers/error_calculator.hpp"
#include "helpers/anim_saver.hpp"

using cgp::mesh_drawable;


struct gui_parameters {
	bool display_frame = false;
	bool display_wireframe = false;
	bool display_surface = true;
	bool display_ground = false;
	bool display_texture = true;
	bool display_skeleton = false;
	bool display_skeleton_joint_sphere = true;
	bool display_skeleton_joint_frame = false;
	bool display_skeleton_bone = true;
	bool rotate_head_effect_active = false;
	bool sketch_mode = false;
	bool constraint = false;
	int selected_motion = -1; // -1 : none, other < motions.size() : local, motions.size() : global
	bool play_anim = false;
};




// The structure of the custom scene
struct scene_structure : cgp::scene_inputs_generic {

	bool old_sketch_mode = false;
	bool is_action = true;
	bool is_cue = false;
	bool is_impact = false;
	bool delete_mode = false;
	bool camera_lock;
	vec3 pos_to_lock = vec3(0.f,0.f,0.f);
	vec3 center_to_lock = vec3(0.f,0.f,0.f);
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;
	window_structure window;

	mesh_drawable global_frame;          // The standard global frame
	environment_structure environment;   // Standard environment controler
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                  // Standard GUI element storage
	bool previous_clicked_left = false;
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //

	std::map<std::string, character_structure> characters;
	std::string current_active_character;

	std::map<std::string, effect_transition_structure> effect_transition;	
	effect_walking_structure effect_walk;
	effect_ik_structure effect_ik;

	mesh_drawable sphere_ik;
	mesh_drawable ground;

	int id_to_remove = 0;

	cgp::numarray<Direction> motion_dirs;
	cgp::numarray<Cue> motion_cues;
	cgp::numarray<Motion> motions;
	Direction global_motion;
	cgp::numarray<line_structure> lines;
	cgp::numarray<line_structure> impact_lines;

	numarray<int> all_joint_ids;

	float tmp_scale;
	bool is_global_movement = false;

	// Store the curve sketched on screen. 
	//   Each new stroke (continuous click+motion of the mouse) is a new element of the buffer
	cgp::numarray<cgp::curve_drawable_dynamic_extend> sketch_drawable;
	cgp::numarray<cgp::vec2> current_line_projected_positions;
	cgp::numarray<cgp::numarray<cgp::vec2>> list_current_lines_projected;

	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();    // Standard initialization to be called before the animation loop
	void display_frame(); // The frame display to be called within the animation loop
	void display_gui();   // The display of the GUI, also called within the animation loop


	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();
	void update_root_iks();
	void order_motions_by_joint_id();
	void update_character();
	float calculate_animation_duration();
	void build_empty_motions();
	void stop_delete_mode();
	void deselect_clusters();
	int find_line_from_cluster_line(line_structure m_line);
	void push_motion_button(int i, Motion& motion);
	void switch_benchmark(bool to_global);
	void lock_camera(cgp::vec3 pos_to_lock, cgp::vec3 center_to_lock);

};


