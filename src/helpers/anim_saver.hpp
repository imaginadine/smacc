#pragma once

#include "cgp/cgp.hpp"

#include "../animated_character/motion_lines/motion/motion.hpp"
#include "../animated_character/animated_model/animated_model.hpp"
#include "../animated_character/animated_character.hpp"

void save_anim(std::string path, cgp::numarray<Motion> motions, Motion global_motion, character_structure& character);
cgp::numarray<cgp::mat4> joints_for_one_frame(float t, cgp::numarray<Motion> motions, Motion global_motion, animated_model_structure& animated_model);
cgp::numarray<int> get_ordered_joints_from_file(std::string path);