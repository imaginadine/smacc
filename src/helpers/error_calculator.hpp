#pragma once

#include "cgp/cgp.hpp"

#include "../animated_character/motion_lines/motion/motion.hpp"
#include "../animated_character/animated_model/animated_model.hpp"
#include "../animated_character/animated_character.hpp"


float mean_error(cgp::numarray<float> angles_ref, cgp::numarray<float> angles_drawn);
float compare_for_one_frame(float t, std::string anim, cgp::numarray<Motion> motions, Motion global_motion, animated_model_structure& animated_model);
float compare_for_all_anim(std::string anim, cgp::numarray<Motion> motions, Motion global_motion, character_structure& character);
