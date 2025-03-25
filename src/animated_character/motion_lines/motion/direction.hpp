#pragma once

#include "cgp/cgp.hpp"

#include "../line.hpp"
#include "motion.hpp"
#include "../../animated_model/animated_model.hpp"


class Direction : public Motion {

    public:
    
        Direction() = default;
        Direction(line_structure line, int id) : Motion(line, id) {};

        static void merge_dir_motions(cgp::numarray<Direction>& motions, skeleton_structure skeleton, cgp::numarray<int> cue_ids);
        void find_positions(skeleton_structure skeleton, cgp::vec3 t_source) override;
        void find_positions_global(skeleton_structure skeleton, cgp::vec3 t_source);

        void find_after_joints(animated_model_structure& animated_model);

        cgp::numarray<line_structure> is_impacted(cgp::numarray<line_structure> impact_lines, skeleton_structure skeleton);
        void update_impact(cgp::numarray<line_structure> impacting_lines, animated_model_structure& animated_model, bool is_global);
        void check_impact(cgp::numarray<line_structure> impact_lines, animated_model_structure& animated_model);
        void check_impact_global(cgp::numarray<line_structure> impact_lines, cgp::numarray<Direction> dir_motions, animated_model_structure& animated_model);
        static void update_dirs_with_impacts(cgp::numarray<Direction>& dir_motions, cgp::numarray<line_structure> impact_lines, animated_model_structure& animated_model);
        void precompute_positions_with_impacts(animated_model_structure& animated_model, bool is_global);
        cgp::numarray<cgp::mat4> get_joints_in_chain(cgp::numarray<cgp::mat4> skeleton_joints);
        cgp::numarray<cgp::numarray<cgp::vec3>> compute_angle_velocities(animated_model_structure& animated_model, cgp::numarray<cgp::mat4>& root_global_joints);

};