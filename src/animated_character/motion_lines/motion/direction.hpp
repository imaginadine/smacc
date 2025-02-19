#pragma once

#include "cgp/cgp.hpp"

#include "../line.hpp"
#include "motion.hpp"
#include "../../animated_model/animated_model.hpp"


class Direction : public Motion {

    public:
    
        Direction() = default;
        Direction(line_structure line, int id, int method_to_give) : Motion(line, id, method_to_give) {};

        static void merge_dir_motions(cgp::numarray<Direction>& motions, skeleton_structure skeleton, cgp::numarray<int> cue_ids);
        void find_positions(skeleton_structure skeleton, cgp::vec3 t_source, cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view_inverse) override;
        void find_after_joints(float t_end, float dt_before, animated_model_structure& animated_model, cgp::numarray<Motion> ordered_motions, cgp::numarray<int> all_joint_ids);

        cgp::numarray<line_structure> is_impacted(cgp::numarray<line_structure> impact_lines, skeleton_structure skeleton);
        void update_impact(float t_drawn, cgp::numarray<line_structure> impacting_lines, animated_model_structure& animated_model, cgp::numarray<Motion> motions, cgp::numarray<int> all_joint_ids, bool is_global);
        void check_impact(float t_drawn, cgp::numarray<line_structure> impact_lines, animated_model_structure& animated_model, cgp::numarray<Motion> motions, cgp::numarray<int> all_joint_ids);
        void check_impact_global(float t_drawn, cgp::numarray<line_structure> impact_lines, cgp::numarray<Direction> dir_motions, animated_model_structure& animated_model, cgp::numarray<Motion> motions, cgp::numarray<int> all_joint_ids);
        static void update_dirs_with_impacts(float t_drawn, cgp::numarray<Direction>& dir_motions, cgp::numarray<line_structure> impact_lines, animated_model_structure& animated_model, cgp::numarray<Motion> motions, cgp::numarray<int> all_joint_ids);
        void precompute_positions_with_impacts(animated_model_structure& animated_model, cgp::numarray<Motion> motions, cgp::numarray<int> all_joint_ids, bool is_global);

};