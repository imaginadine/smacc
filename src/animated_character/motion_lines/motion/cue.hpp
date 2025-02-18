#pragma once

#include "cgp/cgp.hpp"

#include "../line.hpp"
#include "motion.hpp"

class Cue : public Motion {

    public:
        Cue() = default;
        Cue(line_structure line, int id, int method_to_give) : Motion(line, id, method_to_give) {};

        void find_positions(skeleton_structure skeleton, cgp::vec3 t_source, cgp::camera_projection_perspective const& P, cgp::mat4 const& camera_view_inverse) override;
        static void merge_cue_motions(cgp::numarray<Cue>& cue_motions, cgp::numarray<int> dir_ids, skeleton_structure skeleton);
};