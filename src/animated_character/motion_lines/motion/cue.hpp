#pragma once

#include "cgp/cgp.hpp"

#include "../line.hpp"
#include "motion.hpp"

class Cue : public Motion {

    public:
        Cue() = default;
        Cue(line_structure line, int id) : Motion(line, id) {};

        void find_positions(skeleton_structure skeleton, cgp::vec3 t_source) override;
        static void merge_cue_motions(cgp::numarray<Cue>& cue_motions, cgp::numarray<int> dir_ids, skeleton_structure skeleton);
};