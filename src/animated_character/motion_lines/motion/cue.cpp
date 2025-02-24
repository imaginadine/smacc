#include "cue.hpp"

using namespace cgp;


void Cue::find_positions(skeleton_structure skeleton, vec3 t_source)
{
    line_structure dir_line = lines[0];

    calculate_speed();
    std::cout<<"v = "<<v<<std::endl;

    std::cout<<"cue cue"<<std::endl;

    v = 2.f*v;
    
    int repetition = 50;
    float amplitude = 0.65f;

    vec3 correction = t_source - dir_line.samples[dir_line.samples.size()/2 - 1];

    numarray<vec3> pos;
    // well positionned points
    for(int i=0; i<dir_line.samples.size();i++){
        pos.push_back(dir_line.samples[i] + correction);
    }
    int middle = dir_line.samples.size()/2;
    // well scaled points
    for(int i=0; i<dir_line.samples.size();i++){
        vec3 vec_to_middle = pos[middle] - pos[i];
        pos[i] = pos[i] + vec_to_middle * amplitude;
    }

    for (int k=0; k<repetition;k++) {
        // forward
        for(int i=0; i<pos.size();i++){
            positions_to_follow.push_back( pos[i] );
        }
        // backward
        for(int i=0; i<pos.size();i++){
            positions_to_follow.push_back( pos[pos.size()-1-i] );
        }
    }

    // find distances between the positions
    find_distances();

}

void Cue::merge_cue_motions(numarray<Cue>& cue_motions, numarray<int> dir_ids, skeleton_structure skeleton)
{
    numarray<int> id_list;
    numarray<Cue> new_motions;
    // for every cue motion
    for (Cue cue : cue_motions) {
        // if the id has not been used yet
        if (! id_list.contains(cue.joint_id) ) {
            // and if the cue is not a parent of a direction
            bool is_parent_dir = false;
            for (int id_dir : dir_ids) {
                if (is_joint_parent(id_dir, cue.joint_id, skeleton)) {
                    is_parent_dir = true;
                    break;
                }
            }
            if (! is_parent_dir) {
                // we keep it
                cue.find_chain(skeleton);
                new_motions.push_back(cue);
                id_list.push_back(cue.joint_id);
            }
        } else {
            // if id already used, add its lines to the one kept
            new_motions[id_list.get_index(cue.joint_id)].add_lines(cue.lines);
        }

        
    }

    cue_motions = new_motions;
}
