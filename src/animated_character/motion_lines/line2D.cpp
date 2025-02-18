#include "line2D.hpp"

using namespace cgp;

/**
Orientation :
Positive if v is to the left of u.
Negative if v is to the right of u.
Zero if they are collinear.
 */
float cross2D (vec2 u, vec2 v) {
    return ((u.x * v.y) - (u.y * v.x));
}

bool in_list (int nb, numarray<int> list) {
    bool res = false;
    for (int nb_to_verify : list)  {
        if (nb == nb_to_verify) {
            res = true;
            break;
        }  
    }
    return res;
}

bool has_acceptable_joints(int vertex_id, controller_skinning_structure controller_skinning, numarray<int> black_list)
{
    bool res = true;
    int N_dependence = controller_skinning.vertex_to_joint_dependence.at(vertex_id).size();
    for (int k_joint =0; k_joint < N_dependence ; k_joint++) {
        auto const dependence = controller_skinning.vertex_to_joint_dependence[vertex_id][k_joint];
        int joint_index = dependence.joint_index;

        if( in_list(joint_index, black_list)){
            res = false;
            break;
        }
    }
    return res;
}


int most_dependant_joint(int vertex_id, controller_skinning_structure controller_skinning)
{
    int N_dependence = controller_skinning.vertex_to_joint_dependence.at(vertex_id).size();

    auto const dependence = controller_skinning.vertex_to_joint_dependence[vertex_id][0];
    int joint_winning_index = dependence.joint_index;
    float max_weight = dependence.weight;

    for (int k_joint =1; k_joint < N_dependence ; k_joint++) {
        auto const dependence = controller_skinning.vertex_to_joint_dependence[vertex_id][k_joint];
        int joint_index = dependence.joint_index;
        float weight = dependence.weight;

        if(weight > max_weight){
            joint_winning_index = joint_index;
            max_weight = weight;
        }
    }

    return joint_winning_index;
}


int find_joint_from_2D_line(numarray<vec2> projected_positions, animated_model_structure model, camera_projection_perspective const& P, mat4 const& camera_view, float& depth_to_find)
{
    int N_line_pos = projected_positions.size();
    vec2 last_pos = projected_positions[N_line_pos - 1];

    numarray<mat4> global_joints = model.skeleton.joint_matrix_global;
    int N_joints = global_joints.size();

    // project joint positions
    numarray<vec3> projected_j_pos;
    for (int k=0; k<N_joints; k++) {
        vec3 joint_pos = global_joints[k].get_block_translation();
        vec4 position_projected = P.matrix() * camera_view * vec4(joint_pos,1.0f);
        projected_j_pos.push_back(position_projected.xyz()/position_projected.w);
    }


    float min_dist = 100.f;
    int final_joint = -1;
    for(int k_joint = 0; k_joint<N_joints;k_joint++) {
        vec3 j_pos = projected_j_pos[k_joint];
        vec3 vec_j_3D = j_pos - vec3(last_pos, 0.f);
        float dist = norm(vec_j_3D);
        if(dist < min_dist) {
            min_dist = dist;
            final_joint = k_joint;
        }
    }

    // find depth
    if(final_joint != -1) {
        depth_to_find = projected_j_pos[final_joint].z;
    }

    return final_joint;
}


bool is_wrapping_object(cgp::numarray<cgp::vec2> projected_positions, animated_model_structure animated_model, camera_projection_perspective const& P, mat4 const& camera_view)
{
    numarray<mat4> global_joints = animated_model.skeleton.joint_matrix_global;
    int N_joints = global_joints.size();

    // project joint positions
    numarray<vec2> projected_j_pos;
    for (int k=0; k<N_joints; k++) {
        vec3 joint_pos = global_joints[k].get_block_translation();
        vec4 position_projected = P.matrix() * camera_view * vec4(joint_pos,1.0f);
        projected_j_pos.push_back(position_projected.xy()/position_projected.w);
    }

    bool res = true;

    // for each joint
    for (int i=0; i<N_joints;i++) {
        vec2 j_pos = projected_j_pos[i];
        // if it is outside the line:
        if(! is_inside_polygon(j_pos, projected_positions)){
            res = false;
            break;
        }
    }

    return res;
}


int get_closest_line_id_2D(vec2 pos, numarray<line_structure> lines, camera_projection_perspective const& P, mat4 const& camera_view)
{
    int N_lines = lines.size();

    // project the middle of each line to the current view plan
    numarray<vec2> middle_projections;
    for (int i = 0; i< N_lines;i++) {
        vec4 position_projected = P.matrix() * camera_view * vec4(lines[i].samples[lines[i].samples.size()/2],1.0f);
        middle_projections.push_back(position_projected.xy()/position_projected.w);
    }

    // take the line that is the closest to mouse position
    float min_dist = norm(pos - middle_projections[0]);
    int closest_line_id = 0;
    // find the closest line
    for (int k_line = 1; k_line < N_lines ; k_line ++) {
        float dist_l = norm(pos - middle_projections[k_line]);
        if (dist_l < min_dist) {
            min_dist = dist_l;
            closest_line_id = k_line;
        }
    }

    return closest_line_id;
}