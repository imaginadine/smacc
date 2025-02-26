#pragma once

#include "cgp/cgp.hpp"

#include "../controller_skinning/controller_skinning.hpp"
#include "../skeleton_structure/skeleton_structure.hpp"
#include "../skeleton_animation/skeleton_animation.hpp"
#include "../motion_lines/motion/motion.hpp"
#include "../controller_skinning/dual_quaternion.hpp"
#include "../../helpers/ellipse_fitter.hpp"

struct rigged_mesh_structure {
    cgp::mesh mesh_bind_pose;  // Bind pose (/un-deformed) mesh
    cgp::mesh mesh_deformed;   // Deformed mesh
    controller_skinning_structure controller_skinning;   // skinning weights dependence
};

struct animated_model_structure {

    skeleton_structure skeleton; // A skeleton that stores its structure/hierarchy, and the local and global joint frames
    std::map<std::string, rigged_mesh_structure> rigged_mesh; // an animated model may be linked to multiple rigged meshes
    std::map<std::string, skeleton_animation_structure> animation; // storage for all the possible animation of this skeleton

    skeleton_structure skeleton_t0;

    // Compute the joint position corresponding to the given animation_name at time t from the animation structure. Then update the skeleton structure in filling the local joint matrix. Finally update the global matrices of the skeleton.
    //  - t can be an arbitrary float values, the animation matrix are interpolated between the frames
    //  - This function doesn't call the Skinning deformation on the rigged_mesh
    void set_skeleton_from_animation(std::string const& animation_name, float t);

    // 1st method : all the object is moving
    void set_skeleton_from_motion_all(Motion& m, float t);
    // 2nd method : a joint is moving
    void set_skeleton_from_motion_joint(Motion m, float t);
    void set_skeleton_from_motion_joint_ik(Motion& m, float t);
    cgp::vec3 set_skeleton_from_motion_impacts(Motion& m);
    bool is_reachable_from_motion_impacts(Motion& m, int impact_joint_id, cgp::vec3 pos_impact);
    void set_skeleton_from_ending_joints(Motion m, float t);

    // Compute the Linear Blend Skinning deformation on the designated rigged mesh
    void skinning_lbs(std::string const& mesh_name);
    // Compute the Dual Quaternion Skinning deformation on the designated rigged mesh
    void skinning_dqs(std::string const& mesh_name);

    // Apply a tranlation, rotation, and scaling to all the skeleton structure (current skeleton and all animation)
    void apply_transformation(cgp::vec3 const& translation, cgp::rotation_transform rotation= cgp::rotation_transform(), float scaling=1.0f);

    void set_skeleton_from_t0(int id_parent);
};

// Structure storing state to handle the Inverse Kinematics data
//  The joint at index joint_target, should target the position given as target_position+target_offset
//  The target_offset is controled via the GUI, as an offset relative to the initial position.
struct ik_structure {
	int joint_target;   // Index of the target joint IK
	int joint_root_ik = 7;  // Index of the start of the kinematics chain
	cgp::vec3 target_position; // Base target position
	cgp::vec3 target_offset;   // Offset of the target position (to be added to target_position)
    bool normal_direction = true;
};

// Compute an Inverse Kinematic 
void ik_compute(ik_structure const& effect_ik, skeleton_structure& skeleton, bool with_constraints);
// Update the current position of the IK constraint to match at its start the current joint position
ik_structure ik_start(int joint_end, cgp::vec3 current_pos, cgp::vec3 target_offset, int root_ik);
bool is_reachable(ik_structure const effect_ik, skeleton_structure skeleton);
cgp::numarray<float> get_distances_to_children(skeleton_structure skeleton, int id_parent);
void give_follow_through(skeleton_structure& skeleton, int id_parent, cgp::numarray<float> children_dist_to_keep, cgp::vec3 old_pos_parent);