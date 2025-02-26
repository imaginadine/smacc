#include "animated_model.hpp"

using namespace cgp;

void animated_model_structure::skinning_lbs(std::string const& mesh_name)
{
    cgp::mesh const& mesh_bind_pose = rigged_mesh[mesh_name].mesh_bind_pose;
    cgp::mesh& mesh_deformed = rigged_mesh[mesh_name].mesh_deformed;
    controller_skinning_structure const& controller_skinning = rigged_mesh[mesh_name].controller_skinning;


    // Prepare the transformation matrix for all the joints that impact the current mesh
    int N_impacting_joints = controller_skinning.inverse_bind_matrices.size(); // only a subset of the skeleton joints may impact the current mesh
    cgp::numarray<cgp::mat4> transformation_matrix;
    transformation_matrix.resize(N_impacting_joints);
    for(int k=0; k<N_impacting_joints; ++k) {
        mat4 const& inv_bind_pose = controller_skinning.inverse_bind_matrices.at(k); // the inverse of the bind pose (precomputed)
        int joint_index_in_skeleton = controller_skinning.rig_index_to_skeleton_index.at(k); // need to find the corresponding index of this joint into the global skeleton
        mat4 const& joint_current = skeleton.joint_matrix_global.at(joint_index_in_skeleton); // retrieve the current joint pose in the global position

        transformation_matrix.at(k) = joint_current * inv_bind_pose;
    }

    // Compute skinning deformation
    int N_vertex = mesh_bind_pose.position.size();
    for(int k_vertex=0; k_vertex<N_vertex; ++k_vertex) {
        int N_dependence = controller_skinning.vertex_to_joint_dependence.at(k_vertex).size();
        auto const& vertex_to_joint_dependence = controller_skinning.vertex_to_joint_dependence.at(k_vertex);
        vec3 const& p0 = mesh_bind_pose.position.at(k_vertex);
        vec3 const& n0 = mesh_bind_pose.normal.at(k_vertex);

        vec3 p_accumulated;
        vec3 n_accumulated;
        for(int k=0; k<N_dependence; ++k) {
            auto const& dependence = vertex_to_joint_dependence.at(k);
            int joint_index = dependence.joint_index;
            float weight = dependence.weight;

            mat4 const& T = transformation_matrix.at(joint_index);

            vec3 p_tmp = T.transform_position(p0);
            vec3 n_tmp = T.transform_vector(n0);
            p_tmp *= weight;
            n_tmp *= weight;
            p_accumulated += p_tmp;
            n_accumulated += n_tmp;
        }
        mesh_deformed.position[k_vertex] = p_accumulated;
        mesh_deformed.normal[k_vertex] = n_accumulated;
    }
}


void animated_model_structure::skinning_dqs(std::string const& mesh_name)
{

    controller_skinning_structure const& controller_skinning = rigged_mesh[mesh_name].controller_skinning;

    // Prepare the dual quaternion for all the joints that impact the current mesh
    int N_impacting_joints = controller_skinning.inverse_bind_matrices.size(); // only a subset of the skeleton joints may impact the current mesh
    numarray<quaternion_dual> dq;
    dq.resize(N_impacting_joints);
    for(int k=0; k<N_impacting_joints; ++k) {
        mat4 const& inv_bind_pose = controller_skinning.inverse_bind_matrices.at(k); // the inverse of the bind pose (precomputed)
        int joint_index_in_skeleton = controller_skinning.rig_index_to_skeleton_index.at(k); // need to find the corresponding index of this joint into the global skeleton
        mat4 const& joint_current = skeleton.joint_matrix_global.at(joint_index_in_skeleton); // retrieve the current joint pose in the global position

        mat4 transformation_matrix = joint_current * inv_bind_pose;
        affine_rt a = affine_rt::from_matrix(transformation_matrix);
        dq[k] = quaternion_dual(a.rotation.data, a.translation);
    }


    // Compute Skinning Deformation
    int N_vertex = rigged_mesh[mesh_name].mesh_bind_pose.position.size();
    for(int kv=0; kv<N_vertex; ++kv) {
        int N_dependence = controller_skinning.vertex_to_joint_dependence.at(kv).size();
        auto const& vertex_to_joint_dependence = controller_skinning.vertex_to_joint_dependence.at(kv);
        
        quaternion_dual d = { {0,0,0,0}, {0,0,0,0} };
        for(int k=0; k<N_dependence; ++k){
            auto const& dependence = vertex_to_joint_dependence.at(k);
            int joint_index = dependence.joint_index;
            float weight = dependence.weight;

            d += weight * dq[joint_index];
        }

        float const q_n = norm(d.q);
        if( std::abs(q_n)>1e-5f)
			d = d/q_n;

        rigged_mesh[mesh_name].mesh_deformed.position[kv] = rotation_transform(d.q)*rigged_mesh[mesh_name].mesh_bind_pose.position[kv] + d.translation();
        rigged_mesh[mesh_name].mesh_deformed.normal[kv] = rotation_transform(d.q)*rigged_mesh[mesh_name].mesh_bind_pose.normal[kv];
   }
}


void animated_model_structure::set_skeleton_from_animation(std::string const& animation_name, float t)
{
    auto const& current_animation = animation[animation_name];
    int N_joint = current_animation.joint_index.size();
    for(int k=0; k<N_joint; ++k) {
        int joint = current_animation.joint_index[k];
        mat4 M = current_animation.evaluate(k, t);
        skeleton.joint_matrix_local[joint] = M;
    }
    skeleton.update_joint_matrix_local_to_global();
}


void animated_model_structure::set_skeleton_from_ending_joints(Motion m, float t)
{

    for(int k=0; k<m.chain.size()-1; k++) {
        int joint = m.chain[k];
        mat4 M = m.evaluate_end(k, t);
        skeleton.joint_matrix_local[joint] = M;
    }

    skeleton.update_joint_matrix_local_to_global();
}


void animated_model_structure::set_skeleton_from_motion_all(Motion& m, float t)
{
    //the joint that moves
    mat4 M = m.evaluate(t);
    skeleton.joint_matrix_global[0] = M;
    // to local
    skeleton.joint_matrix_local[0] = skeleton.joint_matrix_global[0];
    skeleton.update_joint_matrix_local_to_global();

    if(m.impacts.size() > 0 && t >= m.times[m.N_pos_before]) { 
        // manage impacts
        set_skeleton_from_motion_impacts(m);
    }
    
    skeleton.update_joint_matrix_local_to_global();

}


bool animated_model_structure::is_reachable_from_motion_impacts(Motion& m, int impact_joint_id, vec3 pos_impact)
{

    bool reachable;

    if (impact_joint_id == m.joint_id) {
        reachable = false;
    } else {
        // do not start at the root of the character
        int m_joint_id = m.joint_id;
        if(m_joint_id == 0) {
            while( ! is_joint_parent(impact_joint_id, m_joint_id, skeleton) ) {
                m_joint_id++;
            }
        }

        vec3 current_pos_m = skeleton.joint_matrix_global[m_joint_id].get_block_translation();

        // 1) move impact_joint to the position of the impact
        vec3 impact_joint_pos = skeleton.joint_matrix_global[impact_joint_id].get_block_translation();
        vec3 translation = pos_impact - impact_joint_pos;

        // 2) move the body from joint_id (included) to all its children
        skeleton.apply_translation_recursive(m_joint_id, translation);

        // 3) do the inverse of inverse kinematics lol
        int start_id = impact_joint_id; // id blocked
        int end_id = m_joint_id; // to be moved if necessary
        vec3 current_pos = skeleton.joint_matrix_global[m_joint_id].get_block_translation(); // current position of joint_id
        vec3 offset = current_pos_m - current_pos; // where it should be - where it is

        ik_structure effect_ik = ik_start(end_id, current_pos, offset, start_id);
        effect_ik.normal_direction = false;
        reachable = is_reachable(effect_ik, skeleton);
    }

    return reachable;

}

vec3 animated_model_structure::set_skeleton_from_motion_impacts(Motion& m)
{
    // for each impact
    for (const auto& impact_pair : m.impacts){

        int impact_joint_id = impact_pair.first;
        vec3 pos_impact = impact_pair.second;

        // do not start at the root of the character
        int m_joint_id = m.joint_id;
        while(m_joint_id == 0 || ! is_joint_parent(impact_joint_id, m_joint_id, skeleton) ) { // add the condition to do with multiple impacts
            m_joint_id++;
        }

        vec3 current_pos_m = skeleton.joint_matrix_global[m_joint_id].get_block_translation();

        // 1) move impact_joint to the position of the impact
        vec3 impact_joint_pos = skeleton.joint_matrix_global[impact_joint_id].get_block_translation();
        vec3 translation = pos_impact - impact_joint_pos;

        // 2) move the body from joint_id (included) to all its children
        skeleton.apply_translation_recursive(m_joint_id, translation);

        // 3) do the inverse of inverse kinematics lol
        int start_id = impact_joint_id; // id blocked
        int end_id = m_joint_id; // to be moved if necessary
        vec3 current_pos = skeleton.joint_matrix_global[m_joint_id].get_block_translation(); // current position of joint_id
        vec3 offset = current_pos_m - current_pos; // where it should be - where it is

        ik_structure effect_ik = ik_start(end_id, current_pos, offset, start_id);
        effect_ik.normal_direction = false;

        // 4) compute ik and update skeleton
        ik_compute(effect_ik, skeleton, m.is_constrained);
        skeleton.update_joint_matrix_local_to_global();

        // 5) return the position to follow
        return current_pos_m;

    }

}

void animated_model_structure::set_skeleton_from_motion_joint_ik(Motion& m, float t)
{

    int k = m.joint_id;
        
    // the joint that moves
    mat4 M = m.evaluate(t); // our offset
        
    vec3 current_position = skeleton.joint_matrix_global[k].get_block_translation();
    vec3 offset = M.get_block_translation() - current_position;

    // compute ik_structure with ik_start
    ik_structure effect_ik = ik_start(k, current_position, offset, m.joint_root_ik);
    // compute the inverse kinematics
    ik_compute(effect_ik, skeleton, m.is_constrained);


    // do not add "bouncing" on the same joint
    bool same_id = false;
    for (const auto& impact_pair : m.impacts){        
        int impact_joint_id = impact_pair.first;
    
        if (m.joint_id == impact_joint_id) {
            same_id = true;
            break;
        }
    }

    if(m.impacts.size() > 0 && !same_id && t >= m.times[m.N_pos_before]) { 

        // manage impacts
        skeleton.update_joint_matrix_local_to_global();
        set_skeleton_from_motion_impacts(m);
    }

    skeleton.update_joint_matrix_local_to_global();
   
}



void animated_model_structure::set_skeleton_from_motion_joint(Motion m, float t)
{
    int N_joint = skeleton.size();
    int k = m.joint_id;

    //the joint that moves
    mat4 M = m.evaluate(t);
    skeleton.joint_matrix_global[k] = M ;
    
    // after the joint
    for (int i = k+1; i < N_joint; i++)
        skeleton.joint_matrix_global[i] = skeleton.joint_matrix_global[skeleton.parent_index[i]] * skeleton.joint_matrix_local[i];

    // before the joint
    m.update_skeleton(skeleton);

}


void animated_model_structure::apply_transformation(vec3 const& translation, rotation_transform rotation, float scaling)
{
    if(skeleton.joint_matrix_local.size()>0) {
        skeleton.joint_matrix_local[0] = rotation * skeleton.joint_matrix_local[0];
        skeleton.joint_matrix_local[0].apply_scaling_to_block_linear(scaling);
        skeleton.joint_matrix_local[0].apply_translation(translation);
        skeleton.update_joint_matrix_local_to_global();
    }

    for(auto& anim_entry : animation) {
        auto& animation_matrices = anim_entry.second.matrix;
        for(int kt=0; kt<animation_matrices[0].size(); ++kt) {
            mat4& M = animation_matrices[0][kt];
            M = rotation*M;
            M.apply_scaling_to_block_linear(scaling);
            M.apply_translation(translation);
        }
    }

}

/**
Inputs : 
    p : the position of the joint that we want to change the orientation. It has just moved.
    p_base = the initial position of p
    p_orientation_base = the initial orientation of p, to be changed
    p2 = the position of the joint after p
    p2_base = the initial position of p2
    p2_or = the orientation of p2
    angle_max : the maximum angle that the rotation between two joints can make

Output : the new orientation of the frame of the joint
 */ 
rotation_transform constrain_orientation(vec3 p, vec3 p_base, rotation_transform p_orientation_base, vec3 p2, vec3 p2_base, rotation_transform p2_or, float angle_max)
{
    rotation_transform final_orientation;

    // find the new orientation of the joint (without constraints)
    rotation_transform rt = rotation_transform::from_vector_transform(normalize(p2_base - p_base), normalize(p2 - p));
    mat3 RT = rt.matrix();
    mat3 RA = p_orientation_base.matrix();
    mat3 RB = RT * RA;
    rotation_transform temp_orientation = rotation_transform::from_matrix(RB);

    // if there are no constraints
    if (angle_max >= 2.0f*Pi) {
        final_orientation = temp_orientation;
    } else {
        // in the case of constraints, look if the orientation is within the limits
        frame f = frame(temp_orientation, p);
        frame f2 = frame(p2_or, p2);
        rotation_transform turning = rotation_transform::from_frame_transform(f2.ux(), f2.uy(), f.ux(), f.uy());
        vec3 axis; float angle;
        while(angle < 0.0f) {
            angle += 2.0f*Pi;
        }
        turning.to_axis_angle(axis, angle);
        if (angle > angle_max) {
            angle = angle_max;
            turning = rotation_transform::from_axis_angle(axis, angle);
            final_orientation = f2.orientation * inverse(turning);
        } else {
            final_orientation = temp_orientation;
        }
        
    }

    return final_orientation;
}


// Input : the target position target (to modify), the two positions before p_1 and p_2, the angle constraints tetas
// Output : the new position of target
vec3 constrain_rotation(vec3 t, vec3 p_1, vec3 p_2, float teta1, float teta2, float teta3, float teta4)
{
    vec3 new_target = t;
    // find the line equation L1
    vec3 l1 = normalize(p_1 - p_2);
    // find the projection o of the target on line l1
    vec3 o = p_1 + dot((t - p_1), l1) * l1;
    // find the distance between the point o and the joint position
    float dist = norm(p_1 - o);
    // map the target (rotate and translate) in such a way that o is now located at the axis origin and oriented according to the x and y axis
    vec3 translation = vec3(0.f, 0.f, 0.f) - o;
    rotation_transform rt = rotation_transform::from_vector_transform(l1, vec3(0.f,0.f,1.f)); 
    t += translation;
    t = rt * t;
    // find in which quadrant the target belongs and find the conic section (ellipse parameters)
    float a, b;
    if(t.x > 0) {
        if(t.y > 0) {
            a = dist * tan(teta3);
            b = dist * tan(teta2);
        } else {
            a = dist * tan(teta3);
            b = dist * tan(teta4);
        }
    } else {
        if(t.y > 0) {
            a = dist * tan(teta1);
            b = dist * tan(teta2);
        } else {
            a = dist * tan(teta1);
            b = dist * tan(teta4);
        }
    }

    // t outside of the ellipse?
    if ( ( (t.x*t.x)/(a*a) + (t.y*t.y)/(b*b) ) > 1) {
        // find the nearest point on that conic section from the target
        vec2 t_res = find_nearest_point_ellipse(a, b, vec2(t.x, t.y));
        new_target = vec3(t_res.x, t_res.y, 0.f);
    
        // inverse transformation
        new_target = inverse(rt) * new_target; 
        new_target -= translation;
    }

    return new_target;
}

bool is_reachable(ik_structure const effect_ik, skeleton_structure skeleton)
{

    int start_joint_index = effect_ik.joint_root_ik;
    int current_joint_index = effect_ik.joint_target;
    if (effect_ik.joint_root_ik > effect_ik.joint_target) {
        current_joint_index = effect_ik.joint_root_ik;
        start_joint_index = effect_ik.joint_target;
    }

    numarray<float> chain_length;
    while(current_joint_index!=start_joint_index && current_joint_index!=0) {
        vec3 p1 = skeleton.joint_matrix_global[current_joint_index].get_block_translation();
        vec3 p2 = skeleton.joint_matrix_global[skeleton.parent_index(current_joint_index)].get_block_translation();
        chain_length.push_back(norm(p1-p2));
        current_joint_index = skeleton.parent_index(current_joint_index);
    }

    vec3 p_constrained = effect_ik.target_position + effect_ik.target_offset;
    vec3 p0 = skeleton.joint_matrix_global[effect_ik.joint_root_ik].get_block_translation();
    float dist = norm(p0 - p_constrained);
	float total_dist = sum(chain_length);

    return dist <= total_dist ;
}

// Update the current position of the IK constraint to match at its start the current joint position
ik_structure ik_start(int joint_end, vec3 current_pos, vec3 target_offset, int root_ik)
{
    ik_structure effect_ik;

    effect_ik.joint_target = joint_end;

	// set this position as the default objective for the IK
	effect_ik.target_position = current_pos;

	effect_ik.target_offset = target_offset;
    effect_ik.joint_root_ik = root_ik;

    return effect_ik;
}

void ik_compute(ik_structure const& effect_ik, skeleton_structure& skeleton, bool with_constraints)
{

    int end_joint_index = effect_ik.joint_target;
	int start_joint_index = effect_ik.joint_root_ik;

	// Build IK chain of joint between end and start
	vec3 const start_joint_position = skeleton.joint_matrix_global[start_joint_index].get_block_translation();

    // Compute the chain of positions, from the end effector to the root
    numarray<int> chain_index;
    
    if (effect_ik.normal_direction) {
        int current_joint_index = end_joint_index;
        while(current_joint_index!=start_joint_index && current_joint_index!=0) {
            chain_index.push_back(current_joint_index);
            current_joint_index = skeleton.parent_index(current_joint_index);
        }
        chain_index.push_back(start_joint_index);

        if(current_joint_index==0 && start_joint_index!=0) {
            warning_cgp("Joint Index","Start joint index "+str(start_joint_index)+" is not a parent of End joint index "+str(end_joint_index));
        }

        // Inverse the chain : from the root to the end effector
        numarray<int> c = chain_index;
        for(int k=0; k<chain_index.size(); ++k) {
            chain_index[k] = c[chain_index.size()-k-1];
        }

    } else {
        int current_joint_index = start_joint_index;
        while(current_joint_index!=end_joint_index && current_joint_index!=0) {
            chain_index.push_back(current_joint_index);
            current_joint_index = skeleton.parent_index(current_joint_index);
        }
        chain_index.push_back(end_joint_index);

    }


	// Store positions, length and rotation transforms of the IK chain
	int N = chain_index.size();
	numarray<vec3> chain_position;
    numarray<rotation_transform> chain_rt;
	for(int k=0; k<N; ++k) {
		chain_position.push_back(skeleton.joint_matrix_global[chain_index[k]].get_block_translation());
        chain_rt.push_back(rotation_transform::from_matrix(skeleton.joint_matrix_global[chain_index[k]].get_block_linear()));
	}
	numarray<float> chain_length;
	for(int k=0; k<N-1; ++k) {
		vec3 const& p1 = skeleton.joint_matrix_global[chain_index[k]].get_block_translation();
		vec3 const& p2 = skeleton.joint_matrix_global[chain_index[k+1]].get_block_translation();
		chain_length.push_back(norm(p1-p2));
	}
    numarray<vec3> chain_old_position;
    chain_old_position.resize(N);
    chain_old_position = chain_position;

    vec3 p_constrained = effect_ik.target_position + effect_ik.target_offset;
   
    // IK algorithm
    int N_IK_iteration = 6;
    float tol = 0.01f;

    int iter = 0;
    float offset = norm(chain_position[N-1] - p_constrained);
    while( (offset > tol) && (iter < N_IK_iteration)) {

        // Backward pass
        chain_position[N-1] = p_constrained;
        for(int k=0; k<N-1; ++k) {
            // position
            vec3 p1 = chain_position[N-k-1];
            vec3& p2 = chain_position[N-k-2];
            vec3 p12 = p2-p1;
        
            float L12 = norm(p12);
            float L120 = chain_length[N-k-2]; 
            vec3 u12 =p12/L12;
            p2 = p1+L120*u12;

            // orientation
            vec3 p10 = chain_old_position[N-k-1];
            vec3 p20 = chain_old_position[N-k-2];
            if (with_constraints) {
                chain_rt[N-k-2] = constrain_orientation(p2, p20, chain_rt[N-k-2], p1, p10, chain_rt[N-k-1], skeleton.constraint_orientation[chain_index[N-k-2]]);
            } else {
                if(effect_ik.normal_direction) {
                    chain_rt[N-k-2] = constrain_orientation(p2, p20, chain_rt[N-k-2], p1, p10, chain_rt[N-k-1], 2.f*Pi);
                } else {
                    chain_rt[N-k-1] = constrain_orientation(p1, p10, chain_rt[N-k-1], p2, p20, chain_rt[N-k-2], 2.f*Pi);
                }
            }    
    
           if (with_constraints) {
                // rotation
                if(k<N-2){
                    vec3& p3 = chain_position[N-k-3];
                    vec4 angles = skeleton.constraint_angles[chain_index[N-k-2]];
                    if(angles.x != -1.f) { // if a constraint on the angles is defined for this joint
                        p3 = constrain_rotation(p3, p2, p1, angles.x, angles.y, angles.z, angles.w);
                    }
                }
            }

        }
        chain_old_position = chain_position;

        // Forward pass
        chain_position[0] = start_joint_position;
        for(int k=0; k<N-1; ++k) {
            // position
            vec3 p1 = chain_position[k];
            vec3& p2 = chain_position[k+1];
            vec3 p12 = p2-p1;
            float L12 = norm(p12);
            float L120 = chain_length[k]; 
            vec3 u12 =p12/L12;
            p2 = p1+L120*u12;

            // orientation
            vec3 p10 = chain_old_position[k];
            vec3 p20 = chain_old_position[k+1];
            if (with_constraints) {
                chain_rt[k] = constrain_orientation(p1, p10, chain_rt[k], p2, p20, chain_rt[k+1], skeleton.constraint_orientation[chain_index[k]]);
            } else{
                if(effect_ik.normal_direction) {
                    chain_rt[k] = constrain_orientation(p1, p10, chain_rt[k], p2, p20, chain_rt[k+1], 2.f*Pi);
                } else {
                    chain_rt[k+1] = constrain_orientation(p2, p20, chain_rt[k+1], p1, p10, chain_rt[k], 2.f*Pi);
                }
            }

            if (with_constraints) {

                // rotation
                if(k<N-2){
                    vec3& p3 = chain_position[k+2];
                    vec4 angles = skeleton.constraint_angles[chain_index[k+1]];
                    if(angles.x != -1.f) { // if a constraint on the angles is defined for this joint
                        p3 = constrain_rotation(p3, p2, p1, angles.x, angles.y, angles.z, angles.w);
                    }
                }
            }    
        }
        chain_old_position = chain_position;

        iter++;
        offset = norm(chain_position[N-1] - p_constrained);
	}
  
	// Build the rotation to match the computed position of the IK along the chain
    if(effect_ik.normal_direction) {
        for(int k=0; k<N-1; ++k) {
        
            mat3 RA = skeleton.joint_matrix_global[chain_index[k]].get_block_linear();
            mat3 RB = chain_rt[k].matrix();
            mat3 RT = RB * transpose(RA);

            skeleton.joint_matrix_global[chain_index[k]].apply_transform_to_block_linear(RT);
        }
    } else {
        for(int k=1; k<N; ++k) {
        
            mat3 RA = skeleton.joint_matrix_global[chain_index[k]].get_block_linear();
            mat3 RB = chain_rt[k].matrix();
            mat3 RT = RB * transpose(RA);

            skeleton.joint_matrix_global[chain_index[k]].apply_transform_to_block_linear(RT);
        }
        // do it also for the 
    }
	
	// Set the position along the chain
    for(int k=0; k<N; ++k) {
        skeleton.joint_matrix_global[chain_index[k]].set_block_translation(chain_position[k]);
    }

    

	// Build the corresponding local matrix (for coherent hierarchy even after the end of the IK)
    if(effect_ik.normal_direction) {
        for(int k=0; k<N-1; ++k) {

            int idx = chain_index[k];
            
            mat4 parent_matrix_global = mat4().set_identity();
            if(idx>0) {
                parent_matrix_global = skeleton.joint_matrix_global[skeleton.parent_index[idx]];
            }
            mat4 Lk = parent_matrix_global.inverse_assuming_rigid_transform() * skeleton.joint_matrix_global[idx];
            skeleton.joint_matrix_local[idx] = Lk;
        }
    } else {
        for(int k=N-1; k>0; --k) {

            int idx = chain_index[k];
            
            mat4 parent_matrix_global = mat4().set_identity();
            if(idx>0) {
                parent_matrix_global = skeleton.joint_matrix_global[skeleton.parent_index[idx]];
            }
            mat4 Lk = parent_matrix_global.inverse_assuming_rigid_transform() * skeleton.joint_matrix_global[idx];
            skeleton.joint_matrix_local[idx] = Lk;
        }
    }


}

numarray<float> get_distances_to_children(skeleton_structure skeleton, int id_parent)
{
    numarray<float> children_dist_to_keep;

    mat4 global_parent_joint = skeleton.joint_matrix_global[id_parent];

    for (int j=0; j<skeleton.child(id_parent).size();j++) {
        int id_child = skeleton.child(id_parent)[j];
        mat4 global_child_joint  = skeleton.joint_matrix_global[id_child];
        children_dist_to_keep.push_back( norm( global_parent_joint.get_block_translation() - global_child_joint.get_block_translation() ) );
    }
    return children_dist_to_keep;
}

void give_follow_through(skeleton_structure& skeleton, int id_parent, numarray<float> children_dist_to_keep, vec3 old_pos_parent)
{
    // find children of parent
    cgp::numarray<int> children = skeleton.child(id_parent);
    // if it has children, for each child
    for (int i=0; i<children.size();i++) {
        int id_child = children[i];
        float dist_to_keep = children_dist_to_keep[i];

        // get the global distance to keep from the local joints
        mat4& global_child_joint = skeleton.joint_matrix_global[id_child];
        mat4& global_parent_joint = skeleton.joint_matrix_global[id_parent];
        mat4& local_parent_joint = skeleton.joint_matrix_local[id_parent];

        vec3 old_pos_child = global_child_joint.get_block_translation();

        // before moving the joints, save the distances of the children of the child
        numarray<float> after_children_dist_to_keep = get_distances_to_children(skeleton, id_child);

        // calculate the actual distance of the joints
        float actual_dist = norm( global_parent_joint.get_block_translation() - global_child_joint.get_block_translation() );

        if (actual_dist != dist_to_keep) {
            // replace the child joint to keep this distance
            global_child_joint.set_block_translation( global_parent_joint.get_block_translation() + dist_to_keep * normalize( global_child_joint.get_block_translation() - global_parent_joint.get_block_translation() ) );
            // update linear part
            if (i == (children.size()/2)) {
                rotation_transform rt = rotation_transform::from_vector_transform(normalize(old_pos_child - old_pos_parent), normalize(global_child_joint.get_block_translation() - global_parent_joint.get_block_translation()));
                global_parent_joint.apply_transform_to_block_linear(rt.matrix());
                // update local ?
		        local_parent_joint = skeleton.joint_matrix_global[skeleton.parent_index[id_parent]].inverse_assuming_rigid_transform() * global_parent_joint;
                //std::cout<<"rotation applied to "<<id_parent<<std::endl;
            }
        }

        // continue recursively to the children of the child
        give_follow_through(skeleton, id_child, after_children_dist_to_keep, old_pos_child);
    }
}

void animated_model_structure::set_skeleton_from_t0(int id_parent)
{
    cgp::numarray<int> children = skeleton.child(id_parent);
    // if it has children, for each child
    for (int i=0; i<children.size();i++) {

        int id_child = children[i];

        skeleton.joint_matrix_global[id_child] = skeleton_t0.joint_matrix_global[id_child];
        skeleton.joint_matrix_local[id_child] = skeleton_t0.joint_matrix_local[id_child];

        set_skeleton_from_t0(id_child);
    }
    
}
