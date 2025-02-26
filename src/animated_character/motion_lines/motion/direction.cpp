#include "direction.hpp"

using namespace cgp;


void Direction::find_positions(skeleton_structure skeleton, vec3 t_source)
{
    line_structure dir_line = get_median_line(skeleton);

    calculate_speed();
    std::cout<<"v = "<<v<<std::endl;

    N_pos_before = dir_line.samples.size()-1;
    
    int N_pos_total = N_pos_before+1;
    positions_to_follow.resize(N_pos_total);

    // position of movement
    positions_to_follow[N_pos_before] = t_source;

    vec3 correction = t_source - dir_line.samples[dir_line.samples.size()-1];

    // before the movement
    for(int i=0; i<N_pos_before;i++){
        positions_to_follow[i] = dir_line.samples[i] + correction;
    }

    // find distances between the positions
    find_distances();
}


void Direction::find_positions_global(skeleton_structure skeleton, vec3 t_source)
{
    line_structure dir_line = get_median_line(skeleton);

    calculate_speed();

    N_pos_before = dir_line.samples.size();
    
    int N_pos_total = N_pos_before * 2 + 1; // N_pos_total > (N_pos_before + 1)
    positions_to_follow.resize(N_pos_total);

    // position of movement
    positions_to_follow[N_pos_before] = t_source;

    vec3 correction = t_source - dir_line.samples[dir_line.samples.size()-1];

    // before the movement
    for(int i=0; i<N_pos_before;i++){
        positions_to_follow[i] = dir_line.samples[i] + correction;
    }

    // case of close start and end
    if (norm(dir_line.samples[dir_line.samples.size()-1] - dir_line.samples[0]) < 0.1f && dir_line.get_length() > 0.3f) {
        std::cout<<"boucle"<<std::endl;
        
        // same position
        for (int i=0 ; i < N_pos_total; i++) {
            positions_to_follow[i] = t_source;
        }

        // the axis equals (0;0;0), compute it with half of the points
        line_structure half_line = dir_line;
        half_line.samples.resize(dir_line.samples.size()/2);
        float magnitude;
        vec3 axis;
        half_line.angular_velocity(axis, magnitude);

        // the rotation is around the circle axis
        rotation_transform rt_spinning = rotation_transform::from_axis_angle(axis, magnitude * 0.03f);

        joints.resize(N_pos_total);
        // pose
        joints[N_pos_before] = skeleton.joint_matrix_global[joint_id];
        // before
        for(int i=N_pos_before-1; i >= 0; i--) {
            mat4 centered_joint = joints[i+1];
            centered_joint.apply_translation(-t_source); // /!\ need to turn around the root joint
            joints[i] = inverse(rt_spinning) * centered_joint;
            joints[i].apply_translation(t_source);
        }
        // after
        for(int i=N_pos_before+1; i < N_pos_total; i++) {
            mat4 centered_joint = joints[i-1];
            centered_joint.apply_translation(-t_source); // /!\ need to turn around the root joint
            joints[i] = rt_spinning * centered_joint;
            joints[i].apply_translation(t_source);
        }

    } else {
        std::cout<<"ligne"<<std::endl;
        vec3 dir = dir_line.samples[dir_line.samples.size()-1] - dir_line.samples[0];
        vel = v * dir;

        // after the movement
        for(int i = N_pos_before+1 ; i < N_pos_total; i++){
            positions_to_follow[i] = positions_to_follow[i-1] + 0.06f*dir;
        }
    }

    // find distances between the positions
    find_distances();
}



numarray<mat4> Direction::get_joints_in_chain(numarray<mat4> skeleton_joints)
{
    numarray<mat4> joints_in_chain;
    joints_in_chain.resize(chain.size());
    for (int i = 0; i < chain.size(); i++) {
        joints_in_chain[i] = skeleton_joints[chain[i]];
    }
    return joints_in_chain;
}


numarray<numarray<vec3>> Direction::compute_angle_velocities(animated_model_structure& animated_model, numarray<mat4>& root_global_joints)
{
    root_global_joints.resize_clear(0);

    numarray<numarray<vec3>> all_angle_vel;
    all_angle_vel.resize(times.size()-1);

    // initialize local_joints_before at t=0
    animated_model.set_skeleton_from_animation("Idle", 0.0f);
    animated_model.set_skeleton_from_motion_joint_ik(*this, 0.0f);
    numarray<mat4> local_joints_before = get_joints_in_chain(animated_model.skeleton.joint_matrix_local);

    int N_time = times.size();
    for (int k_time=1 ; k_time< N_time; k_time++) {

        // dt from the end
        float dt = times[N_time - k_time] - times[N_time - k_time - 1];

        // put the skeleton in its current pose
        animated_model.set_skeleton_from_animation("Idle", 0.0f);
        animated_model.set_skeleton_from_motion_joint_ik(*this, times[k_time]);

        root_global_joints.push_back(animated_model.skeleton.joint_matrix_global[animated_model.skeleton.parent_index[joint_root_ik]]);

        // save the joints at this current position
        numarray<mat4> local_joints_now = get_joints_in_chain(animated_model.skeleton.joint_matrix_local);

        all_angle_vel[k_time-1].resize(chain.size());
        // for each joint in the ik_chain of the motion
        for (int k_joint = 0; k_joint < chain.size(); k_joint++) {
            // extract angular velocity from IK motion

            // extract the local orientation at time step t - dt
            mat3 orientation_before = local_joints_before[k_joint].get_block_linear();
            rotation_transform rt_before = rotation_transform::from_matrix(orientation_before);
            // extract the local orientation at time step t
            mat3 orientation_now = local_joints_now[k_joint].get_block_linear();
            rotation_transform rt_now = rotation_transform::from_matrix(orientation_now);
            // compute the relative rotation between two consecutive frames
            rotation_transform relative_rt = inverse(rt_before) * rt_now;
            // compute the angular velocity
            vec3 axis;
            float angle;
            relative_rt.to_axis_angle(axis, angle);
            
            vec3 ang_vel = (2.f*axis*angle/dt) * 0.3f;
            if (std::abs(angle) < 1e-6f) {
                ang_vel = vec3(0.0f, 0.0f, 0.0f);
            }

            all_angle_vel[k_time-1][k_joint] = ang_vel;
        }

        // update before joints
        local_joints_before = local_joints_now;
    }

    return all_angle_vel;
}


void Direction::find_after_joints(animated_model_structure& animated_model)
{
    all_local_joints_after.resize_clear(0);

    float t = times[N_pos_before];

    numarray<mat4> root_global_joints;
    numarray<numarray<vec3>> all_angle_vel = compute_angle_velocities(animated_model, root_global_joints);

    // Put the skeleton at the end of the motion line
    animated_model.set_skeleton_from_animation("Idle", 0.0f);
	animated_model.set_skeleton_from_motion_joint_ik(*this, t);

    // Save the joints at this current position
    numarray<mat4> local_joints_now = get_joints_in_chain(animated_model.skeleton.joint_matrix_local);

    numarray<mat4> local_joints_after;
    all_local_joints_after.push_back(local_joints_now);

    int N_time = all_angle_vel.size();
    for (int k_time = 1; k_time < N_time ; k_time++) {

        // compute dt
        float dt = (times[N_time - k_time] - times[N_time - k_time - 1]);

        // add a time
        times.push_back(times[times.size()-1] + dt);

        mat4 global_joint = root_global_joints[k_time - 1];
        // compute the local joints after
        for (int i = 0; i < chain.size(); i++) {
            
            // extract the local orientation at time step t
            mat3 orientation_now = local_joints_now[i].get_block_linear();
            rotation_transform rt_now = rotation_transform::from_matrix(orientation_now);
            quaternion q_now = rt_now.get_quaternion();

            // propagate the motion with forward kinematics
            vec3 ang_vel = all_angle_vel[k_time-1][i];
            quaternion pure_ang_vel = quaternion(ang_vel, 0.f);
            quaternion deriv_q = 0.5f * q_now * pure_ang_vel;
            quaternion q_after = slerp(q_now, normalize(q_now + deriv_q * dt), 1.0f);

            // put the new orientation to the joint after!
            mat4 joint_after = local_joints_now[i];
            joint_after.set_block_linear(rotation_transform(q_after).matrix());

            local_joints_after.push_back(joint_after);

            // calculate the global
            global_joint = global_joint * joint_after;
        }

        // put in the big matrix
        all_local_joints_after.push_back(local_joints_after);

        // add the global position of joint_id to the positions to follow
        positions_to_follow.push_back(global_joint.get_block_translation());

        // update before / now / after
        local_joints_now = local_joints_after;
        local_joints_after.resize_clear(0);

        // find the new distances
        find_distances();
       
    }

}


void Direction::merge_dir_motions(cgp::numarray<Direction>& motions, skeleton_structure skeleton, cgp::numarray<int> cue_ids)
{
    int N_motions;

    // STEP 1 : look for the ones to merge and merge

    // merge motions with same joint_ids
    cgp::numarray<Direction> new_motions;
    bool to_merge = true;
    int i, j, j_merged;
    while(to_merge) {
        to_merge = false;
        i = 0;
        N_motions = motions.size();
        j_merged = -1;

        while(i < N_motions) {
            Direction& m1 = motions[i];
            j = i + 1;
            while(!to_merge && j < N_motions) {
                Direction m2 = motions[j];
                
                // if the merge is needed
                if ((m1.joint_id == m2.joint_id) || is_joint_parent(m1.joint_id, m2.joint_id, skeleton) || is_joint_parent(m2.joint_id, m1.joint_id, skeleton)) {
                    to_merge = true;
                    j_merged = j;
                    if(m1.joint_id < m2.joint_id) {
                        m1.add_lines(m2.lines);
                    } else {
                        m2.add_lines(m1.lines);
                        m1 = m2;
                    }
                    m1.joint_root_ik = 0;
                }
                j++;
            }
            if (i!=j_merged) {
                new_motions.push_back(m1);
            }
            i++;
        }
        motions = new_motions;
        new_motions.clear();
    }

    N_motions = motions.size();


    // find ik chains
    for (int i = 0; i < N_motions ; i++) {
        motions[i].find_chain(skeleton);
    }

    // STEP 2 : find ik roots (from cues)

    // for each motion
    for (int i = 0; i < N_motions ; i++) {
        Direction& dir_motion = motions[i];
        // for each cue id
        for (int cue_id : cue_ids) {
            // if cue_id inside the IK chain
            if (cue_id > dir_motion.joint_root_ik && cue_id < dir_motion.joint_id && dir_motion.chain.contains(cue_id)) {
                // then ik_root is cue_id
                dir_motion.joint_root_ik = cue_id;
                // update the chain
                dir_motion.find_chain(skeleton);
            }
        }
    }

}

numarray<line_structure> Direction::is_impacted(numarray<line_structure> impact_lines, skeleton_structure skeleton)
{
    numarray<line_structure> res_impact_lines; // not impacted if empty

    // for each line representing an impact
    for (int i=0; i<impact_lines.size(); i++) {
        int impact_line_id = impact_lines[i].joint_id;
        // if impact line id is a child of the motion joint_id
        if (is_joint_parent(impact_line_id, joint_id, skeleton) || impact_line_id == joint_id) {
            res_impact_lines.push_back(impact_lines[i]);
        }
    }

    return res_impact_lines;
}


/*
    pre-condition: ordered motions and times with a=0 have been computed
*/
void Direction::precompute_positions_with_impacts(animated_model_structure& animated_model, bool is_global)
{

    // 1) Put the character in its current pose
    animated_model.set_skeleton_from_animation("Idle", 0.0f);
    if(is_global) {
        animated_model.set_skeleton_from_motion_all(*this, times[0]);
    } else {
        animated_model.set_skeleton_from_motion_joint_ik(*this, times[0]);
    }
    

    // 2) Compute initial distance to impact to find which impact id we should use
    int impact_joint_id = 0;
    vec3 pos_impact_joint, pos_impact_drawn; 
    float initial_d = 10000.f; // take the minimum distance
    for (const auto& impact_pair : impacts){ // do with the lines directly instead
        
        int tmp_impact_joint_id = impact_pair.first;
        vec3 tmp_pos_impact_joint = animated_model.skeleton.joint_matrix_global[tmp_impact_joint_id].get_block_translation();
        vec3 tmp_pos_impact_drawn = impact_pair.second;

        if (norm(tmp_pos_impact_joint - tmp_pos_impact_drawn) < initial_d) {
            impact_joint_id = tmp_impact_joint_id;
            pos_impact_joint = tmp_pos_impact_joint;
            pos_impact_drawn = tmp_pos_impact_drawn;
            initial_d = norm(pos_impact_joint - pos_impact_drawn);
        }
    }

    if (joint_id == impact_joint_id) {
        positions_to_follow.resize(N_pos_before + 1);

    } else {

        int nb_steps_front = positions_to_follow.size(); if(is_global) nb_steps_front = 10;
        int max_front_step = N_pos_before - 1 + nb_steps_front;
        
        numarray<vec3> new_positions = positions_to_follow;
        new_positions.resize(N_pos_before+1);

        // Put the character in its current pose
        animated_model.set_skeleton_from_animation("Idle", 0.0f);
        if(is_global) {
            animated_model.set_skeleton_from_motion_all(*this, times[0]);
        } else {
            animated_model.set_skeleton_from_motion_joint_ik(*this, times[0]);
        }
        
        vec3 position_to_follow;
        int step=N_pos_before+1;
        while (step < positions_to_follow.size() && step <= max_front_step)
        {
            // Put the character in its pose
            animated_model.set_skeleton_from_animation("Idle", 0.0f);
            if(is_global) {
                animated_model.set_skeleton_from_motion_all(*this, times[0]);
            } else {
                animated_model.set_skeleton_from_motion_joint_ik(*this, times[step]);
            }
            
            if(is_global) {
               new_positions.push_back(positions_to_follow[step]);

            } else if(animated_model.is_reachable_from_motion_impacts(*this, impact_joint_id, pos_impact_drawn)){ // If reachable
                // Compute the skeleton with the impact, with the Inverse of IK
                position_to_follow = animated_model.set_skeleton_from_motion_impacts(*this);
                new_positions.push_back(position_to_follow);
            } else {
                // stop here
                max_front_step = step;
            }

            step++;
        }
        
        max_front_step = step; // in case of step == positions_to_follow

        // Add positions
        if(is_global){
            new_positions.push_back(positions_to_follow[step]);
        } else {
            new_positions.push_back(position_to_follow);
        }
        
        // go back
        numarray<vec3> positions_back;
        int nb_steps_back = 0.8 * (max_front_step - N_pos_before);
        for(int i=1; i< nb_steps_back; i++) {
            if (step-i >= 0) {
                positions_back.push_back(new_positions[step-i]);
            }
        }
        new_positions.push_back(positions_back);

        // Update new positions_to_follow
        positions_to_follow.clear();
        positions_to_follow = new_positions;
        
    }
    // find distances between the positions
    find_distances();
    
}


void Direction::update_impact(numarray<line_structure> impacting_lines, animated_model_structure& animated_model, bool is_global)
{
    // re compute the skeleton initial movement
    a = 0.0f;
    animate_motion_to_joint(animated_model.skeleton);

    for(line_structure line : impacting_lines) {
        impacts[line.joint_id] = line.pos_impact;
    }

    precompute_positions_with_impacts(animated_model, is_global);
    
    a = 0.9f;
    if(is_global) a = 1.5f;
    animate_motion_to_joint(animated_model.skeleton);
    
}

void Direction::check_impact(numarray<line_structure> impact_lines, animated_model_structure& animated_model)
{
    numarray<line_structure> impacting_lines = is_impacted(impact_lines, animated_model.skeleton);
    // if there is an impact on the motion and it's a new one
    if (impacts.size() < impacting_lines.size()) { 
        update_impact(impacting_lines, animated_model, false);
    }
}

void Direction::check_impact_global(numarray<line_structure> impact_lines, numarray<Direction> dir_motions, animated_model_structure& animated_model)
{
    numarray<line_structure> impacting_lines;
    numarray<int> not_impact_ids; // the impacts already used by other motions
    for (Direction dir : dir_motions) {
        // add to the black list the impacts used by the direction motion
        numarray<line_structure> not_impacting_lines = dir.is_impacted(impact_lines, animated_model.skeleton);
        for (line_structure not_impacting_line : not_impacting_lines) not_impact_ids.push_back(not_impacting_line.joint_id);
    }
    for (line_structure line : impact_lines) {
        if (! not_impact_ids.contains(line.joint_id)) {
            // add to the list the impacts that are not in the black list
            impacting_lines.push_back(line);
        }
    }
    // update the global motion with the good ids
    if (impacting_lines.size() > 0) {
        update_impact(impacting_lines, animated_model, true);
    }
}

void Direction::update_dirs_with_impacts(numarray<Direction>& dir_motions, numarray<line_structure> impact_lines, animated_model_structure& animated_model) 
{
    for (Direction& dir : dir_motions) {
        dir.check_impact(impact_lines, animated_model);
    }
}