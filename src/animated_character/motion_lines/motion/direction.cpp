#include "direction.hpp"

using namespace cgp;


void Direction::find_positions(skeleton_structure skeleton, vec3 t_source, camera_projection_perspective const& P, mat4 const& camera_view_inverse)
{
    line_structure dir_line = get_median_line(skeleton);

    calculate_speed();
    std::cout<<"v = "<<v<<std::endl;

    std::cout<<"dir dir"<<std::endl;

    N_pos_before = dir_line.samples.size();
    
    int N_pos_total = N_pos_before + 10; // N_pos_total > (N_pos_before + 1)
    positions_to_follow.resize(N_pos_total);

    // position of movement
    positions_to_follow[N_pos_before] = t_source;


    float magnitude;
    vec3 axis;
    vec3 vel = dir_line.angular_velocity(axis, magnitude);

    vec3 correction = t_source - dir_line.samples[dir_line.samples.size()-1];

    // before the movement
    for(int i=0; i<N_pos_before;i++){
        positions_to_follow[i] = dir_line.samples[i] + correction;
    }

    // case of close start and end
    if (norm(dir_line.samples[dir_line.samples.size()-1] - dir_line.samples[0]) < 0.1f && dir_line.get_length() > 0.3f) {
        //std::cout<<"boucle"<<std::endl;

        // continue the loop if local
        if (lines[0].type_motion == Line_type::DirT) {

            N_pos_total = N_pos_before + dir_line.samples.size()/2; 
            positions_to_follow.resize(N_pos_total);

            // repeated movement, after = before
            for(int i = N_pos_before+1 ; i < N_pos_total; i++){
                positions_to_follow[i] = dir_line.samples[i - N_pos_before - 1] + correction;
            }

        // global spinning
        } else {
            
            // same position
            for (int i=0 ; i < N_pos_total; i++) {
                positions_to_follow[i] = t_source;
            }

            // the axis equals (0;0;0), compute it again with half of the points
            line_structure half_line = dir_line;
            half_line.samples.resize(dir_line.samples.size()/2);
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

        }

    } else {
        // case of straight dir_line:
        if(axis.x == 0.0f && axis.y == 0.0f && axis.z == 0.0f)
        {
            std::cout<<"ligne"<<std::endl;
            vec3 dir = dir_line.samples[dir_line.samples.size()-1] - dir_line.samples[0];
            vel = v * dir;

            // after the movement
            for(int i = N_pos_before+1 ; i < N_pos_total; i++){
                positions_to_follow[i] = positions_to_follow[i-1] + 0.06f*dir;
            }

        
        } else {
            circle_2D c_2D = find_circle(dir_line.projected_samples);

            // verify if it is not a random curve
            if (dir_line.is_random(c_2D.radius, c_2D.center)) {
                std::cout<<"random"<<std::endl;
                vec3 dir = dir_line.samples[dir_line.samples.size()-1] - dir_line.samples[dir_line.samples.size()-1 - dir_line.samples.size()/10];
                vel = v * dir;

                // after the movement
                for(int i = N_pos_before+1 ; i < N_pos_total; i++){
                    positions_to_follow[i] = positions_to_follow[i-1] + 0.06f*dir;
                }
            
            // case of circular line
            } else {

                vec3 center = unproject(P, camera_view_inverse, c_2D.center, dir_line.depth_2D);

                std::cout<<"arc cercle"<<std::endl;
                quaternion rotation_q = quaternion(axis * sin(magnitude*0.03f), cos(magnitude*0.03f));

                // after the movement
                for(int i = N_pos_before+1 ; i < N_pos_total; i++){
                    vec3 vect = positions_to_follow[i-1] - center;
                    quaternion q_res = rotation_q * quaternion(vect,0.0f) * conjugate(rotation_q);
                    positions_to_follow[i] = q_res.xyz() + center;
                }
            }
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


quaternion slerp(const quaternion& q1, const quaternion& q2, float t) {
    // Compute the dot product (cosine of the angle)
    float dot_product = dot(q1, q2);

    // If dot product is negative, negate one quaternion to take the shortest path
    quaternion q2_copy = q2;
    if (dot_product < 0.0f) {
        q2_copy = -1.0f*q2;
        dot_product = -dot_product;
    }

    // If quaternions are very close, use linear interpolation to avoid instability
    const float THRESHOLD = 0.9995f;
    if (dot_product > THRESHOLD) {
        quaternion result = normalize(q1 + t * (q2_copy - q1));
        return result;
    }

    // Compute the angle between the quaternions
    float theta_0 = acos(dot_product);  // Angle between q1 and q2
    float theta = theta_0 * t;          // Scaled angle

    // Compute sin values
    float sin_theta = sin(theta);
    float sin_theta_0 = sin(theta_0);

    // Compute the interpolation weights
    float s0 = cos(theta) - dot_product * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    // Return the interpolated quaternion
    return normalize(s0 * q1 + s1 * q2_copy);
}




void Direction::find_after_joints(float t_stop, animated_model_structure& animated_model, numarray<Motion> ordered_motions, numarray<int> all_joint_ids)
{
    t_end = t_stop;
    //float dt_after = times[N_pos_before] - times[N_pos_before-1];
    float dt_after = 0.1f*times[N_pos_before];
    std::cout<<"dt_after = "<<dt_after<<std::endl;
    //float dt_after = 0.1f;
    float dt_initial = dt_after;

    all_local_joints_after.resize_clear(0);

    float t = times[N_pos_before];

    // 1) put the skeleton in the pose at time t - dt
    animated_model.set_skeleton_from_animation("Idle", 0.0f);
        
    int i=0;
	while(i<ordered_motions.size() && ordered_motions[i].joint_id>=joint_id ){
		if (ordered_motions[i].joint_id != 0) {
			animated_model.set_skeleton_from_motion_joint_ik(ordered_motions[ordered_motions.size()-1-i], t - dt_after, all_joint_ids);
		}
        i++;
	}

    // 2) save the 3D orientations in its IK chain at this moment (locally)
    numarray<mat4> local_joints_before = get_joints_in_chain(animated_model.skeleton.joint_matrix_local);

    // 3) Put the skeleton at the end of the motion line
    animated_model.set_skeleton_from_animation("Idle", 0.0f);
        
    i=0;
	while(i<ordered_motions.size() && ordered_motions[i].joint_id>=joint_id ){
		if (ordered_motions[i].joint_id != 0) {
			animated_model.set_skeleton_from_motion_joint_ik(ordered_motions[ordered_motions.size()-1-i], t, all_joint_ids);
		}
        i++;
	}

    // 4) save the joints at this current position
    numarray<mat4> local_joints_now = get_joints_in_chain(animated_model.skeleton.joint_matrix_local);

    numarray<mat4> local_joints_after;
    all_local_joints_after.push_back(local_joints_now);

    float alpha = 0.2f;
    int k = N_pos_before;
    while(k < times.size()) {

        // 5) compute angular velocity for each joint in the IK chain

            for (int i = 0; i < chain.size(); i++) {
                // a) extract angular velocity from IK motion

                // extract the local orientation at time step t - dt
                mat3 orientation_before = local_joints_before[i].get_block_linear();
                rotation_transform rt_before = rotation_transform::from_matrix(orientation_before);
                // extract the local orientation at time step t
                mat3 orientation_now = local_joints_now[i].get_block_linear();
                rotation_transform rt_now = rotation_transform::from_matrix(orientation_now);
                quaternion q_now = rt_now.get_quaternion();
                // compute the relative rotation between two consecutive frames
                rotation_transform relative_rt = inverse(rt_before) * rt_now;
                // compute the angular velocity
                vec3 axis;
                float angle;
                relative_rt.to_axis_angle(axis, angle);
                vec3 ang_vel = (2.f*axis*angle/dt_after) * 0.5f * (1.0f - alpha * (t - times[N_pos_before]));
                if (std::abs(angle) < 1e-6f) {
                    ang_vel = vec3(0.0f, 0.0f, 0.0f);
                }

                // b) propagate the motion with forward kinematics
                quaternion pure_ang_vel = quaternion(ang_vel, 0.f);
                quaternion deriv_q = 0.5f * q_now * pure_ang_vel;
                /*quaternion q_after = q_now + deriv_q * dt_after;
                q_after = normalize(q_after);*/
                quaternion q_after = slerp(q_now, normalize(q_now + deriv_q * dt_after), 1.0f);
                if (dot(q_after, q_now) < 0.0f) {
                    q_after = -1.0f*q_after;  // Flip to ensure shortest path
                }


                // c) put the new orientation to the joint after!
                mat4 joint_after = local_joints_now[i];
                joint_after.set_block_linear(rotation_transform(q_after).matrix());

                local_joints_after.push_back(joint_after);
            }

        // put in the big matrix
        all_local_joints_after.push_back(local_joints_after);

        // **Update time step (dt_after decreases over time)**
        //dt_after = dt_initial * (1.0f - alpha * (t - times[N_pos_before]));
        //dt_after = std::max(dt_after, 0.001f);  // Prevent too small dt

        // update before / now / after
        local_joints_before = local_joints_now;
        local_joints_now = local_joints_after;
        local_joints_after.resize_clear(0);
        // next frame
        t += dt_after;
        times[k] = t;
        k ++;
    }

    std::cout<<times<<std::endl;

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
void Direction::precompute_positions_with_impacts(animated_model_structure& animated_model, numarray<Motion> ordered_motions, numarray<int> all_joint_ids, bool is_global)
{
    bool impact_passed=false;
    int max_front_step = positions_to_follow.size() - 1;
    numarray<vec3> new_positions;
    new_positions.push_back(positions_to_follow[0]);
    int impact_step_tmp = positions_to_follow.size() - 1;

    int nb_steps_front = 10;
    int nb_steps_back = 5;

    // 0) if there is already a motion of "boing", delete it
    if (positions_to_follow[max_front_step].x == positions_to_follow[max_front_step - 2*nb_steps_back].x && positions_to_follow[max_front_step].y == positions_to_follow[max_front_step - 2*nb_steps_back].y && positions_to_follow[max_front_step].z == positions_to_follow[max_front_step - 2*nb_steps_back].z) {
        positions_to_follow.resize(positions_to_follow.size() - nb_steps_back);
    }


    // 1) Put the character in its current pose

    animated_model.set_skeleton_from_animation("Idle", 0.0f);
        
    int i=0;
	while(i<ordered_motions.size() && ordered_motions[i].joint_id!=joint_id ){
		if (ordered_motions[i].joint_id != 0) {
			animated_model.set_skeleton_from_motion_joint_ik(ordered_motions[ordered_motions.size()-1-i], times[0], all_joint_ids);
		}
        i++;
	}

    // 2) Compute initial distance to impact
    if(!is_global) {
        animated_model.set_skeleton_from_motion_joint_ik(*this, times[0], all_joint_ids);
    } else {
        animated_model.set_skeleton_from_motion_all(*this, times[0]);
    }
    

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

    for (int step=1; step < positions_to_follow.size(); step++)
    {
        // 1) Put the character in its current pose
        animated_model.set_skeleton_from_animation("Idle", 0.0f);
        
        int i=0;
		while(i<ordered_motions.size() && ordered_motions[i].joint_id!=joint_id){
			if (ordered_motions[i].joint_id != 0) {
				animated_model.set_skeleton_from_motion_joint_ik(ordered_motions[ordered_motions.size()-1-i], times[step], all_joint_ids);
			}
            i++;
		}

        // 3) Compute IK without impact
        if(!is_global) {
            animated_model.set_skeleton_from_motion_joint_ik(*this, times[step], all_joint_ids);
        } else {
            animated_model.set_skeleton_from_motion_all(*this, times[step]);
        }

        // 4) Compute the distance between impact_joint and the impact
        if (!impact_passed) {

            vec3 pos_impact_joint_now = animated_model.skeleton.joint_matrix_global[impact_joint_id].get_block_translation();
            float d = norm(pos_impact_joint_now - pos_impact_drawn);

            // if we are on the impact
            if ( d < initial_d/1000.f ) {
                impact_passed = true;
                impact_step_tmp = step+1;
            } 
        }


        // case of impact on same joint: just stop there
        if (impact_passed && joint_id == impact_joint_id) {
            break;
        }

        if (impact_passed && step <= max_front_step) {
            
            // 5) If reachable:
            if(animated_model.is_reachable_from_motion_impacts(*this, impact_joint_id, pos_impact_drawn)){
                // Compute the skeleton with the impact, with the Inverse of IK
                animated_model.set_skeleton_from_motion_impacts(*this);
                if(step + nb_steps_front < max_front_step){
                    // add nb_steps_front next steps
                    max_front_step = step + nb_steps_front;
                }
            } else {
                // stop here
                max_front_step = step;
                new_positions.resize(step);
            }

        }

        // 7) Add positions
        if (step == max_front_step ) { // && joint_id != impact_joint_id
            new_positions.push_back(positions_to_follow[step]);
            // go back
            for(int i=1; i<=nb_steps_back; i++) {
                if (step-i >= 0) {
                    new_positions.push_back(positions_to_follow[step-i]);
                }
            }
            
            break;

        } else if (step < max_front_step) {
            new_positions.push_back(positions_to_follow[step]);
        }
            

    }

    // 8) Update new positions_to_follow
    positions_to_follow.clear();
    positions_to_follow = new_positions;
    
    this->step_with_impact = impact_step_tmp; // at the end, put this

    // find distances between the positions
    find_distances();
    
}


void Direction::update_impact(float t_drawn, numarray<line_structure> impacting_lines, animated_model_structure& animated_model, numarray<Motion> motions, numarray<int> all_joint_ids, bool is_global)
{
    // re compute the skeleton initial movement
    a = 0.0f;
    animate_motion_to_joint(animated_model.skeleton);

    for(line_structure line : impacting_lines) {
        impacts[line.joint_id] = line.pos_impact;
    }

    if (t_impact < t_drawn) {
        t_impact = t_drawn;
        // not necessary to insert positions because we're taking the positions from Idle
        //int step = get_step_from_time(t_impact);
        //insert_position(animated_model.skeleton.joint_matrix_global[joint_id].get_block_translation(), step, animated_model.skeleton);
    }

    precompute_positions_with_impacts(animated_model, motions, all_joint_ids, is_global);

    a = 0.7f;
    animate_motion_to_joint(animated_model.skeleton);
    
}

void Direction::check_impact(float t_drawn, numarray<line_structure> impact_lines, animated_model_structure& animated_model, numarray<Motion> motions, numarray<int> all_joint_ids)
{
    numarray<line_structure> impacting_lines = is_impacted(impact_lines, animated_model.skeleton);
    // if there is an impact on the motion and it's a new one
    if (impacts.size() < impacting_lines.size()) { 
        update_impact(t_drawn, impacting_lines, animated_model, motions, all_joint_ids, false);
    }
}

void Direction::check_impact_global(float t_drawn, numarray<line_structure> impact_lines, numarray<Direction> dir_motions, animated_model_structure& animated_model, numarray<Motion> motions, numarray<int> all_joint_ids)
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
        update_impact(t_drawn, impacting_lines, animated_model, motions, all_joint_ids, true);
    }
}

void Direction::update_dirs_with_impacts(float t_drawn, numarray<Direction>& dir_motions, numarray<line_structure> impact_lines, animated_model_structure& animated_model, numarray<Motion> motions, numarray<int> all_joint_ids) 
{
    for (Direction& dir : dir_motions) {
        dir.check_impact(t_drawn, impact_lines, animated_model, motions, all_joint_ids);
    }
}