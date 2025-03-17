#include "motion.hpp"
#include "direction.hpp"
#include "cue.hpp"

#include "../../skeleton_animation/skeleton_animation.hpp"

using namespace cgp;


Motion::Motion(line_structure line, int id, int method_to_give)
    : joint_id(id), method(method_to_give) {
    lines.push_back(line);
}

Motion::Motion(line_structure line, skeleton_structure skeleton, int method_to_give)
    : method(method_to_give)
{
    
    joint_id = find_motion_id(line, skeleton);

    if (joint_id == 0) {
        std::cout << "No joint found, global movement" << std::endl;
        method = 0;
        old_method = 0;
    }
    lines.push_back(line);

}

float Motion::get_a()
{
    return a;
}

void Motion::clear()
{
    positions_to_follow.clear();
    joints.clear();
    times.clear();
    lines.clear();
    impacts.clear();
    chain.clear();
}


/**
    Find the closest joint id
*/
int find_motion_id(line_structure line, skeleton_structure& skeleton)
{
    // find the id of the joint on which the movement will be
    float min_dist = 5.0f;
    int joint_id = 0;
    // for all the joints
    for(int k=1; k<skeleton.joint_matrix_global.size();k++){
        mat4 M_source = skeleton.joint_matrix_global[k];
        affine_rt a_source = affine_rt::from_matrix(M_source);
        vec3 joint_pos = a_source.translation;
        // if the joint is in the approximate direction of the line
        vec3 last_pos = line.samples[line.samples.size() - 1];
        vec3 vec1 = last_pos - line.samples[line.samples.size() - 2];
        vec3 vec2 = joint_pos - last_pos;
        if(dot(vec1, vec2) > 0){
            float dist = sqrt( (joint_pos.x - last_pos.x)*(joint_pos.x - last_pos.x) + (joint_pos.y - last_pos.y)*(joint_pos.y - last_pos.y) + (joint_pos.z - last_pos.z)*(joint_pos.z - last_pos.z) );
            // if the joint is close enough
            if(dist < min_dist){
                min_dist = dist;
                joint_id = k;
            }
        }
    }

    if (joint_id != 0) {
        std::cout << "Joint id found: " << joint_id << ", with distance: " << min_dist << std::endl;
    }

    return joint_id;
}


void Motion::set_joint_root_ik(int id_root)
{
    joint_root_ik = id_root;
}

int Motion::get_joint_root_ik()
{
    return joint_root_ik;
}

float sigmoid(float x, float k) {
    return 1.0f / (1.0f + exp(-k * x));
}

void Motion::find_distances()
{
    distances.clear();
    int N_positions = positions_to_follow.size();
    for(int i = 1; i < N_positions ; i++){
        float d = norm(positions_to_follow[i] - positions_to_follow[i-1]);
        distances.push_back(d);
    }
    dist_total = sum(distances);
}

float Motion::speed(int step, numarray<float> old_times)
{
    float speed = v;

    numarray<float> covered_distances = distances;
    covered_distances.resize(step);
    float covered_dist = sum(covered_distances);
    float ratio = covered_dist / dist_total;

    // case of acceleration with impacts
    if(a != 0.0f){

        if(ratio > 0.25f) speed = v + a * exp(ratio);

    } else if(lines[0].type_motion != Line_type::CueT){

        float k = 2.5f; // Steepness of acceleration/deceleration (increase for sharper effect)

        // if an end or start time have been defined
        if(old_times.size() > 0 && (t_end - t_start != 0.0f) && (t_end - t_start != 1000.0f) && (t_end - t_start != old_times[old_times.size()-1])){
            ratio = (old_times[step] -  old_times[0])/ (t_end - t_start);
        } else {
            ratio = covered_dist / (dist_total*2.0f);
        }

        // Define thresholds for acceleration, max speed, and deceleration
        float accel_ratio = 0.1f;  // First 10% is acceleration
        float decel_ratio = 0.9f;  // Last 10% is deceleration
        float min_factor = 0.2f;

        float speed_factor;
        if (ratio < accel_ratio) {
            // Acceleration phase: Sigmoid from 0 to 1
            speed_factor = sigmoid((ratio / accel_ratio - 0.5f) * k, k);
        } 
        else if (ratio > decel_ratio) {
            // Deceleration phase: Flipped sigmoid from 1 to 0
            float decel_progress = (ratio - decel_ratio) / (1.0f - decel_ratio);
            speed_factor = 1.0f - sigmoid((decel_progress - 0.5f) * k, k);
        } 
        else {
            // Max speed phase: Constant speed
            speed_factor = 1.0f;
        }
        if(speed_factor < min_factor) speed_factor = min_factor;
        speed = v * speed_factor;
    }
    
    return speed;
}

// give a motion (defined by some positions and a velocity) to a joint
void Motion::animate_motion_to_joint(skeleton_structure& skeleton)
{
    bool global_spinning = (lines[0].type_motion == Line_type::GlobT && (norm(lines[0].samples[lines[0].samples.size()-1] - lines[0].samples[0]) < 0.1f && lines[0].get_length() > 0.3f));
	// fetch the joint
    mat4 M_global = skeleton.joint_matrix_global[joint_id];

    int N_positions = positions_to_follow.size();
	// verify that motion_positions > 1
    assert_cgp_no_msg(positions_to_follow.size() > 1);

    numarray<float> old_times = times;
    // interpolation between the positions with time given by velocity
    times.clear();

    if(global_spinning) {
        times.push_back(t_start); //t0
        float delta = 0.06f/float(lines.size());
        for (int i = 1; i < N_positions; i++) {
            times.push_back(times[times.size()-1] + delta);
        }

    } else {
        times.push_back(t_start); //t0
        for (int i = 1; i < N_positions; i++) {
            // 1) calculate the distance between two positions
            float d = distances[i-1];
            // 2) calculate the time of the position
            float t = d/speed(i, old_times);
            times.push_back(times[times.size()-1] + t);
        }
        
        //joints
        joints.resize_clear(N_positions);
        // 3) find the associated joint
        for (int i = 0; i < N_positions; i++) {
            joints[i] = M_global.set_block_translation(positions_to_follow[i]);
        }

    }
    
}


line_structure Motion::get_closest_line(skeleton_structure skeleton)
{
    int N_lines = lines.size();

    // take the line that is the closest to the joint for the direction
    vec3 joint_pos = skeleton.joint_matrix_global[joint_id].get_block_translation();
    float min_dist = norm(joint_pos - lines[0].samples[lines[0].samples.size()-1]);
    int closest_line_id = 0;
    // find the closest line
    for (int k_line = 1; k_line < N_lines ; k_line ++) {
        line_structure l = lines[k_line];
        float dist_l = norm(joint_pos - l.samples[l.samples.size()-1]);
        if (dist_l < min_dist) {
            min_dist = dist_l;
            closest_line_id = k_line;
        }
    }

    return lines[closest_line_id];
}


line_structure Motion::get_median_line(skeleton_structure skeleton)
{
    numarray<line_structure> ordered_lines = lines;
    vec3 joint_pos = skeleton.joint_matrix_global[joint_id].get_block_translation();

    // order them by their distance to the joint
	int N = ordered_lines.size();
	line_structure tmp;

    std::cout<<"The lines belonging to the motion are:"<<std::endl;
    for (int i = 0 ; i< N ; i++) {
        std::cout<<" - last_pos = "<<lines[i].samples[lines[i].samples.size()-1]<<std::endl;
    }

	for (int i = 0 ; i< N-1 ; i++) {
        float dist_i = norm(joint_pos - ordered_lines[i].samples[ordered_lines[i].samples.size()-1]);
		for (int j = i+1 ; j < N ;j++) {
            float dist_j = norm(joint_pos - ordered_lines[j].samples[ordered_lines[j].samples.size()-1]);
			if (dist_j < dist_i) {
				tmp = ordered_lines[i];
				ordered_lines[i] = ordered_lines[j];
				ordered_lines[j] = tmp;
			}
		}		
	}

    std::cout<<"Within the "<<N<<" lines, the medium one is with last_pos = "<<ordered_lines[N/2].samples[ordered_lines[N/2].samples.size()-1]<<std::endl;

    // the median is the middle line in the ordered ones
    return ordered_lines[N/2];
}


void Motion::calculate_speed()
{
    // find speed
    float coeff = 0.8f;
    v = 0.0f;
    for (line_structure l : lines) {
        float total_length = l.get_length();
        float local_v = coeff * total_length;
        if (local_v>1.2f) local_v = 1.2f;
        if (local_v<0.1f) local_v = 0.1f;
        v += local_v;
    }
}


void Motion::add_lines(numarray<line_structure> lines_to_add)
{
    lines.push_back(lines_to_add);
}


void Motion::find_positions(skeleton_structure skeleton, vec3 t_source)
{
    line_structure dir_line = get_closest_line(skeleton);

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


mat4 Motion::evaluate(float t) {

    cgp::numarray<float> const& time_array = times;
    int N_time = time_array.size();

    int idx0; // index_placement: i such that t_i < t < t_{i+1}
    float alpha; // ratio_placement: the relative position of t between its two discrete key time r = (t-t_i) / (t_{i+1}-t_i)
    find_relative_placement_in_array(time_array, t, idx0, alpha);


    mat4 M;
    if(t > t_end) {
        int end_step = get_step_from_time(t_end);
        M = joints[end_step];
    } else {
        if(idx0<N_time-1){
            mat4 const& M0 = joints[idx0];
            mat4 const& M1 = joints[idx0+1];
            M = (1.0f-alpha)*M0 + alpha*M1;
        }
        if(idx0>=N_time-1) {
            M = joints[N_time-1];
        }
    }
    

    return M;
}


mat4 Motion::evaluate_end(int id_joint_in_chain, float t) {

    cgp::numarray<float> time_array = times;
    int N_time = time_array.size();

    int idx0; // index_placement: i such that t_i < t < t_{i+1}
    float alpha; // ratio_placement: the relative position of t between its two discrete key time r = (t-t_i) / (t_{i+1}-t_i)
    find_relative_placement_in_array(time_array, t, idx0, alpha);

    mat4 M;
    if(t > t_end) {
        M = all_local_joints_after[get_step_from_time(t_end)- N_pos_before][id_joint_in_chain];
    } else {
        if(idx0<N_time-1){
            mat4 const& M0 = all_local_joints_after[idx0 - N_pos_before][id_joint_in_chain];
            mat4 const& M1 = all_local_joints_after[idx0+1 - N_pos_before][id_joint_in_chain];
            M = (1.0f-alpha)*M0 + alpha*M1;
        }
        if(idx0>=N_time-1) {
            M = all_local_joints_after[N_time-1 - N_pos_before][id_joint_in_chain];
        }
    }

    return M;
}



bool is_joint_parent(int child, int parent, skeleton_structure skeleton)
{
    bool res;

    if (skeleton.parent_index[child] == parent) {
        res = true;
    } else if (child < parent || child == 0 || child == -1) {
        res = false;
    } else {
        res = is_joint_parent(skeleton.parent_index[child], parent, skeleton);
    }

    return res;
}


// needs to have joint_id and joint_root_ik
void Motion::find_chain(skeleton_structure skeleton)
{
    chain.clear();

    // Compute the chain of positions, from the end effector to the root
	int current_joint_index = joint_id;
    int start_joint_index = joint_root_ik;

	while(current_joint_index!=start_joint_index && current_joint_index!=0) {
		chain.push_back(current_joint_index);
		current_joint_index = skeleton.parent_index(current_joint_index);
	}
	chain.push_back(start_joint_index);

    // Inverse the chain : from the root to the end effector
	numarray<int> c = chain;
	for(int k=0; k<chain.size(); ++k) {
		chain[k] = c[chain.size()-k-1];
	}
}

void Motion::update_skeleton(skeleton_structure& skeleton)
{
    for (int i = chain.size()-2; i >= 0; i--) {
        int joint_index = chain[i];
        int joint_child_index = chain[i+1];
        skeleton.joint_matrix_global[joint_index] = skeleton.joint_matrix_global[joint_child_index] * skeleton.joint_matrix_local[joint_child_index].inverse_assuming_rigid_transform();
    }
}

void Motion::find_roots_ik(numarray<std::shared_ptr<Motion>>& motions, skeleton_structure skeleton)
{
    int N_motions = motions.size();

    // verify that it does not start at the root
    for (int i = 0; i < N_motions ; i++) {
        if(motions[i]->joint_root_ik == 0){
            motions[i]->joint_root_ik = motions[i]->chain[1];
            motions[i]->find_chain(skeleton);
        }
    }            

    // find ik roots (from others)

    // for each motion, relative to another
    for (int i = 0; i < N_motions ; i++) {
        for (int j = 0; j < N_motions ; j++) { 
            if (i != j) { 
                std::shared_ptr<Motion>& m1 = motions[i];
                std::shared_ptr<Motion>& m2 = motions[j];
                
                // if a root is the parent of the joint_id of the other motion, problem
                while ( (m2->joint_root_ik < m2->joint_id) && ( is_joint_parent(m1->joint_id, m2->joint_root_ik, skeleton) || !is_joint_parent(m2->joint_id, m2->joint_root_ik, skeleton) ) ) {
                    m2->joint_root_ik ++;
                }
                m2->find_chain(skeleton);
            }
        }
    }

}

int Motion::get_step_from_time(float t)
{
    int i = 0;
    while(i <= times.size()-1 && times[i] < t){
        i++;
    }
    return i-1;
}

void Motion::insert_position(vec3 pos_to_insert, int step, skeleton_structure skeleton)
{
    numarray<vec3> new_positions_to_follow;
    for (int i=0; i<=step; i++) {
        new_positions_to_follow.push_back(positions_to_follow[i]);
    }
    new_positions_to_follow.push_back(pos_to_insert);
    for(int i = step+1; i<positions_to_follow.size();i++) {
        new_positions_to_follow.push_back(positions_to_follow[i]);
    }

    positions_to_follow.clear();
    positions_to_follow = new_positions_to_follow;

    animate_motion_to_joint(skeleton);
}
