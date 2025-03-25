#include "anim_saver.hpp"

using namespace cgp;

numarray<int> get_ordered_joints_from_file(std::string path)
{
    std::ifstream file(path+"/skeleton_animation_joint_index.txt");
    if (!file) {
        std::cerr << "Error opening reading file!" << std::endl;
        return 1;
    }

    numarray<int> values;
    std::string line;
    
    while (std::getline(file, line)) { // Read line by line
        std::stringstream ss(line);
        double value;
        while (ss >> value) { // Read space-separated values
            values.push_back(value);
        }
    }

    file.close();

    // Display read values
    std::cout << "Values read from file:" << std::endl;
    for (const auto& val : values) {
        std::cout << val << " ";
    }
    std::cout << std::endl;

    return values;
}


numarray<mat4> joints_for_one_frame(float t, numarray<Motion> motions, Motion global_motion, animated_model_structure& animated_model)
{
    animated_model.set_default_pose();

	for(int i=0; i<motions.size();i++){
		Motion motion_used = motions[motions.size()-1-i];
		if (motion_used.joint_id != 0) {
			// we continue the line with the movement based on the angles of before, IF it's a direction with no impacts
			if (motion_used.lines[0].type_motion == Line_type::DirT && t > motion_used.times[motion_used.N_pos_before] && motion_used.impacts.size() == 0 && motion_used.t_end > motion_used.times[motion_used.N_pos_before]) {
				animated_model.set_skeleton_from_ending_joints(motion_used, t);
			} else {
				animated_model.set_skeleton_from_motion_joint_ik(motion_used, t);
			}
		}
	}
	if(global_motion.lines.size()>0) animated_model.set_skeleton_from_motion_all(global_motion, t);

    numarray<mat4> local_joints = animated_model.skeleton.joint_matrix_local;
    for(int k=0; k<local_joints.size(); ++k) {
        local_joints[k].apply_scaling_to_block_translation(100.0f);
    }

    return local_joints;
}


void save_anim(std::string path, numarray<Motion> motions, Motion global_motion, character_structure& character) 
{
    numarray<float> timings;
    numarray<numarray<mat4>> joints_at_time;
    for(float t = 0.0f; t < character.timer.event_period; t += 0.04f){
        // save t
        timings.push_back(t);

        // save the global joints
        joints_at_time.push_back(joints_for_one_frame(t, motions, global_motion, character.animated_model));
    }

    int N_joints = character.animated_model.skeleton.joint_matrix_local.size();


    // write timing file

    std::ofstream timing_file(path+"/skeleton_animation_timing.txt");
    if (!timing_file) {
        std::cerr << "Error opening output file!" << std::endl;
    }

    for(int i=0; i< N_joints; i++) {
        for (const auto& val : timings) {
            timing_file << val << " ";
        }
        timing_file << "\n";
    }

    timing_file.close();


    // write joint file
    numarray<int> matrix_ids = get_ordered_joints_from_file(path);

    std::ofstream joint_file(path+"/skeleton_animation_matrix.txt");
    if (!joint_file) {
        std::cerr << "Error opening output file!" << std::endl;
    }

    for(int k = 0; k < matrix_ids.size(); k++) {

        // id line:
        int mat_id = matrix_ids[k];
        if(mat_id < N_joints) {
            // for all the timings
            for(int i=0; i < timings.size();i++) {
                // get the joint mat_id of time i
                mat4 joint_mat = joints_at_time[i][mat_id];
                // write the whole matrix
                joint_file << joint_mat << " ";
            }
           
        }

        // other joint = other line
        joint_file << "\n";
    }

    joint_file.close();
}
