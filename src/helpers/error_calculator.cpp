#include "error_calculator.hpp"

using namespace cgp;



float mean_error(numarray<float> angles_ref, numarray<float> angles_drawn)
{
    float mean_error = 0.f;
    int N = angles_ref.size();
    for (int k=0; k<N; k++) {

        float angle_ref = angles_ref[k];
        while(angle_ref < 0) { angle_ref += 2.f*Pi; }
        while(angle_ref >= 2.f*Pi) { angle_ref -= 2.f*Pi; }

        float angle_drawn = angles_drawn[k];
        while(angle_drawn < 0) { angle_drawn += 2.f*Pi; }
        while(angle_drawn >= 2.f*Pi) { angle_drawn -= 2.f*Pi; }

        float angle = std::abs(angle_ref - angle_drawn);
        mean_error += angle;
    }
    mean_error = mean_error / float(N);
    return mean_error;
}


float compare_for_one_frame(float t, std::string anim, numarray<Motion> motions, Motion global_motion, animated_model_structure& animated_model)
{
    // calculate the angles for the drawn animation

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

    numarray<float> angles_drawn = animated_model.skeleton.get_joint_angles();
    vec3 pos_root_drawn = animated_model.skeleton.joint_matrix_global[0].get_block_translation();

    // calculate the angles for the reference animation

    animated_model.set_default_pose();
    animated_model.set_skeleton_from_animation(anim, t);

    numarray<float> angles_ref = animated_model.skeleton.get_joint_angles();
    vec3 pos_root_ref = animated_model.skeleton.joint_matrix_global[0].get_block_translation();

    // calculate the mean error between the angles
    return (mean_error(angles_ref, angles_drawn));
}


float compare_for_all_anim(std::string anim, numarray<Motion> motions, Motion global_motion, character_structure& character){

    float mean_t_error = 0.0f;
    int n = 0;

    for(float t = 0.0f; t < character.timer.event_period; t += 0.2f){
        float err = compare_for_one_frame(t, anim, motions, global_motion, character.animated_model);
        std::cout<<"The mean angular error for t = "<<t<<" is "<<err<<std::endl;
        mean_t_error += err;
        n++;
    }

    mean_t_error = mean_t_error/float(n);
    std::cout<<"The total mean angular error for all frames is "<<mean_t_error<<std::endl;
    return mean_t_error;
}