#include "effects.hpp"

using namespace cgp;



// Update a transition_structure before starting a new transition
void effect_transition_start(effect_transition_structure& effect_transition, character_structure& character, std::string const& destination_anim)
{
	effect_transition.source_anim = character.current_animation_name;
	effect_transition.timer_source_anim = character.timer;

	effect_transition.destination_anim = destination_anim;
	effect_transition.timer_destination_anim.event_period = character.animated_model.animation.at(destination_anim).time_max;
	effect_transition.timer_destination_anim.t_periodic=0;
	effect_transition.timer_destination_anim.start();

	effect_transition.timer_completion.t=0;
	effect_transition.timer_completion.scale = 1.0f/effect_transition.transition_time; // force the timer to go from 0 to 1 during the transition_time
	effect_transition.timer_completion.start();
	effect_transition.active = true;

	character.current_animation_name = destination_anim;
}

// Check if a transition need to be de-activated if it is fully completed
void effect_transition_stop_if_completed(effect_transition_structure& transition, character_structure& character) {
	if(transition.timer_completion.t >= 1.0f) {
		character.set_current_animation(transition.destination_anim);
		character.timer = transition.timer_destination_anim;
		transition.active = false;
	}
}


void effect_transition_compute(effect_transition_structure& effect_transition, character_structure& character)
{

	effect_transition_structure& transition = effect_transition;
	animated_model_structure& model = character.animated_model;

	// Compute the skeleton from the source animation
	model.set_skeleton_from_animation(transition.source_anim, transition.timer_source_anim.t_periodic);
	numarray<mat4> joint_source_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_source_anim_global = model.skeleton.joint_matrix_global;

	// Compute the skeleton from the destination animation
	model.set_skeleton_from_animation(transition.destination_anim, transition.timer_destination_anim.t_periodic);
	numarray<mat4> joint_destination_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_destination_anim_global = model.skeleton.joint_matrix_global;

	float alpha_completion = transition.timer_completion.t;

	/** TO DO : Compute a smooth transition between two animations 
	 *  The objective is to fill "model.skeleton.joint_matrix_local and global"
	 *     with the correct interpolated matrices corresponding to the blending between the source and destination animation 
	 * 
	 * 	
	 * 
	 *  Notes:
	 *    - model.skeleton.joint_matrix_local is a numarray<mat4>. Its size corresponds to the number of joints.
	 *    - joint_source_anim contains the (local) matrices of the source animation
	 *    - joint_destination_anim contains the (local) matrices of the destination animation
	 * 	  - alpha_completion contains the ratio of completion.
	 *       e.g.: alpha_completion=0 => we have the source anim
	 *             alpha_completion=1 => we have the destination anim
	 *             alpha_completion=0.5 => we have "0.5 source anim + 0.5 destination anim"
	 *    - if model.skeleton.joint_matrix_local is filled, you can update the global matrices using "model.skeleton.update_joint_matrix_local_to_global();"
	 *    - if model.skeleton.joint_matrix_global is filled, you can update the global matrices using "model.skeleton.update_joint_matrix_global_to_local();"
	 * 
	 *  Help:
	 *    - You can convert a matrix M to its translation t and rotation r (or quaternion q) representation using the following syntax:
	 *      affine_rt a = affine_rt::from_matrix(M);
	 *      vec3 t = a.translation;
	 *      rotation_transform r = a.rotation;
	 *      quaternion q = r.get_quaternion();
	 *    - An affine_rt structure is a container for a rotation_transform and a translation. Its corresponding 4x4 matrix can be obtained
	 *        with {affine_rt}.matrix();
	 * 
	 * 
	 */

	numarray<mat4> transition_joints;

	for(int k_joint=0; k_joint<joint_source_anim_local.size();k_joint++){
		mat4 M_source = joint_source_anim_local[k_joint];
		affine_rt a_source = affine_rt::from_matrix(M_source);
      	vec3 t_source = a_source.translation;
	    rotation_transform r_source = a_source.rotation;
		quaternion q_source = r_source.get_quaternion();

		mat4 M_dest = joint_destination_anim_local[k_joint];
		affine_rt a_dest = affine_rt::from_matrix(M_dest);
      	vec3 t_dest = a_dest.translation;
	    rotation_transform r_dest = a_dest.rotation;
		quaternion q_dest = r_dest.get_quaternion();

		vec3 new_t = (1.0f-alpha_completion)*t_source + alpha_completion*t_dest;
		// take the shortest path
		float dot_val = dot(q_source, q_dest);
		if (dot_val < 0.0f){
			q_dest = -1.0f*q_dest;
		}
		quaternion new_q = (1.0f-alpha_completion)*q_source + alpha_completion*q_dest;

		rotation_transform new_r = rotation_transform(normalize(new_q));
		affine_rt new_affine = affine_rt(new_r, new_t);

		mat4 M = new_affine.matrix();
		//mat4 M = (1.0f-alpha_completion)*M_source + alpha_completion*M_dest;
		transition_joints.push_back(M);
	}

	model.skeleton.joint_matrix_local = transition_joints;
	model.skeleton.update_joint_matrix_local_to_global();

	// Update the timers
	transition.timer_source_anim.update();
	transition.timer_destination_anim.update();
	transition.timer_completion.update();
	character.timer = transition.timer_destination_anim;
}




// This function implements the change of animation when we press or release the UP key during a walk effect
void effect_walking_keyboard_event(effect_transition_structure& effect_transition, character_structure& character, cgp::input_devices const& inputs, effect_walking_structure const& effect_walking)
{
	// If we just press the key up (or W), start the Walk cycle animation
	if(inputs.keyboard.last_action.is_pressed(GLFW_KEY_UP) || inputs.keyboard.last_action.is_pressed(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.walk_anim_name);
	}
	// If we just release the key up (or W), start the Idle cycle animation
	if(inputs.keyboard.last_action.is_released(GLFW_KEY_UP) || inputs.keyboard.last_action.is_released(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.idle_anim_name);
	}

}


// This function implements the change of position of the character when a directional key is pressed
// This function is called at every frame when the walk effect is active
void effect_walking(effect_walking_structure& effect_walking,  character_structure& character, cgp::input_devices const& inputs, effect_transition_structure const& effect_transition)
{

	/** TO DO : A displacement of the character along the direction of the walk.
	 *   The character should be able to move straight, and turn, based on the user command.
	 *  
	 *   Inputs: 
	 *      - The effect_walking structure storing the current position and angle of the character (root joint position and angle)
	 *      - The skeleton to be modified by the walk effect
	 *      - The current state of the keyboard input (e.g. inputs.keyboard.is_pressed(...))
	 *   Output: 
	 * 		- A modified effect_walking structure and skeleton corresponding to the expected displacement
	 * 		  
	 * 
	 *   Help:
	 *      - The following syntax allows to check if a specific key is currently pressed:
	 *        if( inputs.keyboard.is_pressed({KeyName}) ) {...} , with
	 * 		   {KeyName} being: GLFW_KEY_UP, GLFW_KEY_RIGHT, GLFW_KEY_LEFT, GLFW_KEY_DOWN for the arrows
	 *                      or: GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_D, GLFW_KEY_S using for instance the letters
	 *      - Given a mat4 structure representing an affine transformation, you can apply the following block operations
	 *        - {mat4}.get/set_block_translation({vec3}) : get/set the vec3 block corresponding to the translation part
	 *        - {mat4}.get/set_block_linear({mat3}) : get/set the mat3 block corresponding to the linear part
	 *      - A rotation with a given axis and angle can be created with the syntax
	 *         rotation_transform r = rotation_axis_angle({vec3 axis}, {float angle});
	 *         The associated 3x3 matrix can be obtained using r.matrix();  
	 *      - An internal timer is available in effect_walking.timer
	 *        The timer can be updated via effect_walking.timer.update() and the elapsed time since the last update is returned.
	 *        This can be used to enforce a constant speed independently of the frame rate
	 * 
	 * 
	 * 	 Algorithm:     
	 * 		If(press UP)
	 *        Advance straight and update effect_walking.root_position
	 *      If(press Left/Right)
	 *        Rotate and update effect_walking.root_angle
	 *      Update the root joint of the skeleton
	 * 
	 * 
	 */

	float elapsed_t = effect_walking.timer.update();
	float angle = 0.1f * 3.14f/4.0f; // pi/4 * force

	// update effect walking structure
	if( inputs.keyboard.is_pressed(GLFW_KEY_UP) ){
		//find direction vector
		effect_walking.root_position.x += elapsed_t * sin(effect_walking.root_angle);
		effect_walking.root_position.z += elapsed_t * cos(effect_walking.root_angle);
		effect_walking.root_position.y = character.animated_model.skeleton.joint_matrix_global[0].get_block_translation().y;
	}

	if (inputs.keyboard.is_pressed(GLFW_KEY_RIGHT)) {
		effect_walking.root_angle -= angle;
	}
	if (inputs.keyboard.is_pressed(GLFW_KEY_LEFT)) {
		effect_walking.root_angle += angle;
	}
	
	// Update the root joint of the position
	mat4 transformation_matrix = mat4(1.0f); // Identity matrix
	transformation_matrix.set_block_translation(effect_walking.root_position);
	rotation_transform rt = rotation_axis_angle(vec3(0.0f,1.0f,0.0f), effect_walking.root_angle);
	mat3 linear_part_mat = rt.matrix();
	transformation_matrix.set_block_linear(linear_part_mat);

	character.animated_model.skeleton.joint_matrix_global[0] = transformation_matrix;
	character.animated_model.skeleton.joint_matrix_local[0] = transformation_matrix;
	character.animated_model.skeleton.update_joint_matrix_local_to_global();
}



// Update the current position of the IK constraint to match at its start the current joint position
void effect_ik_start(effect_ik_structure& effect_ik, skeleton_structure& skeleton, int joint_end)
{
	// get current position of the joint at index joint_end
	vec3 current_position = skeleton.joint_matrix_global[joint_end].get_block_translation();
	// set this position as the default objective for the IK
	effect_ik.target_position = current_position;
	effect_ik.target_offset = {0,0,0};
}


void effect_ik_compute(effect_ik_structure const& effect_ik, skeleton_structure& skeleton)
{
	/** TO DO : Implement an Inverse Kinematic on the skeleton
	 *  
	 *   Inputs: 
	 *      - The skeleton to be modified by the IK
	 *      - The index of the joint constrained by the IK : effect_ik.joint_end
	 *      - The index of the joint at the beginning of the skeleton chain : effect_ik.joint_start
	 *        (effect_ik.joint_start should be a parent of effect_ik.joint_end, otherwise the start of the chain can be considered to be the skeleton root)
	 *      - The position associated to constrained joint: 
	 *          p_constrained = effect_ik.target_position + effect_ik.target_offset
	 *        Rem. this position is split between an initial position and an offset that can be manipulated via the GUI.
	 *   Output: 
	 * 		- A modified joint position of the skeleton satisfying at best the constraint.
	 * 
	 *   Rem. 
	 *      - effect_ik.joint_target should be a parent of effect_ik.joint_end such that the set of joints between [joint_start - joint_end] is a kinematic chain, subset of the skeleton hierarchy. If it is not the case, we may assume that the start of the chain is at the skeleton root.
	 * 
	 *   Help:
	 *      - A rotation R transforming a vector v1 to a vector v2 (with ||v1||=||v2||=1) can be computed using
	 *        rotation_transform R = rotation_transform::from_vector_transform(v1, v2);
	 *      - Considering a mat4 representing an affine transform:
	 *         - {mat4}.get_block_translation() gives the vec3 corresponding to the translation part
	 *         - {mat4}.set_block_translation({vec3}) set the block to the mat4 corresponding to the translation with the vec3 passed as argument
	 *         - {mat4}.apply_transform_to_block_linear({mat3}) : Applies the mat3 matrix to the linear part of the mat4 matrix (doesn't modify the translation part)
	 *         - mat4 M_inv = {mat4}.inverse_assuming_rigid_transform() : gives the 4x4 matrix corresponding to the inverse of the {mat4}
	 *      - mat4().set_identity() generate a 4x4 identity matrix
	 *      
	 */

	float tol = 0.01;

	vec3 p_constrained = effect_ik.target_position + effect_ik.target_offset;
	vec3 p_target = effect_ik.target_position;

	numarray<float> distances;
	numarray<vec3> positions;
	int id = effect_ik.joint_target;
	int end_id = effect_ik.joint_root_ik;
	// fill the tab with the positions and the distances
	positions.push_back(p_target);
	while(id!=end_id){
		mat4 joint_child = skeleton.joint_matrix_global[id];
		int parent_id = skeleton.parent_index[id];
		mat4 joint_parent = skeleton.joint_matrix_global[parent_id];
		vec3 position_child = joint_child.get_block_translation();
		vec3 position_parent = joint_parent.get_block_translation();
		
		positions.push_back(position_parent);
		distances.push_back(norm(position_parent-position_child));

		id = parent_id;
	} //OK

	float dist = norm(positions[positions.size()-1] - p_constrained); //OK
	float total_dist = sum(distances);
	float r_i;
	float lambda_i; //OK

	//target reachable?
	if(dist>total_dist){
		//no
		// for all points
		for(int i=positions.size()-1; i>0;i--){
			vec3 p_i = positions[i];
			r_i = norm(p_constrained-p_i);
			lambda_i = distances[i-1]/r_i;
			positions[i-1] = (1-lambda_i)*p_i + lambda_i*p_constrained;
		} //OK
	}else{
		//yes
		vec3 b = positions[positions.size()-1]; //OK
		float diff_a = norm(positions[0]-p_constrained);//OK
		while(diff_a>tol){
			// STAGE 1
			positions[0] = p_constrained;
			for(int i=1;i<positions.size()-1;i++){
				r_i = norm(positions[i-1]-positions[i]);//OK
				lambda_i = distances[i-1]/r_i;//OK
				positions[i] = (1-lambda_i)*positions[i-1] + lambda_i*positions[i];// OK
			}
			// STAGE 2
			positions[positions.size()-1] = b;//OK
			for(int i=positions.size()-1;i>0;i--){//OK
				r_i = norm(positions[i-1]-positions[i]);//OK
				lambda_i = distances[i-1]/r_i;
				positions[i-1] = (1-lambda_i)*positions[i] + lambda_i*positions[i-1];//OK
			}
			diff_a = norm(positions[0]-p_constrained);//OK
		}
	}

	// apply the positions on the joints
	id = effect_ik.joint_target;
	int k_pos = 0;
	while(id!=end_id){
		vec3 old_position_child = skeleton.joint_matrix_global[id].get_block_translation();
		vec3 new_position_child = positions[k_pos];
		
		rotation_transform R = rotation_transform::from_vector_transform(normalize(old_position_child), normalize(new_position_child));
		affine_rt af_rt = affine_rt(R, new_position_child);
		skeleton.joint_matrix_global[id] = af_rt.matrix();

		id = skeleton.parent_index[id];
		k_pos++;
	}
	// for the last one too
	vec3 old_position_child = skeleton.joint_matrix_global[end_id].get_block_translation();
	vec3 new_position_child = positions[positions.size()-1];
	rotation_transform R = rotation_transform::from_vector_transform(normalize(old_position_child), normalize(new_position_child));
	affine_rt af_rt = affine_rt(R, new_position_child);
	skeleton.joint_matrix_global[end_id] = af_rt.matrix();

	//update skin and joints
	//skeleton.update_joint_matrix_global_to_local();

}
