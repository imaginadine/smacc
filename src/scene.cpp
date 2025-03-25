#include "scene.hpp"

#include "character_loader/character_loader.hpp"


using namespace cgp;


void initialize_ground(mesh_drawable& ground);

float scene_structure::calculate_animation_duration()
{
	float max_duration = 3.0f; //seconds

	for (Direction m: motion_dirs) {
		float m_duration = m.times[m.times.size()-1];
		if(m_duration > max_duration) {
			max_duration = m_duration;
		}
	}

	/*if(global_motion.lines.size()>0) {
		float m_duration_global = global_motion.times[global_motion.times.size()-1];
		if(m_duration_global > max_duration) {
			max_duration = m_duration_global;
		}
	}*/

	return max_duration + 0.5f;
}


void scene_structure::update_root_iks()
{
	// find cue_ids
	numarray<int> cue_ids;
	for(Cue c : motion_cues){
		cue_ids.push_back(c.joint_id);
	}
	// find dir_ids
	numarray<int> dir_ids;
	for(Direction d : motion_dirs){
		dir_ids.push_back(d.joint_id);
	}
	// 1) cue
	if(motion_cues.size() > 0) {
		// merge cues with same ids
		Cue::merge_cue_motions(motion_cues, dir_ids, characters["Lola"].animated_model.skeleton);
	}
	// 2) dir
	if(motion_dirs.size() > 0) {
		Direction::merge_dir_motions(motion_dirs, characters["Lola"].animated_model.skeleton, cue_ids);
	}
	// 3) all
	numarray<std::shared_ptr<Motion>> all_motions;
	// Populate all_motions
	for (const auto& cue : motion_cues) {
		all_motions.push_back(std::make_shared<Cue>(cue));
	}
	for (const auto& dir : motion_dirs) {
		all_motions.push_back(std::make_shared<Direction>(dir));
	}
	Motion::find_roots_ik(all_motions, characters["Lola"].animated_model.skeleton);
	// Retrieve derived objects back
	for (size_t i = 0; i < motion_cues.size(); i++) {
		auto cue_ptr = std::dynamic_pointer_cast<Cue>(all_motions[i]);
		if (cue_ptr) {
			motion_cues[i] = *cue_ptr; // Dereference the smart pointer
		}
	}
	for (size_t i = 0; i < motion_dirs.size(); i++) {
		auto dir_ptr = std::dynamic_pointer_cast<Direction>(all_motions[motion_cues.size() + i]);
		if (dir_ptr) {
			motion_dirs[i] = *dir_ptr; // Dereference the smart pointer
		}
	}
	// test :
	for(Cue cue : motion_cues) {
		std::cout<<"Cue IK Chain = "<<cue.chain<<std::endl;
	}
	for(Direction dir : motion_dirs) {
		std::cout<<"Dir IK Chain = "<<dir.chain<<std::endl;
	}

}


void scene_structure::build_empty_motions()
{
	motions.clear();
	motion_dirs.clear();
	motion_cues.clear();
	global_motion.clear();
	impact_lines.clear();

	// for each line
	for (int i=0 ; i<lines.size();i++) {
		line_structure new_line = lines[i];

		// Global Direction
		if(new_line.type_motion == Line_type::GlobT) {
			// new global motion
			if (global_motion.lines.size()==0) {
				Direction dir = Direction(new_line, 0, gui.method);
				global_motion = dir;
			
			} else { // add lines to the existing global motion
				numarray<line_structure> new_lines;
				new_lines.push_back(new_line);
				global_motion.add_lines(new_lines);
			}
		}

		// Direction
		if(new_line.type_motion == Line_type::DirT) {
			Direction dir = Direction(new_line, new_line.joint_id, gui.method);
			motion_dirs.push_back(dir);
		}
				
		// Circumfixing lines
		if(new_line.type_motion == Line_type::CueT) {
			Cue cue = Cue(new_line, new_line.joint_id, gui.method);
			motion_cues.push_back(cue);
		}

		if(new_line.type_motion == Line_type::ImpT) {
			impact_lines.push_back(new_line);
		}

	}
}


/*
	Call this when an update is needed, like at the start of the character loop
*/
void scene_structure::update_character()
{

	// update / find the new root ik joints
	update_root_iks();

	// update all joint ids
	all_joint_ids.clear();
	for(Motion m : motion_dirs){
		all_joint_ids.push_back(m.joint_id);
	}

	// Animate the motions
	for(Direction& dir_m : motion_dirs){
		dir_m.find_positions(characters["Lola"].animated_model.skeleton, characters["Lola"].animated_model.skeleton.joint_matrix_global[dir_m.joint_id].get_block_translation(), camera_projection, camera_control.camera_model.matrix_frame());
		dir_m.animate_motion_to_joint(characters["Lola"].animated_model.skeleton);
	}
	for(Cue& cue_m : motion_cues){
		cue_m.find_positions(characters["Lola"].animated_model.skeleton, characters["Lola"].animated_model.skeleton.joint_matrix_global[cue_m.joint_id].get_block_translation(), camera_projection, camera_control.camera_model.matrix_frame());
		cue_m.animate_motion_to_joint(characters["Lola"].animated_model.skeleton);
	}
	if(global_motion.lines.size()>0){
		global_motion.find_positions(characters["Lola"].animated_model.skeleton, characters["Lola"].animated_model.skeleton.joint_matrix_global[global_motion.joint_id].get_block_translation(), camera_projection, camera_control.camera_model.matrix_frame());
		global_motion.animate_motion_to_joint(characters["Lola"].animated_model.skeleton);
	}

	// manage impacts
	if (impact_lines.size()>0) {
		Direction::update_dirs_with_impacts(characters["Lola"].timer.t_periodic, motion_dirs, impact_lines, characters["Lola"].animated_model, motions, all_joint_ids);
	}
	if(global_motion.lines.size()>0) {
		global_motion.check_impact_global(characters["Lola"].timer.t_periodic, impact_lines, motion_dirs, characters["Lola"].animated_model, motions, all_joint_ids);
	}

	// order the motions to know which is calculated first
	order_motions_by_joint_id();

	// define time : the longest animation
	characters["Lola"].timer.event_period = calculate_animation_duration();
}


void scene_structure::initialize()
{
	environment.background_color = vec3(169.f/255.f,234.f/255.f,254.f/255.f);

	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();
	camera_control.look_at({ 4.0f, 3.0f, 3.0f }, {0,0,0}, {0,0,1});
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());
	

	sphere_ik.initialize_data_on_gpu(mesh_primitive_sphere());
	sphere_ik.model.scaling = 0.05f;
	sphere_ik.material.color = {1,0,0};

	initialize_ground(ground);

	
	std::cout<<"- Load Lola character"<<std::endl;
	characters["Lola"] = load_character_xbot();
	characters["Lola"].animated_model.skeleton.init_constraints();
	if(characters["Lola"].timer.event_period < 3.f) characters["Lola"].timer.event_period = 3.f;

	current_active_character = "Lola";

	for(auto& entry : characters)
		entry.second.timer.start();

}

void scene_structure::order_motions_by_joint_id()
{
	motions.clear();
	// Populate all_motions
	for (Motion cue : motion_cues) {
		motions.push_back(cue);
	}
	for (Motion dir : motion_dirs) {
		motions.push_back(dir);
	}

	// order them
	int N = motions.size();
	Motion tmp;

	for (int i = 0 ; i< N-1 ; i++) {
		for (int k = i+1 ; k < N ;k++) {
			if (motions[k].joint_id < motions[i].joint_id) {
				// exchange values between current and min
				tmp = motions[i];
				motions[i] = motions[k];
				motions[k] = tmp;
			}
		}		
	}

}

void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	
	if (gui.display_frame)
		draw(global_frame, environment);

	if(gui.display_ground) 
		draw(ground, environment);

	// Update the local time for each character
	for(auto& entry: characters) {
		entry.second.timer.update();
	}

	// ************************************************* //
	// Update the current skeleton of each character
	// ************************************************* //
	for(auto& entry : characters) {
		std::string character_name = entry.first;
		character_structure& character = entry.second;

		character.animated_model.set_skeleton_from_animation("Idle", 0.267f);

		if(!gui.sketch_mode) {
			for(int i=0; i<motions.size();i++){
				if (motions[i].joint_id != 0) {
					character.animated_model.set_skeleton_from_motion_joint_ik(motions[motions.size()-1-i], character.timer.t_periodic, all_joint_ids);
				}
			}

			if(global_motion.lines.size()>0) character.animated_model.set_skeleton_from_motion_all(global_motion, character.timer.t_periodic);
		}

		
	}

	// ********************************** //
	// Compute Skinning deformation
	// ********************************** //
	for(auto& entry_character : characters) {
		animated_model_structure& animated_model = entry_character.second.animated_model;
		for(auto& rigged_mesh_entry : animated_model.rigged_mesh) {
			std::string mesh_name = rigged_mesh_entry.first;
			animated_model.skinning_lbs(mesh_name);
		}
	}

	// ************************************** //
	// Display the surface and the skeletons
	// ************************************** //
	for(auto& entry_character : characters) {
		character_structure& character = entry_character.second;
		animated_model_structure& animated_model = entry_character.second.animated_model;

		// Display meshes
		for(auto& rigged_mesh_entry : animated_model.rigged_mesh) {
			std::string mesh_name = rigged_mesh_entry.first;
			rigged_mesh_structure& rigged_mesh = rigged_mesh_entry.second;
			
			mesh_drawable& drawable = character.drawable[mesh_name];
			drawable.vbo_position.update(rigged_mesh.mesh_deformed.position);
			drawable.vbo_normal.update(rigged_mesh.mesh_deformed.normal);

			if(gui.display_surface) {
				drawable.material.texture_settings.active = gui.display_texture;
				draw(drawable, environment);
			}
			if(gui.display_wireframe) {
				draw_wireframe(drawable, environment);
			}
		}

		// Display skeleton
		if(gui.display_skeleton) {
			character.sk_drawable.update(animated_model.skeleton);
			character.sk_drawable.display_joint_frame = gui.display_skeleton_joint_frame;
			character.sk_drawable.display_joint_sphere = gui.display_skeleton_joint_sphere;
			character.sk_drawable.display_segments = gui.display_skeleton_bone;
			draw(character.sk_drawable, environment);
		}

	}

	// ************************************** //
	// Display the lines
	// ************************************** //

	for(line_structure line : lines)
	{
		draw(line, environment);
	}

	for (int k = 0; k < sketch_drawable.size(); ++k) {
		draw(sketch_drawable[k], environment);
	}

}

// end of delete mode
void scene_structure::stop_delete_mode(){
	if(delete_mode){
		// put all lines to default color, black
		if(lines.size()>0) {
			lines[id_to_remove].set_color(vec3(0.f,0.f,0.f));
		}
	}
}

void scene_structure::deselect_clusters()
{
	if(gui.selected_motion != -1) {
		Motion motion_to_deselect;
		if(gui.selected_motion == motions.size()) {
			motion_to_deselect = global_motion;
		} else {
			motion_to_deselect = motions[gui.selected_motion];
		}
		for(line_structure m_line : motion_to_deselect.lines) {
			int id_line = find_line_from_cluster_line(m_line);
			lines[id_line].set_color(vec3(0.f,0.f,0.f));
		}
		gui.selected_motion = -1;
	}
	
}


void scene_structure::switch_benchmark(bool to_global)
{
	// find the cluster
	Motion selected_motion;
	if(gui.selected_motion == motions.size()) {
		selected_motion = global_motion;
	} else {
		selected_motion = motions[gui.selected_motion];
	}

	// to local or global benchmark?
	Line_type benchmark = Line_type::DirT;
	if(to_global) {
		benchmark = Line_type::GlobT;
	}

	// change the type of the lines of the motion to GlobT
	for(line_structure m_line : selected_motion.lines) {
		int id_line = find_line_from_cluster_line(m_line);
		lines[id_line].type_motion = benchmark;
	}

	// start again: like the sketch mode was ended
	deselect_clusters();
	characters["Lola"].animated_model.set_skeleton_from_animation("Idle", 0.267f);
	old_sketch_mode = true;
	gui.sketch_mode = false;

}


void scene_structure::push_motion_button(int i, Motion motion)
{
	std::string button_label = "Cluster " + std::to_string(i + 1);

	bool is_selected = (gui.selected_motion == i);

	if (is_selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 1.0f, 1.0f)); // Highlight selected button

	if (ImGui::Button(button_label.c_str())) {
		if(!is_selected) {

			// new selection of the button
			Motion old_motion;
			int old_selected_motion = gui.selected_motion;
			gui.selected_motion = i;
			if(old_selected_motion != -1){
				if(old_selected_motion == motions.size()) {
					old_motion = global_motion;
				} else {
					old_motion = motions[old_selected_motion];
				}
			}
			

			// handle change of selected motion cluster
			if(old_selected_motion != gui.selected_motion) {
				// change the color of the lines in the cluster to yellow
				for(line_structure m_line : motion.lines) {
					int id_line = find_line_from_cluster_line(m_line);
					lines[id_line].set_color(vec3(1.f,1.f,0.f));
				}
				// and the last selected to black
				if(old_selected_motion != -1) {
					for(line_structure m_line : old_motion.lines) {
						int id_line = find_line_from_cluster_line(m_line);
						lines[id_line].set_color(vec3(0.f,0.f,0.f));
					}
				}

			}

		} else {
			deselect_clusters();
		}

	}

	if (is_selected) ImGui::PopStyleColor();

	ImGui::SameLine();
}


void scene_structure::display_gui()
{

	ImGui::Checkbox("Sketch Mode", &gui.sketch_mode);

	if (gui.sketch_mode) {
		deselect_clusters();

		if (ImGui::RadioButton("Trajectory", is_action)) {stop_delete_mode(); is_action = true; is_cue = false; is_impact = false; delete_mode = false;}; ImGui::SameLine();
		if (ImGui::RadioButton("Circumfixing", is_cue)) {stop_delete_mode(); is_cue = true; is_action = false; is_impact = false; delete_mode = false;}; ImGui::SameLine();
		if (ImGui::RadioButton("Impact", is_impact)) {stop_delete_mode(); is_impact = true; is_action = false; is_cue = false; delete_mode = false;};

		if (ImGui::RadioButton("Delete", delete_mode)) { is_impact = false; is_action = false; is_cue = false; delete_mode = true;};
	} else {

		// motion buttons here
		// locals
		for (int i = 0; i < motions.size(); i++) {
			push_motion_button(i, motions[i]);
		}

		// global
		if(global_motion.lines.size()>0) {
			push_motion_button(motions.size(), global_motion);
		}


		ImGui::Spacing();
		// apparition of new parameters
		if (gui.selected_motion != -1) {
			bool is_dir_motion = false;

			bool is_local;
			bool is_global;
			// case of global direction
			if(gui.selected_motion == motions.size() && global_motion.lines.size()>0){
				is_local = false;
				is_global = true;
				is_dir_motion = true;
			// case of local direction
			} else if(motions[gui.selected_motion].lines[0].type_motion == Line_type::DirT) {
				is_local = true;
				is_global = false;
				is_dir_motion = true;
			}

			// for a Direction motion
			if(is_dir_motion) {
				if (ImGui::RadioButton("Local", is_local)) {
					if(!is_local) switch_benchmark(false);
				}; ImGui::SameLine();
				if (ImGui::RadioButton("Global", is_global)) {
					if(!is_global) switch_benchmark(true);
				};
			}
		}
	}



	ImGui::Spacing(); ImGui::Spacing(); ImGui::Separator(); 

	// General display functions
	ImGui::Checkbox("Global Frame", &gui.display_frame); ImGui::SameLine();
	ImGui::Checkbox("Ground", &gui.display_ground);
	ImGui::Checkbox("Wireframe", &gui.display_wireframe); 
	ImGui::Checkbox("Display surface", &gui.display_surface); ImGui::SameLine();
	ImGui::Checkbox("Texture", &gui.display_texture);
	ImGui::Checkbox("Skeleton", &gui.display_skeleton);

	ImGui::Indent();
	ImGui::Checkbox("Bone", &gui.display_skeleton_bone); ImGui::SameLine();
	ImGui::Checkbox("Joint", &gui.display_skeleton_joint_sphere); ImGui::SameLine();
	ImGui::Checkbox("Frame", &gui.display_skeleton_joint_frame);
	ImGui::Unindent();

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Spacing(); ImGui::Separator(); 
	bool is_method1_clicked = ImGui::RadioButton("Rigidity", gui.method == 1); ImGui::SameLine();
	bool is_method2_clicked = ImGui::RadioButton("Expressivity", gui.method == 2); // method = 2

	bool is_constraint_clicked = ImGui::Checkbox("Constraints", &gui.constraint); ImGui::SameLine();
	ImGui::Unindent();

	ImGui::Spacing(); ImGui::Spacing();

	// Display info for all characters
	for(auto& entry : characters) {

		ImGui::Spacing(); ImGui::Spacing(); ImGui::Separator(); 

		std::string name = entry.first;
		auto& character = entry.second;

		// Timers associated to the character
		ImGui::Indent();
		std::string time_scale_txt = "Time scale##"+name;
		ImGui::SliderFloat(time_scale_txt.c_str(), &character.timer.scale, 0.0f, 2.0f);

		std::string local_time_txt = "Anim cycle time##"+name;
		ImGui::SliderFloat(local_time_txt.c_str(), &character.timer.t_periodic, 0.0f, character.timer.event_period);
				
		ImGui::Unindent();
	}

	// Handle change of gui constraint
	if(is_constraint_clicked)
	{
		for (Motion& m : motion_dirs)
		{
			m.is_constrained = gui.constraint;
		}
	}

	// Handle change of method
	if (is_method1_clicked || is_method2_clicked) {
		if(is_method1_clicked) gui.method = 1;
		if(is_method2_clicked) gui.method = 2;

		// TO MODIFY

		/*numarray<line_structure> void_lines;
		for (Direction& motion_dir : motion_dirs) {
			motion_dir.method = gui.method;
			motion_dir.add_lines(void_lines, characters["Lola"].animated_model.skeleton, camera_projection, camera_control.camera_model.matrix_frame()); // change void lines
		}

		// define time : the longest animation
		characters["Lola"].timer.event_period = calculate_animation_duration();*/
	}

	// Hangle end of gui mode
	if(old_sketch_mode != gui.sketch_mode){
		// entering in normal mode from sketch mode
		if(old_sketch_mode) {
				stop_delete_mode();
			
				build_empty_motions();

				update_character();
				sketch_drawable.clear();	

			// the animation begins again
			characters["Lola"].timer.scale = tmp_scale;

		} else { // entering in sketch mode
			tmp_scale = characters["Lola"].timer.scale;
			characters["Lola"].timer.scale = 0.f;
		}
		old_sketch_mode = gui.sketch_mode;
	}
}

void scene_structure::mouse_move_event()
{
	if (!inputs.mouse.on_gui) {
		if (gui.sketch_mode) {

			if(delete_mode) {
				// save the id of the line that is closest to the mouse position
				if(lines.size()>1) {
					int old_id_to_remove = id_to_remove;
					id_to_remove = get_closest_line_id_2D(inputs.mouse.position.current, lines, camera_projection, inverse(camera_control.camera_model.matrix_frame()));
					// change to red if needed
					if ( (old_id_to_remove != id_to_remove) || (id_to_remove == 0 && equals(lines[0].get_color(),vec3(0.f,0.f,0.f))) ) {
						lines[id_to_remove].set_color(vec3(1.f,0.f,0.f));
						if (old_id_to_remove < lines.size()) lines[old_id_to_remove].set_color(vec3(0.f,0.f,0.f));
					}
				} else if(lines.size()==1){
					id_to_remove = 0;
					if(equals(lines[id_to_remove].get_color(),vec3(0.f,0.f,0.f))){
						lines[id_to_remove].set_color(vec3(1.f,0.f,0.f));
					}
				}

			} else {

				if (inputs.mouse.click.left) {
					// Add the new clicked position
					int k_sketch = sketch_drawable.size() - 1;
					vec3 const p = unproject(camera_projection, camera_control.camera_model.matrix_frame(), inputs.mouse.position.current);
					sketch_drawable[k_sketch].push_back(p);
					// non projected
					current_line_projected_positions.push_back(inputs.mouse.position.current);

					previous_clicked_left = true;

				// release mouse clicking
				} else if(previous_clicked_left) {

					// delete the last drawn line
					int k_sketch = sketch_drawable.size()-1;
					sketch_drawable[k_sketch].clear();
					
					if (is_wrapping_object(current_line_projected_positions, characters["Lola"].animated_model, camera_projection, inverse(camera_control.camera_model.matrix_frame()))) {
						std::cout<<"Draw the next line please :)"<<std::endl;
						current_line_projected_positions.clear();
						// delete the last drawn line
						int k_sketch = sketch_drawable.size()-1;
						sketch_drawable[k_sketch].clear();
						sketch_drawable.resize(k_sketch);
						is_global_movement = true;
					} else {

						float depth;
						int joint_id_found = find_joint_from_2D_line(current_line_projected_positions, characters["Lola"].animated_model, camera_projection, inverse(camera_control.camera_model.matrix_frame()), depth);
						printf("found joint = %d with depth = %f\n", joint_id_found, depth);

						// if a joint was found
						if(joint_id_found != -1)
						{
							// project with the good depth
							numarray<vec3> depth_projected_positions;
							// replace the last drawn line  by the new:
							for (int i=0; i<current_line_projected_positions.size();i++)
							{
								vec3 const p = unproject(camera_projection, camera_control.camera_model.matrix_frame(), current_line_projected_positions[i], depth);
								depth_projected_positions.push_back(p);
							}

							// create the new line
							line_structure new_line;
							new_line.init_line(depth_projected_positions, current_line_projected_positions, depth, camera_projection, camera_control.camera_model.matrix_frame());
							new_line.joint_id = joint_id_found;
							new_line.pos_impact = characters["Lola"].animated_model.skeleton.joint_matrix_global[joint_id_found].get_block_translation();

							if(is_global_movement) {
								new_line.type_motion = Line_type::GlobT;
								is_global_movement = false;
							} else {
								if(is_action) new_line.type_motion = Line_type::DirT;
								if(is_cue) new_line.type_motion = Line_type::CueT;
								if(is_impact) new_line.type_motion = Line_type::ImpT;
							}

							lines.push_back(new_line);
						}

						// clean
						current_line_projected_positions.clear();
					}

					previous_clicked_left = false;
				}
			}

			
		}
		else {
			if (!inputs.keyboard.shift)
				camera_control.action_mouse_move(environment.camera_view);
		}
	}
}


void scene_structure::mouse_click_event()
{
	if (!inputs.mouse.on_gui) {
		if (gui.sketch_mode) {

			if(inputs.mouse.click.last_action == last_mouse_cursor_action::click_left) {

				if(delete_mode) {
					// remove the selected line
					if (lines.size()>0) lines = remove_at_index(lines, id_to_remove);
				} else {
					// Create new stroke (curve_dynamic_drawable)
					int k_sketch = sketch_drawable.size();
					sketch_drawable.push_back(curve_drawable_dynamic_extend());
					sketch_drawable[k_sketch].initialize_data_on_gpu();

					// Add the new clicked position
					vec3 const p = unproject(camera_projection, camera_control.camera_model.matrix_frame(), inputs.mouse.position.current);
					sketch_drawable[k_sketch].push_back(p);

					// currrent projected
					current_line_projected_positions.clear();
					current_line_projected_positions.push_back(inputs.mouse.position.current);
				}	
			}

		} else {
			camera_control.action_mouse_click(environment.camera_view);
		}
	}
}

void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);

	if(effect_walk.active){
		effect_walking_keyboard_event(effect_transition[current_active_character], characters[current_active_character], inputs, effect_walk);
	}
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}

void initialize_ground(mesh_drawable& ground) {
	mesh ground_mesh = mesh_primitive_quadrangle();
	ground_mesh.translate({-0.5f,-0.5f,0.0f});
	ground_mesh.rotate({1,0,0},-Pi/2.0f);
	ground_mesh.translate({0.0f,0.0f,-0.05f});
	ground_mesh.scale(15.0f);
	ground_mesh.uv *= 5.0f;
	
	ground.initialize_data_on_gpu(ground_mesh);
	ground.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/texture_wood.jpg",GL_REPEAT,GL_REPEAT);
	ground.material.phong = {1,0,0,1};

}

int scene_structure::find_line_from_cluster_line(line_structure m_line)
{
	int id_found = -1;
	for(int i=0; i<lines.size();i++) {
		if(lines[i].equals(m_line)){
			id_found = i;
		}
	}
	return id_found;
}
