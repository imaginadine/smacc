#include "character_loader.hpp"
#include "../environment.hpp"

using namespace cgp;

character_structure load_character_xbot() {
	filename_loader_structure loader_param;
	loader_param.set_skeleton(project::path+"assets/xbot/skeleton/");
	loader_param.add_rigged_mesh("body1", project::path+"assets/xbot/mesh-xbot/mesh-beta_surface/", project::path+"assets/xbot/mesh-xbot/RobotSkin.png");
	loader_param.add_rigged_mesh("body2", project::path+"assets/xbot/mesh-xbot/mesh-beta_joints/", project::path+"assets/xbot/mesh-xbot/RobotSkin.png");
	loader_param.add_animation("Idle", project::path+"assets/xbot/animation/idle/");
	loader_param.add_animation("Jump", project::path+"assets/xbot/animation/jump/");
	loader_param.add_animation("Punch", project::path+"assets/xbot/animation/punching/");
	loader_param.add_animation("Jazz", project::path+"assets/xbot/animation/jazz/");
	loader_param.add_animation("Wtf", project::path+"assets/xbot/animation/wtfdance/");
	loader_param.add_animation("Throw", project::path+"assets/xbot/animation/throw/");
	loader_param.add_animation("Swim", project::path+"assets/xbot/animation/swimming/");
	loader_param.add_animation("Climb", project::path+"assets/xbot/animation/climbing/");
	loader_param.add_animation("Knee", project::path+"assets/xbot/animation/knee/");
	loader_param.add_animation("Custom", project::path+"assets/xbot/animation/custom-dance/");
	loader_param.add_animation("Action", project::path+"assets/xbot/animation/action/");
	loader_param.add_animation("Crouch", project::path+"assets/xbot/animation/crouch/");
	loader_param.add_animation("Custom-Simple", project::path+"assets/xbot/animation/custom-simple/");
	loader_param.add_animation("Zombie", project::path+"assets/xbot/animation/zombie/");
	loader_param.add_animation("Fist", project::path+"assets/xbot/animation/standingFistPump/");
	loader_param.add_animation("Gun", project::path+"assets/xbot/animation/sittingGunMotion/");
	loader_param.add_animation("Angry", project::path+"assets/xbot/animation/sittingAngry/");
	loader_param.add_animation("Pontera", project::path+"assets/xbot/animation/pontera/");
	loader_param.add_animation("Picking", project::path+"assets/xbot/animation/pickingUpObject/");
	loader_param.add_animation("Jostled", project::path+"assets/xbot/animation/jostledPassenger/");
	loader_param.add_animation("Jab", project::path+"assets/xbot/animation/jabToElbow/");
	loader_param.add_animation("Hands", project::path+"assets/xbot/animation/handsForward/");
	loader_param.add_animation("Falling", project::path+"assets/xbot/animation/fallingFlatImpact/");
	loader_param.add_animation("SitUp", project::path+"assets/xbot/animation/situps/");
	loader_param.add_animation("FrontRaises", project::path+"assets/xbot/animation/frontRaises/");
	

	character_structure character;
	character.load_and_initialize(loader_param, affine_rts().set_scaling(0.01f));

	return character;
}

character_structure load_cow() {
	filename_loader_structure loader_param;
	loader_param.set_skeleton(project::path+"assets/cow/skeleton/");
	loader_param.add_rigged_mesh("Cow",project::path+"assets/cow/mesh-cow/", project::path+"assets/cow/mesh-cow/texture.png");
	loader_param.add_animation("Idle", project::path+"assets/cow/animation/idle/");

	character_structure character;
	rotation_transform rt = rotation_transform::from_matrix(mat3({0.f,1.f,0.f,0.f,0.f,1.f,1.f,0.f,0.f}));
	character.load_and_initialize(loader_param, affine_rts().set_rotation(rt));

	return character;
}