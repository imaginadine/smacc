#include "camera_projecter.hpp"

using namespace cgp;

// Compute a 3D position of a 2D position given by its screen coordinates for perspective projection
vec3 unproject(camera_projection_perspective const& P, mat4 const& camera_view_inverse, vec2 const& p_screen)
{
	// Simple un-project assuming that the viewpoint is an orthogonal projection
	vec4 const p_proj = camera_view_inverse * P.matrix_inverse() *  vec4(p_screen, 0.5f, 1.0f);
	return p_proj.xyz()/p_proj.w;
}

// Compute a 3D position of a 2D position given by its screen coordinates for perspective projection
vec3 unproject(camera_projection_perspective const& P, mat4 const& camera_view_inverse, vec2 const& p_screen, float depth)
{
	// Simple un-project assuming that the viewpoint is an orthogonal projection
	vec4 const p_proj = camera_view_inverse * P.matrix_inverse() *  vec4(p_screen, depth, 1.0f);
	return p_proj.xyz()/p_proj.w;
}
