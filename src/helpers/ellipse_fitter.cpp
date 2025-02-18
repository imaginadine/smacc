#include "ellipse_fitter.hpp"

using namespace cgp;

// Function to compute the gradient of the system
void computeGradientAndHessian(const vec2& p, float a, float b, float theta, float& gradient, float& hessian) {
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    
    float x = a * cos_theta;
    float y = b * sin_theta;

    float dx_dtheta = -a * sin_theta;
    float dy_dtheta = b * cos_theta;

    // Compute the gradient (first derivative)
    gradient = 2.0 * (x - p.x) * dx_dtheta + 2.0 * (y - p.y) * dy_dtheta;

    // Compute the Hessian (second derivative)
    hessian = 2.0 * (dx_dtheta * dx_dtheta + (x - p.x) * (-a * cos_theta)) +
              2.0 * (dy_dtheta * dy_dtheta + (y - p.y) * (-b * sin_theta));
}


vec2 find_nearest_point_ellipse(float a, float b, vec2 p) {

    float tol = 1e-6f;
    int maxIter = 100;

    // Initial guess for theta
    float theta = atan2(p.y / b, p.x / a);

    for (int i = 0; i < maxIter; i++) {
        float gradient = 0.0f, hessian = 0.0f;
        computeGradientAndHessian(p, a, b, theta, gradient, hessian);

        // Newton-Raphson step
        float delta_theta = -gradient / hessian;

        // Update theta
        theta += delta_theta;

        // Check for convergence
        if (cgp::abs(delta_theta) < tol) {
            break;
        }
    }

    // Compute the nearest point on the ellipse
    float x = a * cos(theta);
    float y = b * sin(theta);

    return vec2(x,y);
}
