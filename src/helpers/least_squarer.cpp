#include "least_squarer.hpp"

using namespace cgp;
using namespace Eigen;

circle_2D find_circle(numarray<vec2> points)
{
    int N = points.size(); // Number of points
    if (N < 3) {
        throw std::runtime_error("At least 3 points are required to fit a circle.");
    }

    // Step 1: Build the sparse matrix A and RHS vector b
    SparseMatrix<double> A(N, 3); // A matrix for [x, y, 1] coefficients
    VectorXd b(N);                // RHS vector for - (x^2 + y^2)

    std::vector<Triplet<double>> triplets;
    for (int i = 0; i < N; ++i) {
        double x = (double)points[i].x;
        double y = (double)points[i].y;

        // A matrix entries: x, y, 1
        triplets.emplace_back(i, 0, x); // Column 0: x_i
        triplets.emplace_back(i, 1, y); // Column 1: y_i
        triplets.emplace_back(i, 2, 1); // Column 2: constant term

        // b vector: - (x^2 + y^2)
        b(i) = -(x * x + y * y);
    }
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();

    // Step 2: Solve the system using Eigen's LeastSquaresConjugateGradient
    LeastSquaresConjugateGradient<SparseMatrix<double>> solver;
    solver.compute(A);
    if (solver.info() != Success) {
        throw std::runtime_error("Solver failed to compute the factorization.");
    }

    VectorXd c = solver.solve(b);
    if (solver.info() != Success) {
        throw std::runtime_error("Solver failed to solve the system.");
    }

    // c contains [a, b, c], which corresponds to the circle parameters
    float a = c(0);
    float b_coeff = c(1);
    float c_const = c(2);

    // Step 3: Convert [a, b, c] to circle center and radius
    float x_c = -a / 2.0;
    float y_c = -b_coeff / 2.0;
    float radius = std::sqrt((a * a + b_coeff * b_coeff) / 4.0 - c_const);

    circle_2D res_circle;
    res_circle.radius = radius;
    res_circle.center = vec2(x_c, y_c);

    return res_circle;
}