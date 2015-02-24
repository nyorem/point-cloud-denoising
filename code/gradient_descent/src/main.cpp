#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <iostream>
#include <functional>
#include <vector>

#include "GradientUtils.hpp"
#include "PointCloudUtils.hpp"

#include "GradientDescentSolver.hpp"
#include "NewtonSolver.hpp"

#include "CGAL_AD.hpp"
#include "volume_union_balls_2_ad.hpp"

typedef Eigen::VectorXd VectorXd;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point;
typedef Kernel::FT FT;

// AD-related
typedef CGAL::Simple_cartesian<AD> Kernel_ad;
typedef Kernel_ad::FT FT_ad;
typedef CGAL::Point_2<Kernel_ad> Point_ad;

typedef Eigen::Matrix<FT_ad, Eigen::Dynamic, 1> VectorXAD;
typedef std::function<FT_ad(const VectorXAD &x)> FunctionAD;
typedef VolumeUnion<Point_ad, FT_ad, VectorXAD> VolumeUnionAD;

int main (void) {
    // Example function
    float gamma = 10;
    Function f = [&] (const VectorXd &x) -> double {
        return 0.5 * (x[0] * x[0] + gamma * x[1] * x[1]);
    };

    Gradient g = [&] (const VectorXd &x, VectorXd &grad) {
        grad[0] = x[0];
        grad[1] = gamma * x[1];
    };

    auto fad = [&] (const VectorXAD &x) -> FT_ad {
        return 0.5 * (x[0] * x[0] + gamma * x[1] * x[1]);
    };

    // Not useful for now
    /* Hessian h = [&] (const Eigen::VectorXd &x, Eigen::MatrixXd &hess) { */
    /*     hess << 1, 0, 0, gamma; */
    /* }; */

    // Test grad_finite_n
    std::cout << "gradient eval" << std::endl;
    VectorXd x(2);
    x << 1, 1;
    std::cout << grad_finite_1(f, x) << std::endl;
    std::cout << grad_finite_2(f, x) << std::endl;
    GradFinite1Eval<Function, VectorXd> grad_finite_1_eval;
    std::cout << step_gradient_descent(grad_finite_1_eval, f, x, 0.1) << std::endl;
    GradFinite2Eval<Function, VectorXd> grad_finite_2_eval;
    std::cout << step_gradient_descent(grad_finite_2_eval, f, x, 0.1) << std::endl;

    // Test grad_ad
    VectorXAD xad(2);
    xad(0) = AD(x(0), 2, 0);
    xad(1) = AD(x(1), 2, 1);
    std::cout << toValue(grad_ad<FT_ad>(fad, xad)) << std::endl;
    GradAdEval<FT_ad, FunctionAD, VectorXAD> grad_ad_eval;
    std::cout << toValue(step_gradient_descent(grad_ad_eval, fad, xad, 0.1)) << std::endl;

    // Test gradient_descent
    std::cout << "gradient descent" << std::endl;
    std::cout << gradient_descent(grad_finite_1_eval, f, x, 0.1) << std::endl;
    std::cout << gradient_descent(grad_finite_2_eval, f, x, 0.1) << std::endl;
    std::cout << toValue(gradient_descent(grad_ad_eval, fad, xad, 0.1)) << std::endl;

    std::cout << "Volume AD" << std::endl;
    std::vector<Point> points;
    points.push_back(Point(0, 0));
    points.push_back(Point(1.5, 0));
    points.push_back(Point(3, 0));
    /* points.push_back(Point(2, 0)); */
    /* points.push_back(Point(1, 1)); */
    VectorXAD points_vec = pointCloudToVector<VectorXAD>(points.begin(), points.end());
    VolumeUnionAD volume(1);
    std::cout << "volume: " << volume(points_vec) << std::endl;
    Eigen::VectorXd grad = volume.grad();
    std::cout << "gradient: " << grad << std::endl;
    std::cout << "gradients" << std::endl;
    for (int i = 0; i < grad.rows() / 2; ++i) {
        std::cout << grad(2 * i) << ", " << grad(2 * i + 1) << std::endl;
    }

    // Gradient descent using AD
    VectorXAD new_points_vec = step_gradient_descent(grad_ad_eval, volume, points_vec, 0.1);
    std::cout << "new points: " << toValue(new_points_vec) << std::endl;

    return 0;
}

