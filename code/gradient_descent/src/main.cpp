#include <iostream>

#include "GradientDescentSolver.hpp"
#include "NewtonSolver.hpp"

#include <CGAL/Simple_cartesian.h>
#include "CGAL_AD.hpp"

typedef CGAL::Simple_cartesian<AD> Kernel_ad;
typedef Kernel_ad::FT FT_ad;
typedef CGAL::Point_2<Kernel_ad> Point;

typedef Eigen::Matrix<FT_ad, Eigen::Dynamic, 1> VectorXAD;

int main (void) {
    // Gradient solver
    float gamma = 10;

    Function f = [&] (const Eigen::VectorXd &x) -> double {
        return 0.5 * (x[0] * x[0] + gamma * x[1] * x[1]);
    };

    Gradient g = [&] (const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
        grad[0] = x[0];
        grad[1] = gamma * x[1];
    };

    Hessian h = [&] (const Eigen::VectorXd &x, Eigen::MatrixXd &hess) {
        hess << 1, 0, 0, gamma;
    };

    std::cout << "Gradient descent" << std::endl;
    GradientDescentSolver gs(f, g);
    Eigen::VectorXd start(2);
    start << 1, 1;
    Eigen::VectorXd x = gs.solve(start);
    std::cout << x << std::endl;

    std::cout << "Newton" << std::endl;
    NewtonSolver ns(f, g, h);
    Eigen::VectorXd xn = ns.solve(start);
    std::cout << xn << std::endl;

    std::cout << "AD" << std::endl;

    auto fad = [&] (const VectorXAD &x) -> FT_ad {
        return 0.5 * (x[0] * x[0] + gamma * x[1] * x[1]);
    };

    VectorXAD xad(2);
    xad[0] = AD(1, 2, 0);
    xad[1] = AD(1, 2, 1);
    FT_ad resad = fad(xad);
    std::cout << resad.value() << std::endl;
    std::cout << resad.derivatives() << std::endl;

    std::cout << "CGAL AD" << std::endl;
    Point p(AD(1, 2, 0), AD(1, 2, 1));
    std::cout << p.x().value() << std::endl;
    std::cout << p.x().derivatives() << std::endl;

    std::cout << p.y().value() << std::endl;
    std::cout << p.y().derivatives() << std::endl;

    return 0;
}

