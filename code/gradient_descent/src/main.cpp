#include <iostream>
#include <functional>

#include "GradientDescentSolver.hpp"
#include "NewtonSolver.hpp"
#include "GradientDescentSolverAD.hpp"

#include <CGAL/Simple_cartesian.h>
#include "CGAL_AD.hpp"

typedef CGAL::Simple_cartesian<AD> Kernel_ad;
typedef Kernel_ad::FT FT_ad;
typedef CGAL::Point_2<Kernel_ad> Point;

typedef Eigen::Matrix<FT_ad, Eigen::Dynamic, 1> VectorXAD;
typedef std::function<FT_ad(const VectorXAD &x)> FunctionAD;
typedef GradientDescentSolverAD<AD, FunctionAD, VectorXAD, FT_ad> SolverAD;

int main (void) {
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
    std::cout << f(x) << std::endl;

    /* std::cout << "Newton" << std::endl; */
    /* NewtonSolver ns(f, g, h); */
    /* Eigen::VectorXd xn = ns.solve(start); */
    /* std::cout << xn << std::endl; */

    std::cout << "AD" << std::endl;
    auto fad = [&] (const VectorXAD &x) -> FT_ad {
        return 0.5 * (x[0] * x[0] + gamma * x[1] * x[1]);
    };

    SolverAD gsad(fad);
    VectorXAD startAD(2);
    startAD[0] = AD(1, 2, 0);
    startAD[1] = AD(1, 2, 1);
    VectorXAD sol = gsad.solve(startAD);
    std::cout << toValue(sol) << std::endl;
    std::cout << fad(sol) << std::endl;

    return 0;
}

