#include "GradientDescentSolver.hpp"

#include <iostream>

GradientDescentSolver::GradientDescentSolver (const Function& f, const Gradient& g) : DescentSolver(f, g) {
}

Eigen::VectorXd GradientDescentSolver::descentDirection (Eigen::VectorXd const& x) const {
    Eigen::VectorXd gx(x.rows());
    g(x, gx);

    return -gx;
}

