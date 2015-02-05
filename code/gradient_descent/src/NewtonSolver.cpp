#include "NewtonSolver.hpp"

NewtonSolver::NewtonSolver (const Function& f, const Gradient& g, const Hessian& h) :
    DescentSolver(f, g), h(h) {
}

Eigen::VectorXd NewtonSolver::descentDirection (Eigen::VectorXd const& x) const {
    Eigen::VectorXd gx(x.rows());
    g(x, gx);

    Eigen::MatrixXd hx(x.rows(), x.rows());
    h(x, hx);

    return -hx.inverse() * gx;
}

