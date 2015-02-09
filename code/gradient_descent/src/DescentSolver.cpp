#include "DescentSolver.hpp"

#include <iostream>

DescentSolver::DescentSolver (const Function& f, const Gradient& g) : f(f), g(g) {
}

Eigen::VectorXd DescentSolver::solve (Eigen::VectorXd const& x0,
                                      double eps) const {
    Eigen::VectorXd x(x0), gx(x0.rows());
    g(x, gx);

    while (gx.norm() > eps) {
        x = step(x);
        g(x, gx);
    }

    return x;
}

Eigen::VectorXd DescentSolver::step (Eigen::VectorXd const& x) const {
    Eigen::VectorXd dir = descentDirection(x);
    double t = lineSearch(x);
    return x + t * dir;
}

double DescentSolver::lineSearch (Eigen::VectorXd const& x,
                                  double alpha, double beta) const {
    double t = 1;

    Eigen::VectorXd dir = descentDirection(x);

    double lhs = f(x + t * dir),
           rhs = f(x) + alpha * t * dir.dot(dir);

    while (lhs > rhs) {
        dir = descentDirection(x);

        lhs = f(x + t * dir);
        rhs = f(x) + alpha * t * dir.dot(dir);

        t *= beta;
    }

    return t;
}

