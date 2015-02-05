#ifndef _NEWTONSOLVER_HPP_
#define _NEWTONSOLVER_HPP_

#include "DescentSolver.hpp"

typedef std::function<void(const Eigen::VectorXd &x, Eigen::MatrixXd &hess)> Hessian;

// Newton method solver
class NewtonSolver : public DescentSolver {
    public:
        NewtonSolver (const Function& f, const Gradient& g, const Hessian& h);

    private:
        Eigen::VectorXd descentDirection (Eigen::VectorXd const& x) const;

    protected:
        Hessian h;
};

#endif

