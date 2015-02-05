#ifndef _GRADIENTDESCENTSOLVER_HPP_
#define _GRADIENTDESCENTSOLVER_HPP_

#include "DescentSolver.hpp"

// Gradient descent solver: descent direction = negated gradient
class GradientDescentSolver : public DescentSolver {
    public:
        GradientDescentSolver (const Function& f, const Gradient& g);

    private:
        Eigen::VectorXd descentDirection (Eigen::VectorXd const& x) const;
};

#endif

