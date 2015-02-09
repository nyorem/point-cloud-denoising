#ifndef _DESCENTSOLVER_HPP_
#define _DESCENTSOLVER_HPP_

#include <functional>
#include <Eigen/Dense>

typedef std::function<double(const Eigen::VectorXd &x)> Function;
typedef std::function<void(const Eigen::VectorXd &x, Eigen::VectorXd &grad)> Gradient;

class DescentSolver {
    public:
        DescentSolver (const Function& f, const Gradient& g);

        Eigen::VectorXd solve (Eigen::VectorXd const& x0,
                               double eps = 0.01) const;

    public:
        Function f;
        Gradient g;

        // Gives the descent direction at x
        virtual Eigen::VectorXd descentDirection (Eigen::VectorXd const& x) const = 0;

        // One step of the algorithm
        Eigen::VectorXd step (Eigen::VectorXd const& x) const;

        // Backtracking line search
        double lineSearch (Eigen::VectorXd const& x,
                           double alpha = 0.186,
                           double beta = 0.45) const;
};

#endif

