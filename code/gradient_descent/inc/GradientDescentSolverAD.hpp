#ifndef _GRADIENTDESCENTSOLVERAD_HPP_
#define _GRADIENTDESCENTSOLVERAD_HPP_

template <typename AD, typename FunctionAD, typename VectorAD, typename FTAD>
class GradientDescentSolverAD {
    public:
        GradientDescentSolverAD (FunctionAD const& fad) : fad(fad) {
        }

        VectorAD solve (VectorAD const& x0,
                        double eps = 0.01) const {
            VectorAD x(x0), gx(x0.rows());
            gx = gradient(x);
            double tprev = 1;

            while (gx.norm() > eps) {
                VectorAD dir = descentDirection(x);
                double t = lineSearch(x);
                // TODO: Why?
                if (tprev == t) {
                    break;
                }
                tprev = t;
                x += t * dir;
                gx = gradient(x);
            }

            return x;
        }

    protected:
        FunctionAD fad;

        // Gives the descent direction at x
        VectorAD descentDirection (VectorAD const& x) const {
            return -gradient(x);
        }

        // Compute the gradient using AD
        VectorAD gradient (VectorAD const& x) const {
            FTAD eval = fad(x);
            VectorAD grad(x.rows());

            for (int i = 0; i < x.rows(); ++i) {
                grad[i] = AD(eval.derivatives().coeffRef(i), x.rows(), i);
            }

            return grad;
        }

        // Backtracking line search
        double lineSearch (VectorAD const& x,
                           double alpha = 0.186,
                           double beta = 0.45) const {
            double t = 1;

            VectorAD dir = descentDirection(x);

            FTAD lhs = fad(x + t * dir),
                 rhs = fad(x) + alpha * t * dir.dot(dir);

            while (lhs > rhs) {
                dir = descentDirection(x);

                lhs = fad(x + t * dir);
                rhs = fad(x) + alpha * t * dir.dot(dir);

                t *= beta;
            }

            return t;
        }
};

// Convert a VectorAD to a normal Eigen::VectorXd
template <typename VectorAD>
Eigen::VectorXd toValue (VectorAD const& x) {
    Eigen::VectorXd res(x.rows());

    for (int i = 0; i < x.rows(); ++i) {
        res[i] = x[i].value();
    }

    return res;
}

#endif

