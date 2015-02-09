#ifndef _DESCENTSOLVERAD_HPP_
#define _DESCENTSOLVERAD_HPP_

template <typename AD, typename FunctionAD, typename VectorAD, typename FTAD>
class DescentSolverAD {
    public:
        DescentSolverAD (FunctionAD const& fad, int itermax = 100) : fad(fad), itermax(itermax) {
        }

        VectorAD solve (VectorAD const& x0,
                        double eps = 0.01) const {
            VectorAD x(x0);
            int iter = 0;

            while (gradient(x).norm() > eps && iter < itermax) {
                x = step(x);
                ++iter;
                /* std::cout << toValue(x) << std::endl << std::endl; */
            }

            return x;
        }

    public:
        FunctionAD fad;
        int itermax;

        VectorAD step (VectorAD const& x) const {
            VectorAD dir = descentDirection(x);
            FTAD t = lineSearch(x);
            return x + t * dir;
        }

        // Gives the descent direction at x
        virtual VectorAD descentDirection (VectorAD const& x) const = 0;

        // Computes the gradient using AD
        VectorAD gradient (VectorAD const& x) const {
            FTAD eval = fad(x);
            VectorAD grad(x.rows());

            for (int i = 0; i < x.rows(); ++i) {
                grad[i] = AD(eval.derivatives().coeffRef(i), x.rows(), i);
            }

            return grad;
        }

        // Backtracking line search
        FTAD lineSearch (VectorAD const& x,
                         double alpha = 0.186,
                         double beta = 0.45) const {
            FTAD t = 1;

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

