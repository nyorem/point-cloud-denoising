#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include "AD.hpp"

// Computes the gradient using the finite difference scheme:
// (f(x + eps) - f(x)) / eps
template <typename Function, typename Vector>
Vector grad_finite_1 (Function const& f, Vector const& x, const double eps = 1e-5) {
    Vector grad(x.rows());

    for (int i = 0; i < x.rows(); ++i) {
        Vector xx = x;
        xx[i] += eps;

        grad[i] = (f(xx) - f(x)) / eps;
    }

    return grad;
}

// Wrapper over grad_finite_1
template <typename Function, typename Vector>
struct GradFinite1Eval {
    Vector operator() (Function const& f, Vector const& x) const {
        return grad_finite_1(f, x);
    }
};

// Computes the gradient using the finite difference scheme:
// (f(x + eps) - f(x - eps)) / (2 * eps)
template <typename Function, typename Vector>
Vector grad_finite_2 (Function const& f, Vector const& x, const double eps = 1e-5) {
    Vector grad(x.rows());

    for (int i = 0; i < x.rows(); ++i) {
        Vector xx = x;
        xx[i] += eps;

        Vector xy = x;
        xy[i] -= eps;

        grad[i] = (f(xx) - f(xy)) / (2 * eps);
    }

    return grad;
}

// Wrapper over grad_finite_2
template <typename Function, typename Vector>
struct GradFinite2Eval {
    Vector operator() (Function const& f, Vector const& x) const {
        return grad_finite_2(f, x);
    }
};

// Computes the gradient using AD.
template <typename FunctionAD, typename VectorAD>
VectorAD grad_ad (FunctionAD const& fad, VectorAD const& x) {
    VectorAD grad(x.rows());

    for (int i = 0; i < x.rows(); ++i) {
        grad[i] = AD(fad(x).derivatives().coeffRef(i), x.rows(), i);
    }

    return grad;
}

// Wrapper over grad_ad
template <typename FunctionAD, typename VectorAD>
struct GradAdEval {
    VectorAD operator() (FunctionAD const& fad, VectorAD const& x) const {
        return grad_ad(fad, x);
    }
};

// One step of the gradient descent algorithm using a constant step
template <typename EvalGradient, typename Function, typename Vector>
Vector step_gradient_descent (EvalGradient const& eval_grad,
                              Function const& f, Vector const& x0, double step) {
    Vector grad = eval_grad(f, x0);

    return x0 - step * grad;
}

// Converts a vector to a list of points
template <typename Point, typename Vector, typename OutputIterator>
void vectorToPointCloud (Vector const& v,
                         OutputIterator out) {
    int N = v.rows() / 2;
    for (int i = 0; i < N; ++i) {
        *out++ = Point(v(2 * i), v(2 * i + 1));
    }
}

// Converts a point cloud to a vector
template <typename Vector, typename InputIterator>
Vector pointCloudToVector (InputIterator begin,
                           InputIterator beyond) {
    int N = 0;
    for (InputIterator it = begin; it != beyond; ++it)
        ++N;

    Vector vec(2 * N);
    int i = 0;
    for (InputIterator it = begin; it != beyond; ++it) {
        vec(2 * i) = AD(it->x(), 2 * N, 2 * i);
        vec(2 * i + 1) = AD(it->y(), 2 * N, 2 * i + 1);
        ++i;
    }

    return vec;
}

#endif

