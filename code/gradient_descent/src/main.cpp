#include <iostream>
#include <functional>

#include "GradientDescentSolver.hpp"
#include "GradientDescentSolverAD.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "CGAL_AD.hpp"
#include "NewtonSolver.hpp"

#include "volume_union_balls_2.hpp"
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point;
typedef Kernel::FT FT;

typedef CGAL::Simple_cartesian<AD> Kernel_ad;
typedef Kernel_ad::FT FT_ad;
typedef CGAL::Point_2<Kernel_ad> Point_ad;

typedef Eigen::Matrix<FT_ad, Eigen::Dynamic, 1> VectorXAD;
typedef std::function<FT_ad(const VectorXAD &x)> FunctionAD;
typedef GradientDescentSolverAD<AD, FunctionAD, VectorXAD, FT_ad> SolverAD;

template <typename PointAD, typename FTAD, typename VectorAD>
struct VolumeUnion {
    VolumeUnion (double radius) : m_radius(radius) {
    }

    FTAD operator() (VectorAD const& v) {
        return volume_union_balls_2_vector<FTAD, PointAD, VectorAD>(v, m_radius);
    }

    private:
        double m_radius;
};
typedef VolumeUnion<Point_ad, FT_ad, VectorXAD> VolumeUnionAD;

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

    std::cout << "Volume AD" << std::endl;
    std::vector<Point> points;
    points.push_back(Point(0, 0));
    points.push_back(Point(1.5, 0));
    /* points.push_back(Point(2, 0)); */
    /* points.push_back(Point(1, 1)); */
    int N = points.size();
    // Transform std::vector<Point> to VectorXAD
    VectorXAD points_vec(2 * N);
    for (int i = 0; i < N; ++i) {
        points_vec(2 * i) = AD(points[i].x(), 2 * N, 2 * i);
        points_vec(2 * i + 1) = AD(points[i].y(), 2 * N, 2 * i + 1);
    }
    VolumeUnionAD volume(1);
    std::cout << volume(points_vec) << std::endl;

    // TODO
    std::cout << "Gradient descent volume AD" << std::endl;
    SolverAD gsadv(volume);
    VectorXAD newX = gsadv.step(points_vec);
    std::cout << toValue(newX) << std::endl;

    return 0;
}

