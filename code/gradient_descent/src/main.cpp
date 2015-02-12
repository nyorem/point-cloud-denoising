#include <iostream>
#include <functional>

#include "GradientDescentSolver.hpp"
#include "GradientDescentSolverAD.hpp"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "CGAL_AD.hpp"
#include "NewtonSolver.hpp"

#include "volume_union_balls_2_ad.hpp"
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
    VectorXAD points_vec = pointCloudToVector<VectorXAD>(points.begin(), points.end());
    VolumeUnionAD volume(1);
    std::cout << volume(points_vec) << std::endl;

    // TODO
    std::cout << "Gradient descent volume AD" << std::endl;
    SolverAD gsadv(volume);
    std::cout << toValue(gsadv.gradient(points_vec)) << std::endl;
    VectorXAD newX = gsadv.step(points_vec);
    std::vector<Point_ad> points_bis;
    vectorToPointCloud<Point_ad>(newX, std::back_inserter(points_bis));
    for (size_t i = 0; i < points_bis.size(); ++i) {
        std::cout << points_bis[i] << std::endl;
    }

    return 0;
}

