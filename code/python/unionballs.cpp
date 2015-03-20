#include <CGAL/Simple_cartesian.h>

#include "CGAL_AD.hpp"
#include "union_balls_2.hpp"
#include "union_balls_2_utils.hpp"

// AD
typedef CGAL::Simple_cartesian<AD> Kernel;
typedef Kernel::FT FT;
typedef CGAL::Point_2<Kernel> Point;

#include <Eigen/Dense>

typedef Eigen::Matrix<FT, Eigen::Dynamic, 1> VectorXd;
typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> DynamicStride;

typedef std::function<FT(const VectorXd &x)> Function;

typedef UnionBalls<Point, FT, VectorXd,
                   Accumulator<FT>,
                   AngularSectorPerimeterAccumulator<FT> > PerimeterUnion;
typedef UnionBalls<Point, FT, VectorXd,
                   TriangleAreaAccumulator<FT>,
                   AngularSectorAreaAccumulator<FT> > VolumeUnion;

#include <boost/python.hpp>
#include <boost/numpy.hpp>
namespace p = boost::python;
namespace np = boost::numpy;

// Converts a numpy array to an Eigen Vector
template<class FT>
Eigen::Map<Eigen::Matrix<FT,-1,1>, Eigen::Unaligned,  DynamicStride>
python_to_vector(const np::ndarray &array)
{
    if (array.get_dtype() != np::dtype::get_builtin<FT>())
    {
        PyErr_SetString(PyExc_TypeError, "Incorrect array data type");
        p::throw_error_already_set();
    }
    if (array.get_nd() != 1)
    {
        PyErr_SetString(PyExc_TypeError, "Incorrect number of dimensions");
        p::throw_error_already_set();
    }
    auto s0 = array.strides(0) / sizeof(FT);
    return Eigen::Map<Eigen::Matrix<FT,-1,1>,
           Eigen::Unaligned,  DynamicStride>
               (reinterpret_cast<FT*>(array.get_data()),
                array.shape(0), DynamicStride(0,s0));
}

// Compute the perimeter of the boundary of a union of balls in 2D.
// Input: 2 * N vector of points
double perimeter_union_balls_2 (const np::ndarray &points,
                                double radius) {
    PerimeterUnion per(radius);

    // Convert numpy array to Eigen vector
    Eigen::VectorXd P = python_to_vector<double>(points);

    // Convert Eigen vector to Eigen AD vector
    size_t N = P.rows();

    VectorXd v(N);
    for (size_t i = 0; i < N; ++i) {
        v(i) = AD(P(i), N, i);
    }

    // Compute perimeter
    return per(v).value();
}

// Compute the gradient of the perimeter of the boundary of a union of balls in 2D.
// Input: 2 * N vector of points
np::ndarray gradient_perimeter_union_balls_2 (const np::ndarray &points,
                                              double radius) {
    PerimeterUnion per(radius);

    // Convert numpy array to Eigen vector
    Eigen::VectorXd P = python_to_vector<double>(points);

    // Convert Eigen vector to Eigen AD vector
    size_t N = P.rows();

    VectorXd v(N);
    for (size_t i = 0; i < N; ++i) {
        v(i) = AD(P(i), N, i);
    }

    per(v);
    Eigen::VectorXd grad = per.grad();

    // Convert Eigen vector to numpy array
    auto pgrad = np::zeros(p::make_tuple(grad.rows()),
                           np::dtype::get_builtin<double>());
    for (size_t i = 0; i < grad.rows(); ++i) {
        pgrad[i] = grad[i];
    }

    return pgrad;
}

// Compute the volume of a union of balls in 2D.
// Input: 2 * N vector of points
double volume_union_balls_2 (const np::ndarray &points,
                             double radius) {
    VolumeUnion vol(radius);

    // Convert numpy array to Eigen vector
    auto P = python_to_vector<double>(points);

    // Convert Eigen vector to Eigen AD vector
    size_t N = P.rows();

    VectorXd v(N);
    for (size_t i = 0; i < N; ++i) {
        v(i) = AD(P(i), N, i);
    }

    // Compute perimeter
    return vol(v).value();
}

// Compute the gradient of the volume of a union of balls in 2D.
// Input: 2 * N vector of points
np::ndarray gradient_volume_union_balls_2 (const np::ndarray &points,
                                           double radius) {
    VolumeUnion vol(radius);

    // Convert numpy array to Eigen vector
    Eigen::VectorXd P = python_to_vector<double>(points);

    // Convert Eigen vector to Eigen AD vector
    size_t N = P.rows();

    VectorXd v(N);
    for (size_t i = 0; i < N; ++i) {
        v(i) = AD(P(i), N, i);
    }

    vol(v);
    Eigen::VectorXd grad = vol.grad();

    // Convert Eigen vector to numpy array
    auto pgrad = np::zeros(p::make_tuple(grad.rows()),
                           np::dtype::get_builtin<double>());
    for (size_t i = 0; i < grad.rows(); ++i) {
        pgrad[i] = grad[i];
    }

    return pgrad;
}

BOOST_PYTHON_MODULE(unionballs) {
    np::initialize();

    p::def("perimeter_boundary", &perimeter_union_balls_2);
    p::def("gradient_perimeter_boundary", &gradient_perimeter_union_balls_2);

    p::def("volume", &volume_union_balls_2);
    p::def("gradient_volume", &gradient_volume_union_balls_2);
}

