#ifndef _ADD_NOISE_H_
#define _ADD_NOISE_H_

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>

static boost::mt19937 g_rng;

// Add gaussian noise
struct Add_gaussian_noise {
    Add_gaussian_noise (double squaredVariance) :
        m_var(squaredVariance),
        m_distrib(0.0, std::sqrt(squaredVariance)),
        m_gen(g_rng, m_distrib) {
        g_rng.seed(static_cast<unsigned int>(std::time(0)));
    }

    template <typename Point>
    void operator() (Point &p) {
        typedef typename CGAL::Kernel_traits<Point>::Kernel Kernel;
        typedef typename Kernel::Vector_3 Vector_3;

        Vector_3 v(m_gen(), m_gen(), m_gen());
        p = p + v;
    }

    double m_var;
    boost::normal_distribution<float> m_distrib;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > m_gen;
};

#endif

