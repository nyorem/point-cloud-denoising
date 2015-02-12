#ifndef _GRADIENTDESCENTSOLVERAD_HPP_
#define _GRADIENTDESCENTSOLVERAD_HPP_

#include "DescentSolverAD.hpp"

template <typename AD, typename FunctionAD, typename VectorAD, typename FTAD>
class GradientDescentSolverAD : public DescentSolverAD<AD, FunctionAD, VectorAD, FTAD> {
    public:
        GradientDescentSolverAD (FunctionAD const& fad, int itermax = 100) : DescentSolverAD<AD, FunctionAD, VectorAD, FTAD>(fad, itermax) {
        }

    public:
        // Gives the descent direction at x
        VectorAD descentDirection (VectorAD const& x) const {
            return -this->gradient(x);
        }

};

#endif

