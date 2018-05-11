/*
 * fgeval.hpp
 *
 *  Created on: May 9, 2018
 *      Author: rohitbahl
 */

#if !defined(SRC_FGEVAL_HPP_)
#define SRC_FGEVAL_HPP_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <stdint.h>

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class FG_eval
{

public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs)
    {
        this->coeffs = coeffs;
    }

    typedef CPPAD_TESTVECTOR(CppAD::AD<double>)ADvector;
    void operator()(ADvector& fg, const ADvector& vars);

private:
    const uint32_t costIdx = 0;
    const uint32_t costOffset = costIdx + 1;
};

#endif // SRC_FGEVAL_HPP_
