#if !defined(MPC_H)
#define MPC_H

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <stdint.h>

/**************************************************************************************************
 *  CLASS DEFINITIONS
 *************************************************************************************************/
class MPC
{
public:
    MPC();

    virtual ~MPC();

    enum
    {
        STATE_INDEX_X,
        STATE_INDEX_Y,
        STATE_INDEX_PSI,
        STATE_INDEX_V,
        STATE_INDEX_CTE,
        STATE_INDEX_EPSI,
        TOTAL_STATE_INDEXES
    };
    // Solve the model given an initial state & polynomial coefficients, Return the first actuatotion
    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
