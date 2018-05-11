/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "fgeval.hpp"
#include "globalConstants.hpp"

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using CppAD::AD;
using namespace std;

/**************************************************************************************************
 *  @brief constructor for class MPC
 *************************************************************************************************/
MPC::MPC()
{
}

/**************************************************************************************************
 *  @brief destructor for class MPC
 *************************************************************************************************/
MPC::~MPC()
{
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double)Dvector;

    double x = state[STATE_INDEX_X];
    double y = state[STATE_INDEX_Y];
    double psi = state[STATE_INDEX_PSI];
    double v = state[STATE_INDEX_V];
    double cte = state[STATE_INDEX_CTE];
    double epsi = state[STATE_INDEX_EPSI];

    //Set the number of model variables (includes both states and inputs).
    size_t n_vars = N * TOTAL_STATE_INDEXES + (N - 1) * 2;
    size_t n_constraints = N * TOTAL_STATE_INDEXES;

    Dvector vars(n_vars);
    for (uint32_t i = 0; i < n_vars; ++i)
    {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits to the max negative and positive values.
    for (uint32_t i = 0; i < delta_start; ++i)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25, degrees (values in radians).
    for (uint32_t i = delta_start; i < a_start; ++i)
    {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    for (uint32_t i = a_start; i < n_vars; ++i)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints, should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (uint32_t i = 0; i < n_constraints; ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // options for IPOPT solver
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound,
            constraints_lowerbound, constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    vector<double> result;

    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);

    for (uint32_t i = 0; i < N; ++i)
    {
        result.push_back(solution.x[x_start + i]);
        result.push_back(solution.x[y_start + i]);
    }

    return result;
}
