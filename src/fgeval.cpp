/*
 * fgeval.cpp
 *
 *  Created on: May 9, 2018
 *      Author: rohitbahl
 */
/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "fgeval.hpp"
#include "globalConstants.hpp"

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using CppAD::AD;
using namespace std;

/**************************************************************************************************
 *  CLASS DEFINITIONS
 *************************************************************************************************/
void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
    fg[costIdx] = 0;

    for (uint32_t i = 0; i < N; ++i)
    {
        fg[costIdx] += 3000 * pow(vars[cte_start + i], 2);
        fg[costIdx] += 3000 * pow(vars[epsi_start + i], 2);
        fg[costIdx] += pow(vars[v_start + i] - ref_v, 2);
    }

    for (uint32_t i = 0; i < N - 1; ++i)
    {
        fg[costIdx] += 30 * pow(vars[delta_start + i], 2);
        fg[costIdx] += 30 * pow(vars[a_start + i], 2);
    }

    for (uint32_t i = 0; i < N - 2; ++i)
    {
        fg[costIdx] += 150 * pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
        fg[costIdx] += 20 * pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    fg[costOffset + x_start] = vars[x_start];
    fg[costOffset + y_start] = vars[y_start];
    fg[costOffset + psi_start] = vars[psi_start];
    fg[costOffset + v_start] = vars[v_start];
    fg[costOffset + cte_start] = vars[cte_start];
    fg[costOffset + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = costOffset; t < N; ++t)
    {
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        AD<double> a = vars[a_start + t - 1];
        AD<double> delta = vars[delta_start + t - 1];

        AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2)
                + coeffs[3] * pow(x0, 3);
        AD<double> psides0 = atan(
                coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

        fg[costOffset + x_start + t] = x1 - (x0 + v0 * cos(psi0) * dt);
        fg[costOffset + y_start + t] = y1 - (y0 + v0 * sin(psi0) * dt);
        fg[costOffset + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta * dt);
        fg[costOffset + v_start + t] = v1 - (v0 + a * dt);
        fg[costOffset + cte_start + t] = cte1 - ((f0 - y0) + (v0 * sin(epsi0) * dt));
        fg[costOffset + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta * dt);
    }
}


