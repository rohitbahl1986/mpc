/*
 * globalConstants.cpp
 *
 *  Created on: May 9, 2018
 *      Author: rohitbahl
 */
/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "globalConstants.hpp"

/**************************************************************************************************
 *  CONSTANTS
 *************************************************************************************************/
const size_t N = 10;
const double dt = 0.1;
const double Lf = 2.67;
const double ref_v = 90;
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;
