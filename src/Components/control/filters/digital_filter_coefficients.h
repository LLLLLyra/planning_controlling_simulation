#ifndef _DIGITAL_FILTER_COEFFICIENT_H_
#define _DIGITAL_FILTER_COEFFICIENT_H_

#include <vector>

#include "glog/logging.h"

/**
 * @brief Get low-pass coefficients for digital filter.
 * @param ts Time interval between signals.
 * @param cutoff_freq Cutoff of frequency to filter high-frequency signals out.
 * @param denominators Denominator coefficients for digital filter.
 * @param numerators Numerator coefficients for digital filter.
 */
void LpfCoefficients(const double ts, const double cutoff_freq,
                     std::vector<double> *denominators,
                     std::vector<double> *numerators);

/**
 * @brief Get first order low-pass coefficients for ZOH digital filter.
 * @param ts sampling time.
 * @param settling time: time required for an output to reach and remain within
 *                       a given error band
 * @param dead time: time delay
 * @param denominators Denominator coefficients for digital filter.
 * @param numerators Numerator coefficients for digital filter.
 */
void LpFirstOrderCoefficients(const double ts, const double settling_time,
                              const double dead_time,
                              std::vector<double> *denominators,
                              std::vector<double> *numerators);

#endif
