#ifndef _SEARCH_H_
#define _SEARCH_H_

#include <functional>

/**
 * @brief Given a unimodal function defined on the interval,
 *        find a value on the interval to minimize the function.
 *        Reference: https://en.wikipedia.org/wiki/Golden-section_search
 * @param func The target single-variable function to minimize.
 * @param lower_bound The lower bound of the interval.
 * @param upper_bound The upper bound of the interval.
 * @param tol The tolerance of error.
 * @return The value that minimize the function fun.
 */
double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol = 1e-6);

#endif
