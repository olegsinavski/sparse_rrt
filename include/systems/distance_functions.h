//
// Created by Oleg Sinyavskiy on 11/28/16.
//

#ifndef SPARSERRT_DISTANCE_FUNCTIONS_H
#define SPARSERRT_DISTANCE_FUNCTIONS_H

#include <functional>
#include <vector>
#include <cmath>

/**
 * @brief Finds the distance between two states.
 * @details Finds the distance between two states.
 *
 * @param point1 First point.
 * @param point2 Second point.
 *
 * @return The distance between point1 and point2.
 */


std::function<double(const double*, const double*)> euclidian_distance(const std::vector<bool> is_circular_topology);

std::function<double(const double*, const double*)> two_link_acrobot_distance();

#endif //SPARSERRT_DISTANCE_FUNCTIONS_H
