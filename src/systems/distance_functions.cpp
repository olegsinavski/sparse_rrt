//
// Created by Oleg Sinyavskiy on 11/28/16.
//

#include <assert.h>
#include "systems/distance_functions.h"
#include "systems/two_link_acrobot.hpp"


std::function<double(const double*, const double*, unsigned int)> euclidian_distance(const std::vector<bool> is_circular_topology) {

    return [is_circular_topology] (const double* point1, const double* point2, unsigned int state_dimensions) {
        double result = 0;
        assert(state_dimensions == is_circular_topology.size());
        for (int i=0; i<is_circular_topology.size(); ++i) {
            if (is_circular_topology[i]) {
                double val = fabs(point1[i]-point2[i]);
                if(val > M_PI)
                    val = 2*M_PI-val;
                result += val*val;
            } else {
                result += (point1[i]-point2[i]) * (point1[i]-point2[i]);
            }
        }
        return std::sqrt(result);
    };
}


std::function<double(const double*, const double*, unsigned int)> two_link_acrobot_distance()
{
    return two_link_acrobot_t::distance;
}
