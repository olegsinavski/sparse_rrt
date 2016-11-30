//
// Created by Oleg Sinyavskiy on 11/28/16.
//


#include "systems/distance_functions.h"
#include "systems/two_link_acrobot.hpp"


std::function<double(double*, double*)> euclidian_distance(const std::vector<bool> is_circular_topology) {

    return [is_circular_topology] (double* point1, double* point2) {
        double result = 0;
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


std::function<double(double*, double*)> two_link_acrobot_distance()
{
    return two_link_acrobot_t::distance;
}
