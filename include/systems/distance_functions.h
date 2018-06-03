//
// Created by Oleg Sinyavskiy on 11/28/16.
//

#ifndef SPARSERRT_DISTANCE_FUNCTIONS_H
#define SPARSERRT_DISTANCE_FUNCTIONS_H

#include <functional>
#include <vector>
#include <cmath>
#include <assert.h>


class distance_t
{
public:
    /**
     * @brief Finds the distance between two states.
     * @details Finds the distance between two states.
     *
     * @param point1 First point.
     * @param point2 Second point.
     * @param state_dimensions dimensionality of the state space
     *
     * @return The distance between point1 and point2.
     */
    virtual double distance(const double* point1, const double* point2, unsigned int state_dimensions) const = 0;
};


class euclidean_distance: public distance_t
{
public:
    euclidean_distance(const std::vector<bool>& is_circular_topology)
        : _is_circular_topology(is_circular_topology)
    { };


    double distance(const double* point1, const double* point2, unsigned int state_dimensions) const override {
        double result = 0;
        assert(state_dimensions == _is_circular_topology.size());
        for (unsigned int i=0; i<state_dimensions; ++i) {
            if (_is_circular_topology[i]) {
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

private:
    std::vector<bool> _is_circular_topology;
};


class two_link_acrobot_distance: public distance_t
{
public:
    double distance(const double* point1, const double* point2, unsigned int state_dimensions) const override;
};

#endif //SPARSERRT_DISTANCE_FUNCTIONS_H
