//
// Created by Oleg Sinyavskiy on 11/28/16.
//

#include <assert.h>
#include "systems/distance_functions.h"
#include "systems/two_link_acrobot.hpp"


double two_link_acrobot_distance::distance(const double* p0, const double* p1, unsigned int state_dimensions) const
{
    return two_link_acrobot_t::distance(p0, p1, state_dimensions);
}
