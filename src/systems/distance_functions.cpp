/**
 * @file distance_functions.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Original work Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * Modified work Copyright 2017 Oleg Y. Sinyavskiy
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Original authors: Zakary Littlefield, Kostas Bekris
 * Modifications by: Oleg Y. Sinyavskiy
 *
 */

#include <assert.h>
#include "systems/distance_functions.h"
#include "systems/two_link_acrobot.hpp"


double two_link_acrobot_distance::distance(const double* p0, const double* p1, unsigned int state_dimensions) const
{
    return two_link_acrobot_t::distance(p0, p1, state_dimensions);
}
