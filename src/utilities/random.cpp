/**
 * @file random.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/random.hpp"

#include <stdlib.h>

void init_random(int seed)
{
    srand(seed);
}

double uniform_random()
{
    return rand() / RAND_MAX;
}

double uniform_random(double min, double max)
{
    return (((double)rand() / (double)RAND_MAX) * (max - min)) + min;
}

int uniform_int_random(int min, int max)
{
    return (rand() % (max + 1 - min)) + min;
}
