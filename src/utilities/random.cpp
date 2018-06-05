/**
 * @file random.cpp
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

#include "utilities/random.hpp"

RandomGenerator::RandomGenerator(int seed)
    :engine(seed)
{
}

double RandomGenerator::uniform_random(double min, double max)
{
    return (((double)this->engine() / (double)std::default_random_engine::max()) * (max - min)) + min;
}

int RandomGenerator::uniform_int_random(int min, int max)
{
    return (this->engine() % (max + 1 - min)) + min;
}
