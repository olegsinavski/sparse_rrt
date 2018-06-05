/**
 * @file random.hpp
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

#ifndef SPARSE_RANDOM_HPP
#define SPARSE_RANDOM_HPP

#include <random>

class RandomGenerator {
public:
    /**
    * Initializes the uniform random number generator with the given seed.
    *
    * @brief Initializes the uniform random number generator with the given seed.
    * @param seed The value to seed the RNG with.  Default is 10.
    */
    RandomGenerator(int seed);

    /**
     * Returns a random number from the uniform distribution within the
     * given range.
     *
     * @brief Returns a random number from the uniform distribution within the given range.
     * @param min The minimum random value to return.
     * @param max The maximum random value to return.
     *
     * @return A double precision random number in the given range.
     */
    double uniform_random(double min, double max);

    /**
     * Returns a random integer number from the uniform distribution within
     * the given range.
     *
     * @brief Returns a random integer number from the uniform distribution within the given range.
     * @param min The minimum random value to return.
     * @param max The maximum random value to return.
     *
     * @return An integer random number in the given range.
     */
    int uniform_int_random(int min, int max);

private:
    std::default_random_engine engine;
};

#endif
