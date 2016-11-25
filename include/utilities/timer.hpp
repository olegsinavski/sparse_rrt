/**
 * @file timer.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */
 
#ifndef SPARSE_TIMER_HPP
#define SPARSE_TIMER_HPP

#include <sys/time.h>

/**
 * A class that can express current time as a single number and which can
 * also provide methods that measure elapsed time between consecutive calls.
 * 
 * @brief <b>A clock for measuring time using system time</b>
 * @authors Kostas Bekris
 * 
 */
class sys_timer_t
{

  protected:
    /** @brief When the timer was started */
    struct timeval start;
    /** @brief When the timer finished */
    struct timeval finish;
    /** @brief How much time has elapsed since start */
    double elapsed;

  public:
    sys_timer_t();
    virtual ~sys_timer_t();

    /**
     * Gets the time in seconds
     * 
     * @brief Gets the time in seconds
     * @return The time in seconds
     */
    double get_time_in_secs();

    /**
     * Resets the timer
     * 
     * @brief Resets the timer
     */
    void reset();

    /**
     * Measures the timer and returns the value in seconds
     * 
     * @brief Measures the timer
     * @return The elapsed time in seconds
     */
    double measure();

    /**
     * Performs measure and reset
     * 
     * @brief Calls measure and reset
     * @return The elapsed time in seconds
     */
    double measure_reset();

    /**
     * Adds a delay to the clock
     * 
     * @brief Adds a delay to the clock
     * @param delay Determines how much delay is added
     */
    void add_delay_user_clock(double delay);
};

#endif