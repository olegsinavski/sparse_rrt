/**
 * @file condition_check.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_CONDITION_CHECK
#define SPARSE_CONDITION_CHECK

#include <string>

#include "utilities/timer.hpp"

/**
 * @brief A class which checks if a condition is met.
 * @details A class which checks if a condition is met.
 * 
 */
class condition_check_t
{
public:
	condition_check_t(std::string type, double check );

	/**
	 * @brief Resets the internal counts and timers.
	 * @details Resets the internal counts and timers.
	 */
	void reset();

	/**
	 * @brief Check if condition is satisfied.
	 * @details Check if condition is satisfied.
	 * @return True if satisfied, false if not.
	 */
	bool check();

	/**
	 * @brief Get the time on the timer.
	 * @details Get the time on the timer.
	 * 
	 * @return The current time.
	 */
	double time()
	{
		return timer.measure();
	}

	/**
	 * @brief Get the current iterations.
	 * @details Get the current iterations.
	 * 
	 * @return The current iterations.
	 */
	unsigned iterations()
	{
		return iteration_counter;
	}

protected:

	/**
	 * @brief The timer used for checking times.
	 */
	sys_timer_t timer;

	/**
	 * @brief A counter for iterations.
	 */
	unsigned iteration_counter;

	/**
	 * @brief The maximum time or iterations before being satisfied.
	 */
	double condition_check;

	/**
	 * @brief Which type of condition to check. 0 for iterations, 1 for time.
	 */
	unsigned condition_type;

};

#endif