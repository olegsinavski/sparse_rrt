/**
 * @file timer.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/timer.hpp"

#include <cmath>
#include <cstddef>

sys_timer_t::sys_timer_t()
{
    reset();
    elapsed = 0;
}

sys_timer_t::~sys_timer_t()
{}

double sys_timer_t::get_time_in_secs()
{  
    gettimeofday( &finish, NULL ); 
    double s = finish.tv_sec;
    s += ( 0.000001 * finish.tv_usec );
    return s;
}

void sys_timer_t::reset()
{
    gettimeofday( &start, NULL );
}


double sys_timer_t::measure()
{
    gettimeofday( &finish, NULL ); 
    elapsed = (finish.tv_sec - start.tv_sec) + 0.000001 * (finish.tv_usec - start.tv_usec);
    return elapsed;
}


double sys_timer_t::measure_reset()
{
    measure();
    reset();
    return elapsed;
}

void sys_timer_t::add_delay_user_clock( double delay )
{
    int dl = (int)delay;
    int rest = (int)round( (double)(delay - dl) * 1000000.0);
    
    start.tv_sec  -= dl;
    start.tv_usec -= rest;
    if( start.tv_usec < 0 )
    {
	start.tv_sec -= 1;
	start.tv_usec = 1000000 + start.tv_usec;
    }
}