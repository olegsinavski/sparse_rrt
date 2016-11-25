/**
 * @file condition_check.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/condition_check.hpp"
#include <iostream>
#include <stdlib.h>

condition_check_t::condition_check_t(std::string type, double check )
{
	condition_check = check;
	if(type=="iterations")
		condition_type = 0;
	else if(type=="time")
		condition_type = 1;
	else
	{
		std::cout<<"Condition type is invalid!"<<std::endl;
		exit(0);
	}
}
void condition_check_t::reset()
{
	timer.reset();
	iteration_counter = 0;
}

bool condition_check_t::check()
{
	++iteration_counter;
	if(condition_type==0)
	{
		if(iteration_counter>=condition_check)
			return true;
	}
	else if(condition_type==1)
	{
		if(timer.measure()>=condition_check)
		{
			return true;
		}
	}
	return false;
}	