/*! @file point.cpp
 *
 *  Copyright (C) Qiuchen Qian, 2021
 *  Imperial College London
 */

#include "point.h"

std::vector<float> Point::calcXYDist(const std::vector<Point>& p_list)
{
	std::vector<float> distances;
	for (const Point& this_p : p_list) distances.push_back(this->calcXYDist(this_p));
	return distances;
}