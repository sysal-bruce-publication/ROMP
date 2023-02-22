#pragma once

#include <string>
#include <set>
#include <random>
#include <chrono>
#include <algorithm>
#include "sensor.h"

#ifndef ALGS_H
#define ALGS_H

class Algs
{
public:
	Algs() {};
	~Algs() {};

	/* Generate random integer ranging [0, limit].
	* @param limit: upper boundary value.
	*/ 
	int randInt(int limit);
	// Generate random float from uniform distribution ranging [0, 1]
	float randFloat();
};

#endif // !COMMON_ALGORITHMS_H


