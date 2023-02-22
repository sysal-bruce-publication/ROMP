/**
* @file obstacle.h
* Implementation of obstacle.
*
* @author Qiuchen Qian <qiuchen.qian19@imperial.ac.uk>
*/

#pragma once
#include <stdexcept>
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>
#include "point.h"

namespace fs = std::filesystem;

#ifndef OBSTACLE_H
#define OBSTACLE_H

class Obstacle
{
public:
	Obstacle() {};
	Obstacle(float x_left, float x_right, float y_lower, float y_upper, float height) :
		_x0(x_left), _x1(x_right), _y0(y_lower), _y1(y_upper), _h(height) {};
	~Obstacle() {};

	// Get obstacle left boundary at X-axis.
	float getXL() const { return _x0; }
	// Get obstacle right boundary at X-axis.
	float getXR() const { return _x1; }
	// Get obstacle lower boundary at Y-axis.
	float getYL() const { return _y0; }
	// Get obstacle upper boundary at Y-axis.
	float getYU() const { return _y1; }
	// Get obstacle length (X-axis).
	float getL() const { return abs(_x1 - _x0); }
	// Get obstacle width (Y-axis).
	float getW() const { return abs(_y1 - _y0); }
	// Get obstacle height (Z-axis).
	float getH() const { return _h; }

	void readObsInfo(const fs::path& fname);
	Point findAvoidPoint(const Point& a, const Point& b, float offset, int& intersect_case);

private:
	float _x0 = 0;
	float _y0 = 0;
	float _x1 = 0;
	float _y1 = 0;
	float _h = 0;

	bool _ccw(const Point& a, const Point& b, const Point& c)
	{
		return (c.getY() - a.getY()) * (b.getX() - a.getX()) >
			(b.getY() - a.getY()) * (c.getX() - a.getX());
	}
	bool _checkIntersect(const Point& a, const Point& b, const Point& c, const Point& d)
	{
		return _ccw(a, c, d) != _ccw(b, c, d) && _ccw(a, b, c) != _ccw(a, b, d);
	}
};

#endif // !OBSTACLE_H

