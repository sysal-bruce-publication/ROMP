/**
* @file wind.h
* Implementation of wind vectors.
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

#ifndef WIND_H
#define WIND_H

namespace fs = std::filesystem;

class WindVec
{
public:
	WindVec() {};
	WindVec(float time, float x_comp, float y_comp, float z_comp) :
		_time(time), _x_comp(x_comp), _y_comp(y_comp), _z_comp(z_comp) {};
	~WindVec() {};

	float getT() const { return _time; }
	float getX() const { return _x_comp; }
	float getY() const { return _y_comp; }
	float getZ() const { return _z_comp; }

protected:
	float _time = 0;
	float _x_comp = 0;
	float _y_comp = 0;
	float _z_comp = 0;
};

class Grid : protected WindVec
{
public:
	Point start = Point();		/* Wind data starting coordinate [m] */

	int dx = 0;					/* The distance between each point in x-axis [m] */
	int dy = 0;					/* The distance between each point in x-axis [m] */
	int dz = 0;					/* The distance between each point in x-axis [m] */
	int dt = 0;					/* The duartion between each time stamp [s] */

	int num_xs = 0;				/* Number of points in x-axis */
	int num_ys = 0;				/* Number of points in y-axis */
	int num_zs = 0;				/* Number of points in z-axis */
	int num_ts = 0;				/* Number of time stamps */

	Grid() {};
	Grid(int ox_, int oy_, int oz_,
		int num_xs_, int num_ys_, int num_zs_,
		int dx_, int dy_, int dz_) :
		start(Point(static_cast<float>(ox_), static_cast<float>(oy_), static_cast<float>(oz_))),
		num_xs(num_xs_), num_ys(num_ys_), num_zs(num_zs_), dx(dx_), dy(dy_), dz(dz_) {};
	~Grid() {};

	//! In python data generation code, we have unit of [cm] and [ms].
	void readGridInfo(const fs::path& fname);
	void readWindData(float t_start, float t_end, const fs::path& fname);
	void getWindData(float time, const Point& target_pos, float& avg_x, float& avg_y, float& avg_z);

private:
	//! The variable to decide 
	float _start_time = 0;
	float _end_time = 0;

	//! Wind data vector, dimension [width(xs) * depth(ys) * height(zs)],
	//! to acess wind data at point (x, y, z), index should be [x + num_xs * (y + num_ys * z)]
	std::vector<WindVec> _wind_data;
};

#endif //! WIND_H