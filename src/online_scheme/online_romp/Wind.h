#pragma once
#include <vector>
#include "Point.h"

#ifndef WIND_H
#define WIND_H

struct WindVec
{
	WindVec(double time, double x, double y, double z) : 
		t(time), x_comp(x), y_comp(y), z_comp(z) {}
	double t = 0;
	double x_comp = 0;
	double y_comp = 0;
	double z_comp = 0;
};

class WindGrid
{
public:
	WindGrid() {}
	WindGrid(double ox, double oy, double oz, int num_xs, int num_ys,
		int num_zs, int num_ts, int dx, int dy, int dz, int dt,
		double start_time, double end_time) : _start(Point(ox, oy, oz)),
		_num_xs(num_xs), _num_ys(num_ys), _num_zs(num_zs), _dx(dx), _dy(dy), 
		_dz(dz), _dt(dt), _start_time(start_time), _end_time(end_time) {}
	~WindGrid() {}

	void set_wind_vec(const std::vector<WindVec>& wind_vec) { _wind = wind_vec; }
	void get_vector(double time, const Point& target, 
		double& avg_x, double& avg_y, double& avg_z) const;

private:
	Point _start       = Point();  // The z-axis is 0
	int _dx            = 0;
	int _dy            = 0;
	int _dz            = 0;
	int _dt            = 0;
	int _num_xs        = 0;
	int _num_ys        = 0;
	int _num_zs        = 0;
	int _num_ts        = 0;
	double _start_time = 0;
	double _end_time   = 0;
	std::vector<WindVec> _wind;
};


#endif // !WIND_H





