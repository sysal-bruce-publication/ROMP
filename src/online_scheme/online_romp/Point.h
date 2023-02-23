/*****************************************************************//**
 * \file   Point.h
 * \brief
 *
 * \author Bruce
 * \date   October 2022
 *********************************************************************/
#pragma once
#include <cmath>
#include <utility>
#include <map>
#include <cstddef>		// std::size_t

#ifndef POINT_H
#define POINT_H

using std::size_t;

constexpr double eps = 1e-6;
typedef std::pair<size_t, size_t> IdxPair;

class Point
{
public:
	Point() {}
	Point(const Point& p) { _x = p.get_x(); _y = p.get_y(); _z = p.get_z(); }
	Point(double x, double y, double z) : _x(x), _y(y), _z(z) {}
	~Point() {}

	double get_x() const { return _x; }
	double get_y() const { return _y; }
	double get_z() const { return _z; }
	double get_square() const { return pow(_x, 2) + pow(_y, 2) + pow(_z, 2); }
	double get_xy_square() const { return pow(_x, 2) + pow(_y, 2); }
	double get_xy_norm2() const { return sqrt(pow(_x, 2) + pow(_y, 2)); }
	double get_norm2() const { return sqrt(pow(_x, 2) + pow(_y, 2) + pow(_z, 2)); }
	double get_diff_z(const Point& p) const { return std::abs(_z - p.get_z()); }
	bool is_same_pt(const Point& p) const;
	bool is_init_pt() const { return is_same_pt(Point()); }

	void set_x(double x) { _x = x; }
	void set_y(double y) { _y = y; }
	void set_z(double z) { _z = z; }
	void set_pt(const Point& p) { _x = p.get_x(); _y = p.get_y(); _z = p.get_z(); }
	void set_pt(double x, double y, double z) { _x = x; _y = y; _z = z; }

	size_t closer_pt(const Point& p1, const Point& p2) const;
	Point operator+(const Point& p) const
	{
		return Point(_x + p.get_x(), _y + p.get_y(), _z + p.get_z());
	}
	Point operator-(const Point& p) const
	{
		return Point(_x - p.get_x(), _y - p.get_y(), _z - p.get_z());
	}
	Point operator*(double val) const { return Point(_x * val, _y * val, _z * val); }
	/**
	 * Dot product (i.e. Hadamard product) of points.
	 */
	double operator*(const Point& p) const 
	{ 
		return _x * p.get_x() + _y * p.get_y() + _z * p.get_z(); 
	}
	Point operator/(double val) const 
	{ 
		return Point(_x / val, _y / val, _z / val); 
	}

private:
	double _x = 0;
	double _y = 0;
	double _z = 0;
};

#endif // !POINT_H

