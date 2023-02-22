/**
* @file point.h
* Implementation of point (3D coordinate).
*
* @author Qiuchen Qian <qiuchen.qian19@imperial.ac.uk>
*/

#pragma once
#include <vector>
#include <cmath>

#ifndef POINT_H
#define POINT_H

/*!
*  @class		Point
*  @headerfile	point.h "point.h"
*  @brief		Implementation of @a Point (class)
*
*  The @a Point  class includes the coordinate information, get and
*  set function, calculate distance between two points or a list of points,
*  and determine if two points are overlapped.
*
*  @author		Qiuchen Qian
*  @version		5
*  @date		2020
*  @copyright	MIT Public License
*/

class Point
{
//! @publicsection
public:
	Point() {};
	/* Construct object with user-defined arguments
	* @param x: X-axis coordinate.
	* @param y: Y-axis coordinate.
	* @param z: Z-axis coordinate.
	*/
	Point(float x, float y, float z) : _x(x), _y(y), _z(z) {};
	//! A default destructer.
	~Point() {};

	// Get X-axis coordinate.
	float getX() const { return _x; }
	// Get Y-axis coordinate.
	float getY() const { return _y; }
	// Get Z-axis coordinate.
	float getZ() const { return _z; }

	// Set X-axis coordinate.
	void setX(float x) { _x = x; }
	// Set Y-axis coordinate.
	void setY(float y) { _y = y; }
	// Set Z-axis coordinate.
	void setZ(float z) { _z = z; }
	// Set X-, Y- and Z-axis coordinate with three float variables.
	void setPtFloat(float x, float y, float z) { _x = x; _y = y; _z = z; }
	// Set X-, Y- and Z-axis coordinate with one Point object.
	void setPt(const Point& p) { _x = p._x; _y = p._y; _z = p._z; }

	/*! @brief		Calculate the vertical distance between one point and another z position
	*  @details	Related formula: Distance d = z1 - z2
	*  @param p		A @a Point object with @c T data type
	*  @return		The vertical distance between two points
	*/
	float calcZZDist(float z) { return _z - z; }
	/*! @brief		Calculate the vertical distance between UAV and the SN
	*  @details	Related formula: Distance d = z1 - z2
	*  @param p		A @a Point object with @c T data type
	*  @return		The vertical distance between two points
	*/
	float calcZZDist(const Point& p) { return std::abs(this->_z - p._z); }

	/*! @brief		Calculate the horizontal distance between one point and specified @a x and @a y
	*  @details	Related formula: Distance d = sqrt((x1 - x2)^2 + (y1 - y2)^2)
	*  @param x	A @c float variable to calculate distance
	*  @param y	A @c float variable to calculate distance
	*  @return		The @c float distance
	*/
	float calcXYDist(const float& x, const float& y) { return std::hypotf(this->_x - x, this->_y - y); }
	/*! @brief		Calculate the horizontal distance between one point and specified @a x and @a y
	*  @details	Related formula: Distance d = sqrt((x1 - x2)^2 + (y1 - y2)^2)
	*  @param x	A @c float variable to calculate distance
	*  @param y	A @c float variable to calculate distance
	*  @return		The @c float distance
	*/
	float calcXYDist(const Point& p) { return std::hypotf(this->_x - p._x, this->_y - p._y); }
	/*! @brief		Calculate the list of horizontal distances between one point (UAV) and a list of points
	*  @details	Related formula: Distance d = sqrt((x1 - x2)^2 + (y1 - y2)^2)
	*  @param p_list A @c vector of points with type @c T to be calculated
	*  @return		A vector stored distances
	*/
	std::vector<float> calcXYDist(const std::vector<Point>& p_list);

	/* Calculate the distance between two points. Related formula: Distance d = sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)
	*  @param p		A @a Point object with @c T data type
	*  @return		The distance between two points
	*/
	float calcXYZDist(const Point& p)
	{
		return std::hypot(std::hypot(this->_x - p._x, this->_y - p._y), this->_z - p._z);
	}

//! @prviate section
private:
	float _x = 0;			/* x coordinate */
	float _y = 0;			/* y coordinate */
	float _z = 0;			/* z coordinate */
};

#endif // POINT_H_

