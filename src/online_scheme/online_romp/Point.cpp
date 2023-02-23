#include "Point.h"

using std::abs;

bool Point::is_same_pt(const Point& p) const
{
	if (abs(_x - p.get_x()) <= eps && abs(_y - p.get_y()) <= eps 
		&& abs(_z - p.get_z()) <= eps) {
		return true;
	}
	return false;
}

size_t Point::closer_pt(const Point& p1, const Point& p2) const
{
	double d_p1 = (p1 - *this).get_square();
	double d_p2 = (p2 - *this).get_square();
	return d_p1 < d_p2 ? 0 : 1;
}