/* @file obstacle.cpp
*
*  Copyright (C) Qiuchen Qian, 2021
*  Imperial College, London
*/

#include "obstacle.h"

using namespace std;

void Obstacle::readObsInfo(const fs::path& fname)
{
	ifstream file(fname);
	if (!file) throw runtime_error("Obstacle file cannot open.\n");

	int x0 = -1, y0 = -1, x1 = -1, y1 = -1, h = -1, cnt = 0;
	string line;
	while (getline(file, line)) {
		stringstream ss(line);
		for (int i; ss >> i;) {
			if (!cnt) x0 = i;
			else if (cnt == 1) y0 = i;
			else if (cnt == 2) x1 = i;
			else if (cnt == 3) y1 = i;
			else if (cnt == 4) h = i;

			if (ss.peek() == ',') ss.ignore();
			cnt++;
		}
	}
	file.close();

	this->_x0 = x0 / 100.f;
	this->_y0 = y0 / 100.f;
	this->_x1 = x1 / 100.f;
	this->_y1 = y1 / 100.f;
	this->_h = h / 100.f;
}

Point Obstacle::findAvoidPoint(const Point& a, const Point& b, float offset, int& intersect_case)
{
	Point first = Point(-1, -1, -1);
	unique_ptr<Point> obs0(new Point(this->_x0, this->_y0, 0));
	unique_ptr<Point> obs1(new Point(this->_x1, this->_y0, 0));
	unique_ptr<Point> obs2(new Point(this->_x1, this->_y1, 0));
	unique_ptr<Point> obs3(new Point(this->_x0, this->_y1, 0));

	int cnt = 0;
	if (this->_checkIntersect(a, b, *obs0, *obs1)) cnt += 1;
	if (this->_checkIntersect(a, b, *obs1, *obs2)) cnt += 2;
	if (this->_checkIntersect(a, b, *obs2, *obs3)) cnt += 4;
	if (this->_checkIntersect(a, b, *obs3, *obs0)) cnt += 8;

	float x_dir = 0, y_dir = 0;
	x_dir = b.getX() - a.getX();
	y_dir = b.getY() - a.getY();

	float off_val = offset / sqrtf(2);
	switch (cnt)
	{
	case 3:
		first.setX(this->_x1 + off_val);
		first.setY(this->_y0 - off_val);
		break;
	case 5:
		if (y_dir < 0) {
			if (b.getX() <= (this->_x1 + this->_x0) / 2) {
				first.setX(this->_x0 - off_val);
				first.setY(this->_y1 + off_val);
			}
			else { first.setX(this->_x1 + off_val); first.setY(this->_y1 + off_val); }
		}
		else {
			if (b.getX() <= (this->_x1 + this->_x0) / 2) {
				first.setX(this->_x0 - off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x1 + off_val); first.setY(this->_y0 - off_val); }
		}
		break;
	case 6:
		first.setX(this->_x1 + off_val);
		first.setY(this->_y1 + off_val);
		break;
	case 7:
		if (y_dir > 0) { first.setX(this->_x1 + off_val); first.setY(this->_y0 - off_val); }
		else { first.setX(this->_x1 + off_val); first.setY(this->_y1 + off_val); }
		break;
	case 9:
		first.setX(this->_x0 - off_val);
		first.setY(this->_y0 - off_val);
		break;
	case 10:
		if (x_dir < 0) {
			if (b.getY() <= (this->_y1 + this->_y0) / 2) {
				first.setX(this->_x1 + off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x1 + off_val); first.setY(this->_y1 + off_val); }
		}
		else {
			if (b.getY() <= (this->_y1 + this->_y0) / 2) {
				first.setX(this->_x0 - off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x0 - off_val); first.setY(this->_y1 + off_val); }
		}
		break;
	case 11:
		if (x_dir < 0) {
			if (b.getY() <= (this->_y1 + this->_y0) / 2) {
				first.setX(this->_x1 + off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x1 + off_val); first.setY(this->_y1 + off_val); }
		}
		else {
			if (b.getY() <= (this->_y1 + this->_y0) / 2) {
				first.setX(this->_x0 - off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x0 - off_val); first.setY(this->_y1 + off_val); }
		}
		break;
	case 12:
		first.setX(this->_x0 - off_val);
		first.setY(this->_y1 + off_val);
		break;
	case 13:
		if (y_dir > 0) {
			if (b.getX() <= (this->_x1 + this->_x0) / 2) {
				first.setX(this->_x0 - off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x1 + off_val); first.setY(this->_y0 - off_val); }
		}
		else {
			if (b.getX() <= (this->_x1 + this->_x0) / 2) {
				first.setX(this->_x0 - off_val);
				first.setY(this->_y1 + off_val);
			}
			else { first.setX(this->_x1 + off_val); first.setY(this->_y1 + off_val); }
		}
		break;
	case 14:
		if (x_dir > 0) {
			if (b.getY() <= (this->_y1 + this->_y0) / 2) {
				first.setX(this->_x0 - off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x0 - off_val); first.setY(this->_y1 + off_val); }
		}
		else {
			if (b.getY() <= (this->_y1 + this->_y0) / 2) {
				first.setX(this->_x1 + off_val);
				first.setY(this->_y0 - off_val);
			}
			else { first.setX(this->_x1 + off_val); first.setY(this->_y1 + off_val); }
		}
		break;
	default:
		break;
	}

	intersect_case = cnt;
	return first;
}