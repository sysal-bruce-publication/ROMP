/* @file wind.cpp
*
*  Copyright (C) Qiuchen Qian, 2021
*  Imperial College, London
*/
#include "wind.h"

using namespace std;

void Grid::readGridInfo(const fs::path& fname)
{
	ifstream file(fname);
	if (!file) throw runtime_error("Grid file cannot open.\n"); 

	int x0 = -1, y0 = -1, cnt = 0;
	string line;
	while (getline(file, line)) {
		stringstream ss(line);
		for (int i; ss >> i;) {
			if (!cnt) x0 = i;
			else if (cnt == 1) y0 = i;
			else if (cnt == 2 && x0 != -1 && y0 != -1) {
				this->start = Point(x0 / 100.f, y0 / 100.f, i / 100.f);
			}
			else if (cnt == 3) this->_start_time = i / 1000.f;
			else if (cnt == 4) this->dx = i / 100;
			else if (cnt == 5) this->dy = i / 100;
			else if (cnt == 6) this->dz = i / 100;
			else if (cnt == 7) this->dt = i / 1000;
			else if (cnt == 8) this->num_xs = i;
			else if (cnt == 9) this->num_ys = i;
			else if (cnt == 10) this->num_zs = i;
			else if (cnt == 11) this->num_ts = i;

			if (ss.peek() == ',') ss.ignore();
			cnt++;
		}
	}
}

void Grid::readWindData(float t_start, float t_end, const fs::path& fname)
{
	if (t_end <= t_start) throw invalid_argument("end time before or equal start time\n");
	//! If have data, re-initialize the wind data vector
	if (this->_wind_data.size()) this->_wind_data.clear();
	int read_num_ts = static_cast<int>(floor((t_end - t_start) / this->dt)) + 1;

	ifstream file(fname);
	if (!file) throw runtime_error("Wind data file cannot open.\n");
	string line;
	getline(file, line);

	string t, uv_x, uv_y, uv_z;

	while (getline(file, t, ',')) {
		getline(file, uv_x, ',');
		getline(file, uv_y, ',');
		getline(file, uv_z);

		if (stof(t) / 1000 < t_start) continue;
		if (stof(t) / 1000 > t_end) break;
		this->_wind_data.push_back(WindVec(stof(t) / 1000, stof(uv_x) / 100,
			stof(uv_y) / 100, stof(uv_z) / 100));
	}
	file.close();

	int expected = this->num_xs * this->num_ys * this->num_zs * read_num_ts;
	if (this->_wind_data.size() != expected) {
		throw runtime_error("Read Error! Read data number: " + to_string(this->_wind_data.size())
			+ " Expected data number: " + to_string(expected));
	}

	this->_start_time = t_start;
	this->_end_time = t_end;
}

void Grid::getWindData(float time, const Point& target_pos, float& avg_x, float& avg_y, float& avg_z)
{
	if (this->_end_time == 0 || this->_start_time >= this->_end_time) {
		throw invalid_argument("start time: " + to_string(_start_time) + "end time: " + to_string(_end_time) + ".\n");
	}

	if (time > this->_end_time) {
		avg_x = 0;
		avg_y = 0;
		avg_z = 0;
		return;
	}

	int idx_t_int = static_cast<int>(floor(time / this->dt)) * this->num_xs * this->num_ys * this->num_zs;
	int idx_x_int = -1;
	int idx_y_int = -1;
	int idx_z_int = -1;

	if (!(fmod(target_pos.getX(), this->dx))) idx_x_int = static_cast<int>(target_pos.getX() / this->dx);
	if (!(fmod(target_pos.getY(), this->dy))) idx_y_int = static_cast<int>(target_pos.getY() / this->dy);
	if (!(fmod(target_pos.getZ(), this->dz))) idx_z_int = static_cast<int>(target_pos.getZ() / this->dz);

	//! PDV is not at any edge or exact point, then it needs to average data in cube.
	if (idx_x_int == -1 && idx_y_int == -1 && idx_z_int == -1) {
		int floor_x = static_cast<int>(floor(target_pos.getX() / this->dx));
		int floor_y = static_cast<int>(floor(target_pos.getY() / this->dy));
		int floor_z = static_cast<int>(floor(target_pos.getZ() / this->dz));
		int next_x = floor_x;
		if (floor_x < this->num_xs - 1) next_x++;
		int next_y = floor_y;
		if (floor_y < this->num_ys - 1) next_y++;
		int next_z = floor_z;
		if (floor_z < this->num_zs - 1) next_z++;

		int xyz_0 = idx_t_int + floor_x + this->num_xs * (floor_y + this->num_ys * floor_z);
		int xyz_1 = idx_t_int + next_x + this->num_xs * (floor_y + this->num_ys * floor_z);
		int xyz_2 = idx_t_int + floor_x + this->num_xs * (next_y + this->num_ys * floor_z);
		int xyz_3 = idx_t_int + floor_x + this->num_xs * (floor_y + this->num_ys * next_z);
		int xyz_4 = idx_t_int + next_x + this->num_xs * (next_y + this->num_ys * floor_z);
		int xyz_5 = idx_t_int + next_x + this->num_xs * (floor_y + this->num_ys * next_z);
		int xyz_6 = idx_t_int + floor_x + this->num_xs * (next_y + this->num_ys * next_z);
		int xyz_7 = idx_t_int + next_x + this->num_xs * (next_y + this->num_ys * next_z);

		avg_x = (this->_wind_data[xyz_0].getX() + this->_wind_data[xyz_1].getX() +
			this->_wind_data[xyz_2].getX() + this->_wind_data[xyz_3].getX() +
			this->_wind_data[xyz_4].getX() + this->_wind_data[xyz_5].getX() +
			this->_wind_data[xyz_6].getX() + this->_wind_data[xyz_7].getX()) / 8;
		avg_y = (this->_wind_data[xyz_0].getY() + this->_wind_data[xyz_1].getY() +
			this->_wind_data[xyz_2].getY() + this->_wind_data[xyz_3].getY() +
			this->_wind_data[xyz_4].getY() + this->_wind_data[xyz_5].getY() +
			this->_wind_data[xyz_6].getY() + this->_wind_data[xyz_7].getY()) / 8;
		avg_z = (this->_wind_data[xyz_0].getZ() + this->_wind_data[xyz_1].getZ() +
			this->_wind_data[xyz_2].getZ() + this->_wind_data[xyz_3].getZ() +
			this->_wind_data[xyz_4].getZ() + this->_wind_data[xyz_5].getZ() +
			this->_wind_data[xyz_6].getZ() + this->_wind_data[xyz_7].getZ()) / 8;
	}

	//! PDV is at the exact point
	else if (idx_x_int != -1 && idx_y_int != -1 && idx_z_int != -1) {
		int p_idx = idx_t_int + idx_x_int + this->num_xs * (idx_y_int + this->num_ys * idx_z_int);

		avg_x = this->_wind_data[p_idx].getX();
		avg_y = this->_wind_data[p_idx].getY();
		avg_z = this->_wind_data[p_idx].getZ();
	}

	//! Consider this scenario as averaging vectors in a line segment: p0--pdv-----p1
	//! PDV only needs to average data at x axis 
	else if (idx_x_int == -1 && idx_y_int != -1 && idx_z_int != -1) {
		int floor_x = static_cast<int>(floor(target_pos.getX() / this->dx));
		int next_x = floor_x;
		if (floor_x < this->num_xs - 1) next_x++;

		int x_0 = idx_t_int + floor_x + this->num_xs * (idx_y_int + this->num_ys * idx_z_int);
		int x_1 = idx_t_int + next_x + this->num_xs * (idx_y_int + this->num_ys * idx_z_int);

		avg_x = (this->_wind_data[x_0].getX() + this->_wind_data[x_1].getX()) / 2;
		avg_y = (this->_wind_data[x_0].getY() + this->_wind_data[x_1].getY()) / 2;
		avg_z = (this->_wind_data[x_0].getZ() + this->_wind_data[x_1].getZ()) / 2;
	}
	//! PDV only needs to average data at y axis
	else if (idx_x_int != -1 && idx_y_int == -1 && idx_z_int != -1) {
		int floor_y = static_cast<int>(floor(target_pos.getY() / this->dy));
		int next_y = floor_y;
		if (floor_y < this->num_ys - 1) next_y++;

		int y_0 = idx_t_int + idx_x_int + this->num_xs * (floor_y + this->num_ys * idx_z_int);
		int y_1 = idx_t_int + idx_x_int + this->num_xs * (next_y + this->num_ys * idx_z_int);

		avg_x = (this->_wind_data[y_0].getX() + this->_wind_data[y_1].getX()) / 2;
		avg_y = (this->_wind_data[y_0].getY() + this->_wind_data[y_1].getY()) / 2;
		avg_z = (this->_wind_data[y_0].getZ() + this->_wind_data[y_1].getZ()) / 2;
	}
	//! PDV only needs to average data at z axis
	else if (idx_x_int != -1 && idx_y_int != -1 && idx_z_int == -1) {
		int floor_z = static_cast<int>(floor(target_pos.getZ() / this->dz));
		int next_z = floor_z;
		if (floor_z < this->num_zs - 1) next_z++;

		int z_0 = idx_t_int + idx_x_int + this->num_xs * (idx_y_int + this->num_ys * floor_z);
		int z_1 = idx_t_int + idx_x_int + this->num_xs * (idx_y_int + this->num_ys * next_z);

		avg_x = (this->_wind_data[z_0].getX() + this->_wind_data[z_1].getX()) / 2;
		avg_y = (this->_wind_data[z_0].getY() + this->_wind_data[z_1].getY()) / 2;
		avg_z = (this->_wind_data[z_0].getZ() + this->_wind_data[z_1].getZ()) / 2;
	}

	//! Consider this scenario as averaging vectors in a rectangle: 
	//! p3--------p2
	//! ------------
	//! -----uav----
	//! ------------
	//! p0--------p1
	//! PDV only needs to average data at xy-plane
	else if (idx_x_int == -1 && idx_y_int == -1 && idx_z_int != -1) {
		int floor_x = static_cast<int>(floor(target_pos.getX() / this->dx));
		int floor_y = static_cast<int>(floor(target_pos.getY() / this->dy));
		int next_x = floor_x;
		if (floor_x < this->num_xs - 1) next_x++;
		int next_y = floor_y;
		if (floor_y < this->num_ys - 1) next_y++;

		int xy_0 = idx_t_int + floor_x + this->num_xs * (floor_y + this->num_ys * idx_z_int);
		int xy_1 = idx_t_int + next_x + this->num_xs * (floor_y + this->num_ys * idx_z_int);
		int xy_2 = idx_t_int + floor_x + this->num_xs * (next_y + this->num_ys * idx_z_int);
		int xy_3 = idx_t_int + next_x + this->num_xs * (next_y + this->num_ys * idx_z_int);

		avg_x = (this->_wind_data[xy_0].getX() + this->_wind_data[xy_1].getX() +
			this->_wind_data[xy_2].getX() + this->_wind_data[xy_3].getX()) / 4;
		avg_y = (this->_wind_data[xy_0].getY() + this->_wind_data[xy_1].getY() +
			this->_wind_data[xy_2].getY() + this->_wind_data[xy_3].getY()) / 4;
		avg_z = (this->_wind_data[xy_0].getZ() + this->_wind_data[xy_1].getZ() +
			this->_wind_data[xy_2].getZ() + this->_wind_data[xy_3].getZ()) / 4;
	}
	//! PDV only needs to average data at xz-plane
	else if (idx_x_int == -1 && idx_y_int != -1 && idx_z_int == -1) {
		int floor_x = static_cast<int>(floor(target_pos.getX() / this->dx));
		int floor_z = static_cast<int>(floor(target_pos.getZ() / this->dz));
		int next_x = floor_x;
		if (floor_x < this->num_xs - 1) next_x++;
		int next_z = floor_z;
		if (floor_z < this->num_zs - 1) next_z++;

		int xz_0 = idx_t_int + floor_x + this->num_xs * (idx_y_int + this->num_ys * floor_z);
		int xz_1 = idx_t_int + next_x + this->num_xs * (idx_y_int + this->num_ys * floor_z);
		int xz_2 = idx_t_int + floor_x + this->num_xs * (idx_y_int + this->num_ys * next_z);
		int xz_3 = idx_t_int + next_x + this->num_xs * (idx_y_int + this->num_ys * next_z);

		avg_x = (this->_wind_data[xz_0].getX() + this->_wind_data[xz_1].getX() +
			this->_wind_data[xz_2].getX() + this->_wind_data[xz_3].getX()) / 4;
		avg_y = (this->_wind_data[xz_0].getY() + this->_wind_data[xz_1].getY() +
			this->_wind_data[xz_2].getY() + this->_wind_data[xz_3].getY()) / 4;
		avg_z = (this->_wind_data[xz_0].getZ() + this->_wind_data[xz_1].getZ() +
			this->_wind_data[xz_2].getZ() + this->_wind_data[xz_3].getZ()) / 4;
	}
	//! PDV only needs to average data at yz-plane
	else if (idx_x_int != -1 && idx_y_int == -1 && idx_z_int == -1) {
		int floor_y = static_cast<int>(floor(target_pos.getY() / this->dy));
		int floor_z = static_cast<int>(floor(target_pos.getZ() / this->dz));
		int next_y = floor_y;
		if (floor_y < this->num_ys - 1) next_y++;
		int next_z = floor_z;
		if (floor_z < this->num_zs - 1) next_z++;

		int yz_0 = idx_t_int + idx_x_int + this->num_xs * (floor_y + this->num_ys * floor_z);
		int yz_1 = idx_t_int + idx_x_int + this->num_xs * (next_y + this->num_ys * floor_z);
		int yz_2 = idx_t_int + idx_x_int + this->num_xs * (floor_y + this->num_ys * next_z);
		int yz_3 = idx_t_int + idx_x_int + this->num_xs * (next_y + this->num_ys * next_z);

		avg_x = (this->_wind_data[yz_0].getX() + this->_wind_data[yz_1].getX() +
			this->_wind_data[yz_2].getX() + this->_wind_data[yz_3].getX()) / 4;
		avg_y = (this->_wind_data[yz_0].getY() + this->_wind_data[yz_1].getY() +
			this->_wind_data[yz_2].getY() + this->_wind_data[yz_3].getY()) / 4;
		avg_z = (this->_wind_data[yz_0].getZ() + this->_wind_data[yz_1].getZ() +
			this->_wind_data[yz_2].getZ() + this->_wind_data[yz_3].getZ()) / 4;
	}
	else throw runtime_error("Wind data corruption.\n");
}