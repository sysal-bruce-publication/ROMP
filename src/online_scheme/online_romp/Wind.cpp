#include "Wind.h"

void WindGrid::get_vector(double time, const Point& target,
	double& avg_x, double& avg_y, double& avg_z) const
{
	if (time < _start_time || time > _end_time) {
		// If time stamp is out of range, assume no wind data
		avg_x = avg_y = avg_z = 0;
		return;
	}
	
	int idx_t_int = static_cast<int>(floor((time - _start_time) / _dt)) 
		* _num_xs * _num_ys * _num_zs;
	int floor_x = static_cast<int>(floor(target.get_x() / _dx));
	int floor_y = static_cast<int>(floor(target.get_y() / _dy));
	int floor_z = static_cast<int>(floor(target.get_z() / _dz));
	int next_x = floor_x;
	if (floor_x < _num_xs - 1) next_x++;
	int next_y = floor_y;
	if (floor_y < _num_ys - 1) next_y++;
	int next_z = floor_z;
	if (floor_z < _num_zs - 1) next_z++;

	int xyz_0 = idx_t_int + floor_x + _num_xs * (floor_y + _num_ys * floor_z);
	int xyz_1 = idx_t_int + next_x + _num_xs * (floor_y + _num_ys * floor_z);
	int xyz_2 = idx_t_int + floor_x + _num_xs * (next_y + _num_ys * floor_z);
	int xyz_3 = idx_t_int + floor_x + _num_xs * (floor_y + _num_ys * next_z);
	int xyz_4 = idx_t_int + next_x + _num_xs * (next_y + _num_ys * floor_z);
	int xyz_5 = idx_t_int + next_x + _num_xs * (floor_y + _num_ys * next_z);
	int xyz_6 = idx_t_int + floor_x + _num_xs * (next_y + _num_ys * next_z);
	int xyz_7 = idx_t_int + next_x + _num_xs * (next_y + _num_ys * next_z);

	avg_x = (_wind[xyz_0].x_comp + _wind[xyz_1].x_comp +
		_wind[xyz_2].x_comp + _wind[xyz_3].x_comp +
		_wind[xyz_4].x_comp + _wind[xyz_5].x_comp +
		_wind[xyz_6].x_comp + _wind[xyz_7].x_comp) / 8;
	avg_y = (_wind[xyz_0].y_comp + _wind[xyz_1].y_comp +
		_wind[xyz_2].y_comp + _wind[xyz_3].y_comp +
		_wind[xyz_4].y_comp + _wind[xyz_5].y_comp +
		_wind[xyz_6].y_comp + _wind[xyz_7].y_comp) / 8;
	avg_z = (_wind[xyz_0].z_comp + _wind[xyz_1].z_comp +
		_wind[xyz_2].z_comp + _wind[xyz_3].z_comp +
		_wind[xyz_4].z_comp + _wind[xyz_5].z_comp +
		_wind[xyz_6].z_comp + _wind[xyz_7].z_comp) / 8;
	
	//int idx_x_int = -1;
	//int idx_y_int = -1;
	//int idx_z_int = -1;
	//if (!(fmod(target.get_x(), _dx))) idx_x_int = static_cast<int>(target.get_x() / _dx);
	//if (!(fmod(target.get_y(), _dy))) idx_y_int = static_cast<int>(target.get_y() / _dy);
	//if (!(fmod(target.get_z(), _dz))) idx_z_int = static_cast<int>(target.get_z() / _dz);

	//if (idx_x_int == -1 && idx_y_int == -1 && idx_z_int == -1) {

	//}
	//else if (idx_x_int != -1 && idx_y_int != -1 && idx_z_int != -1) {  
	//	// If UAV is at the exact point
	//	int p_idx = idx_t_int + idx_x_int + _num_xs * (idx_y_int + _num_ys * idx_z_int);

	//	avg_x = _wind[p_idx].x_comp;
	//	avg_y = _wind[p_idx].y_comp;
	//	avg_z = _wind[p_idx].z_comp;
	//}
	//else if (idx_x_int == -1 && idx_y_int != -1 && idx_z_int != -1) {
	//	// Consider this scenario as averaging vectors in a line segment: p0--UAV--p1
	//	// UAV only needs to average data at x axis 
	//	int floor_x = static_cast<int>(floor(target.get_x() / _dx));
	//	int next_x = floor_x;
	//	if (floor_x < _num_xs - 1) next_x++;

	//	int x_0 = idx_t_int + floor_x + _num_xs * (idx_y_int + _num_ys * idx_z_int);
	//	int x_1 = idx_t_int + next_x + _num_xs * (idx_y_int + _num_ys * idx_z_int);

	//	avg_x = (_wind[x_0].x_comp + _wind[x_1].x_comp) / 2;
	//	avg_y = (_wind[x_0].y_comp + _wind[x_1].y_comp) / 2;
	//	avg_z = (_wind[x_0].z_comp + _wind[x_1].z_comp) / 2;
	//}
	//else if (idx_x_int != -1 && idx_y_int == -1 && idx_z_int != -1) {
	//	// UAV only needs to average data at y axis
	//	int floor_y = static_cast<int>(floor(target.get_y() / _dy));
	//	int next_y = floor_y;
	//	if (floor_y < _num_ys - 1) next_y++;

	//	int y_0 = idx_t_int + idx_x_int + _num_xs * (floor_y + _num_ys * idx_z_int);
	//	int y_1 = idx_t_int + idx_x_int + _num_xs * (next_y + _num_ys * idx_z_int);

	//	avg_x = (_wind[y_0].x_comp + _wind[y_1].x_comp) / 2;
	//	avg_y = (_wind[y_0].y_comp + _wind[y_1].y_comp) / 2;
	//	avg_z = (_wind[y_0].z_comp + _wind[y_1].z_comp) / 2;
	//}
	//else if (idx_x_int != -1 && idx_y_int != -1 && idx_z_int == -1) {
	//	// UAV only needs to average data at z axis
	//	int floor_z = static_cast<int>(floor(target.get_z() / _dz));
	//	int next_z = floor_z;
	//	if (floor_z < _num_zs - 1) next_z++;

	//	int z_0 = idx_t_int + idx_x_int + _num_xs * (idx_y_int + _num_ys * floor_z);
	//	int z_1 = idx_t_int + idx_x_int + _num_xs * (idx_y_int + _num_ys * next_z);

	//	avg_x = (_wind[z_0].x_comp + _wind[z_1].x_comp) / 2;
	//	avg_y = (_wind[z_0].y_comp + _wind[z_1].y_comp) / 2;
	//	avg_z = (_wind[z_0].z_comp + _wind[z_1].z_comp) / 2;
	//}
	//else if (idx_x_int == -1 && idx_y_int == -1 && idx_z_int != -1) {
	//	// Consider this scenario as averaging vectors in a rectangle: 
	//	// p3--------p2
	//	// ------------
	//	// -----UAV----
	//	// ------------
	//	// p0--------p1
	//	// UAV only needs to average data at xy-plane
	//	int floor_x = static_cast<int>(floor(target.get_x() / _dx));
	//	int floor_y = static_cast<int>(floor(target.get_y() / _dy));
	//	int next_x = floor_x;
	//	if (floor_x < _num_xs - 1) next_x++;
	//	int next_y = floor_y;
	//	if (floor_y < _num_ys - 1) next_y++;

	//	int xy_0 = idx_t_int + floor_x + _num_xs * (floor_y + _num_ys * idx_z_int);
	//	int xy_1 = idx_t_int + next_x + _num_xs * (floor_y + _num_ys * idx_z_int);
	//	int xy_2 = idx_t_int + floor_x + _num_xs * (next_y + _num_ys * idx_z_int);
	//	int xy_3 = idx_t_int + next_x + _num_xs * (next_y + _num_ys * idx_z_int);

	//	avg_x = (_wind[xy_0].x_comp + _wind[xy_1].x_comp +
	//		_wind[xy_2].x_comp + _wind[xy_3].x_comp) / 4;
	//	avg_y = (_wind[xy_0].y_comp + _wind[xy_1].y_comp +
	//		_wind[xy_2].y_comp + _wind[xy_3].y_comp) / 4;
	//	avg_z = (_wind[xy_0].z_comp + _wind[xy_1].z_comp +
	//		_wind[xy_2].z_comp + _wind[xy_3].z_comp) / 4;
	//}
	//else if (idx_x_int == -1 && idx_y_int != -1 && idx_z_int == -1) {
	//	// UAV only needs to average data at xz-plane
	//	int floor_x = static_cast<int>(floor(target.get_x() / _dx));
	//	int floor_z = static_cast<int>(floor(target.get_z() / _dz));
	//	int next_x = floor_x;
	//	if (floor_x < _num_xs - 1) next_x++;
	//	int next_z = floor_z;
	//	if (floor_z < _num_zs - 1) next_z++;

	//	int xz_0 = idx_t_int + floor_x + _num_xs * (idx_y_int + _num_ys * floor_z);
	//	int xz_1 = idx_t_int + next_x + _num_xs * (idx_y_int + _num_ys * floor_z);
	//	int xz_2 = idx_t_int + floor_x + _num_xs * (idx_y_int + _num_ys * next_z);
	//	int xz_3 = idx_t_int + next_x + _num_xs * (idx_y_int + _num_ys * next_z);

	//	avg_x = (_wind[xz_0].x_comp + _wind[xz_1].x_comp +
	//		_wind[xz_2].x_comp + _wind[xz_3].x_comp) / 4;
	//	avg_y = (_wind[xz_0].y_comp + _wind[xz_1].y_comp +
	//		_wind[xz_2].y_comp + _wind[xz_3].y_comp) / 4;
	//	avg_z = (_wind[xz_0].z_comp + _wind[xz_1].z_comp +
	//		_wind[xz_2].z_comp + _wind[xz_3].z_comp) / 4;
	//}
	//else if (idx_x_int != -1 && idx_y_int == -1 && idx_z_int == -1) {
	//	// UAV only needs to average data at yz-plane
	//	int floor_y = static_cast<int>(floor(target.get_y() / _dy));
	//	int floor_z = static_cast<int>(floor(target.get_z() / _dz));
	//	int next_y = floor_y;
	//	if (floor_y < _num_ys - 1) next_y++;
	//	int next_z = floor_z;
	//	if (floor_z < _num_zs - 1) next_z++;

	//	int yz_0 = idx_t_int + idx_x_int + _num_xs * (floor_y + _num_ys * floor_z);
	//	int yz_1 = idx_t_int + idx_x_int + _num_xs * (next_y + _num_ys * floor_z);
	//	int yz_2 = idx_t_int + idx_x_int + _num_xs * (floor_y + _num_ys * next_z);
	//	int yz_3 = idx_t_int + idx_x_int + _num_xs * (next_y + _num_ys * next_z);

	//	avg_x = (_wind[yz_0].x_comp + _wind[yz_1].x_comp +
	//		_wind[yz_2].x_comp + _wind[yz_3].x_comp) / 4;
	//	avg_y = (_wind[yz_0].y_comp + _wind[yz_1].y_comp +
	//		_wind[yz_2].y_comp + _wind[yz_3].y_comp) / 4;
	//	avg_z = (_wind[yz_0].z_comp + _wind[yz_1].z_comp +
	//		_wind[yz_2].z_comp + _wind[yz_3].z_comp) / 4;
	//}
}
