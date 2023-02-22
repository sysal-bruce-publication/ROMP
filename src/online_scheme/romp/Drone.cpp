#include "Drone.h"

using std::unique_ptr, std::make_unique;
using std::abs;

Drone::Drone(const Point& init_coord, double init_eng, double init_time) :
	_E(init_eng), _T(init_time)
{
	_coord = make_unique<Point>(init_coord);
}

void Drone::reset_state(const Point& init_coord, double init_eng, double init_time)
{
	_coord->set_pt(init_coord);
	_E = init_eng;
	_T = init_time;
	_dist = 0;
}

double Drone::get_energy_cost(const WindGrid& wind, const Sensor& target_node)
{
	double init_E = _E;
	unique_ptr<Point> target_pt = make_unique<Point>(
		_coord->get_x(), _coord->get_y(), _norm_alt);
	takeoff(wind, *target_pt, true);
	target_pt->set_x(target_node.get_pt().get_x());
	target_pt->set_y(target_node.get_pt().get_y());
	cruise(wind, *target_pt, true);

	double ipt_cost = 0, ipt_charged = 0;
	target_node.get_chrg_pkg(ipt_cost, ipt_charged);
	_E -= ipt_cost;

	target_pt->set_z(_hover_alt);
	landing(wind, *target_pt, true);
	return init_E - _E;
}

bool Drone::takeoff(const WindGrid& wind, const Point& target, bool force_exe)
{
	double dist = _coord->get_diff_z(target);
	double time = dist / _vert_vel;
	double step_E = 0, step_T = 0, time_step = _dt;
	unique_ptr<Point> step_coord = make_unique<Point>(*_coord);
	double wind_x = 0, wind_y = 0, wind_z = 0;	// Wind vector
	// Velocity, acceleration and displacement caused by wind force
	double vt_x = 0, vt_y = 0, a_x = 0, a_y = 0, dist_x = 0, dist_y = 0;

	if (time <= 1.5 * _dt) {
		wind.get_vector(_T, *step_coord, wind_x, wind_y, wind_z);
		// Horizontal
		a_x = _horz_wind_force_const * wind_x * abs(wind_x) / _mass;
		a_y = _horz_wind_force_const * wind_y * abs(wind_y) / _mass;
		dist_x = 0.5 * a_x * pow(time, 2);
		dist_y = 0.5 * a_y * pow(time, 2);
		// Vertical 
		double thrust = _prop_drag_force_const * pow(_vert_vel - wind_z, 2)
			+ 0.25 * _mass * a_g;
		double P_mech = 4 * sqrt(pow(thrust, 3) / (2 * _rho_air * _prop_area));
		step_E = P_mech * time / 3600;  // Convert to Wh
		step_T = time;

		if (_E <= step_E && !force_exe) { return false; }
		step_coord->set_x(step_coord->get_x() + dist_x);
		step_coord->set_y(step_coord->get_y() + dist_y);
		step_coord->set_z(step_coord->get_z() + _vert_vel * time);
	}
	else {
		while (step_T < time) {
			wind.get_vector(_T + step_T, *step_coord, wind_x, wind_y, wind_z);
			if (time - step_T < 2 * _dt) { time_step = time - step_T; }
			step_T += time_step;
			// Horizontal 
			a_x = _horz_wind_force_const * wind_x * abs(wind_x) / _mass;
			a_y = _horz_wind_force_const * wind_y * abs(wind_y) / _mass;
			dist_x = vt_x * time_step + 0.5 * a_x * pow(time_step, 2);
			dist_y = vt_y * time_step + 0.5 * a_y * pow(time_step, 2);
			vt_x += a_x * time_step;
			vt_y += a_y * time_step;
			// Vertical
			double thrust = _prop_drag_force_const * pow(_vert_vel - wind_z, 2)
				+ 0.25 * _mass * a_g;
			double P_mech = 4 * sqrt(pow(thrust, 3) / (2 * _rho_air * _prop_area));
			step_E += P_mech * time_step / 3600;
			if (_E <= step_E && !force_exe) { return false; }
			step_coord->set_x(step_coord->get_x() + dist_x);
			step_coord->set_y(step_coord->get_y() + dist_y);
			step_coord->set_z(step_coord->get_z() + _vert_vel * time_step);
		}
	}
	// In-place status update 
	_dist += dist;
	_T += step_T;
	_E -= step_E;
	_coord->set_pt(*step_coord);
	return true;
}

bool Drone::landing(const WindGrid& wind, const Point& target, bool force_exe)
{
	double dist = _coord->get_diff_z(target);
	double time = dist / _vert_vel;
	double step_E = 0, step_T = 0, time_step = _dt;
	unique_ptr<Point> step_coord = make_unique<Point>(*_coord);

	double wind_x = 0, wind_y = 0, wind_z = 0;
	double vt_x = 0, vt_y = 0, a_x = 0, a_y = 0, dist_x = 0, dist_y = 0;
	
	if (time <= 1.5 * _dt) {
		wind.get_vector(_T, *step_coord, wind_x, wind_y, wind_z);
		// Horizontal
		a_x = _horz_wind_force_const * wind_x * abs(wind_x) / _mass;
		a_y = _horz_wind_force_const * wind_y * abs(wind_y) / _mass;
		dist_x = 0.5 * a_x * pow(time, 2);
		dist_y = 0.5 * a_y * pow(time, 2);
		// Vertical
		double thrust = 0.25 * _mass * a_g 
			- _prop_drag_force_const * pow(_vert_vel - wind_z, 2);
		double P_mech = 4 * sqrt(pow(thrust, 3) / (2 * _rho_air * _prop_area));
		step_E = P_mech * time / 3600;
		step_T = time;

		if (_E <= step_E && !force_exe) { return false; }
		step_coord->set_x(step_coord->get_x() + dist_x);
		step_coord->set_y(step_coord->get_y() + dist_y);
		step_coord->set_z(step_coord->get_z() - this->_vert_vel * time);
	}
	else {
		while (step_T < time) {
			wind.get_vector(_T + step_T, *step_coord, wind_x, wind_y, wind_z);
			if (time - step_T < 2 * _dt) { time_step = time - step_T; }
			step_T += time_step;
			// Horizontal
			a_x = _horz_wind_force_const * wind_x * abs(wind_x) / _mass;
			a_y = _horz_wind_force_const * wind_y * abs(wind_y) / _mass;
			dist_x = vt_x * time_step + 0.5 * a_x * pow(time_step, 2);
			dist_y = vt_y * time_step + 0.5 * a_y * pow(time_step, 2);
			vt_x += a_x * time_step;
			vt_y += a_y * time_step;
			double thrust = 0.25 * _mass * a_g
				- _prop_drag_force_const * pow(_vert_vel - wind_z, 2);
			double P_mech = 4 * sqrt(pow(thrust, 3) / (2 * _rho_air * _prop_area));
			step_E += P_mech * time / 3600;
			if (_E <= step_E && !force_exe) { return false; }
			step_coord->set_z(step_coord->get_z() - this->_vert_vel * time);
		}
	}
	// In-place status update
	_dist += dist;
	_T += step_T;
	_E -= step_E;
	_coord->set_pt(*step_coord);
	return true;
}

void Drone::_calc_plane_vec(const Point& target,
	double& vec_x, double& vec_y, double& angle)
{
	double tar_x = target.get_x(), tar_y = target.get_y();
	double uav_x = _coord->get_x(), uav_y = _coord->get_y();
	
	if (tar_x > uav_x) {
		if (tar_y > uav_y) {
			angle = atan((tar_y - uav_y) / (tar_x - uav_x));
			vec_x = _horz_vel * cos(angle);
			vec_y = _horz_vel * sin(angle);
		}
		else if (tar_y < uav_y) {
			angle = atan((uav_y - tar_y) / (tar_x - uav_x));
			vec_x = _horz_vel * cos(angle);
			vec_y = -_horz_vel * sin(angle);
		}
		else {
			angle = 0;
			vec_x = _horz_vel;
			vec_y = 0;
		}
	}
	else if (tar_x < uav_x) {
		if (tar_y > uav_y) {
			angle = atan((tar_y - uav_y) / (uav_x - tar_x));
			vec_x = -_horz_vel * cos(angle);
			vec_y = _horz_vel * sin(angle);
		}
		else if (tar_y < uav_y) {
			angle = atan((uav_y - tar_y) / (uav_x - tar_x));
			vec_x = -_horz_vel * cos(angle);
			vec_y = -_horz_vel * sin(angle);
		}
		else {
			angle = 0;
			vec_x = -_horz_vel;
			vec_y = 0;
		}
	}
	else {
		if (tar_y > uav_y) {
			angle = 0.5 * pi;
			vec_x = 0;
			vec_y = _horz_vel;
		}
		else if (tar_y < uav_y) {
			angle = 0.5 * pi;
			vec_x = 0;
			vec_y = -_horz_vel;
		}
	}
}

bool Drone::cruise(const WindGrid& wind, const Point& target, bool force_exe)
{
	double dist = (*_coord - target).get_xy_norm2();
	double time = dist / _horz_vel;
	double step_E = 0, step_T = 0, time_step = _dt;
	unique_ptr<Point> step_coord = make_unique<Point>(*_coord);
	
	double v_xy_x = 0, v_xy_y = 0, atan_val = 0, dist_x = 0, dist_y = 0;
	_calc_plane_vec(target, v_xy_x, v_xy_y, atan_val);
	double prop_area_x = pi * pow(0.15 * sin(atan_val), 2);
	double prop_area_y = pi * pow(0.15 * cos(atan_val), 2);

	double wind_x = 0, wind_y = 0, wind_z = 0;
	if (time <= 1.5 * _dt) {
		wind.get_vector(_T, *step_coord, wind_x, wind_y, wind_z);
		dist_x = v_xy_x * time;
		dist_y = v_xy_y * time;

		double drag_x = 0.5 * _rho_air * _c_drag * prop_area_y * pow(v_xy_x - wind_x, 2);
		double drag_y = 0.5 * _rho_air * _c_drag * prop_area_x * pow(v_xy_y - wind_y, 2);
		double drag_xy_square = pow(drag_x, 2) + pow(drag_y, 2);
		double thrust = sqrt(drag_xy_square + pow(0.25 * _mass * a_g, 2));
		double P_mech = 4 * sqrt(pow(thrust, 3) / (2 * _rho_air * _prop_area));
		step_E += P_mech * time_step / 3600;

		if (_E <= step_E && !force_exe) { return false; }
		step_coord->set_x(step_coord->get_x() + dist_x);
		step_coord->set_y(step_coord->get_y() + dist_y);
	}
	else {
		while (step_T < time) {
			wind.get_vector(_T + step_T, *step_coord, wind_x, wind_y, wind_z);
			if (time - step_T < 2 * _dt) { time_step = time - step_T; }
			step_T += time_step;

			dist_x = v_xy_x * time_step;
			dist_y = v_xy_y * time_step;

			double drag_x = 0.5 * _rho_air * _c_drag * prop_area_y * pow(v_xy_x - wind_x, 2);
			double drag_y = 0.5 * _rho_air * _c_drag * prop_area_x * pow(v_xy_y - wind_y, 2);
			double drag_xy_square = pow(drag_x, 2) + pow(drag_y, 2);

			double thrust = sqrt(drag_xy_square + pow(0.25 * _mass * a_g, 2));
			double P_mech = 4 * sqrt(pow(thrust, 3) / (2 * _rho_air * _prop_area));
			step_E += P_mech * time_step / 3600;

			if (_E <= step_E && !force_exe) { return false; }
			step_coord->set_x(step_coord->get_x() + dist_x);
			step_coord->set_y(step_coord->get_y() + dist_y);
		}
	}

	_dist += dist;
	_T += step_T;
	_E -= step_E;
	_coord->set_x(step_coord->get_x());
	_coord->set_y(step_coord->get_y());
	return true;
}

void Drone::return_to_home(const WindGrid& wind, const Point& home)
{
	unique_ptr<Point> base;
	// Use base as ascent target first.
	if (_coord->get_z() < _norm_alt) {
		base = make_unique<Point>(_coord->get_x(), _coord->get_y(), _norm_alt);
		takeoff(wind, *base, true);
		base->set_pt(home.get_x(), home.get_y(), _norm_alt);
	}
	else {
		base = make_unique<Point>(home.get_x(), home.get_y(), _norm_alt);
	}

	cruise(wind, *base, true);
	base->set_z(0);
	landing(wind, *base, true);
}
