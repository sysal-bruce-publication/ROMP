#pragma once
#include <memory>
#include "Wind.h"
#include "Sensor.h"

#ifndef DRONE_H
#define DRONE_H

constexpr double pi = 3.141592653;
constexpr double a_g = 9.80665;

class Drone
{
public:
	Drone() {}
	Drone(const Point& init_coord, double init_eng, double init_time);
	~Drone() {}

	size_t get_task_node_num_min() const { return _task_node_num_min; }
	Point get_coord() const { return *_coord; }
	double get_energy() const { return _E; }
	double get_time() const { return _T; }
	double get_dist() const { return _dist; }
	double get_norm_alt() const { return _norm_alt; }
	double get_hover_alt() const { return _hover_alt; }
	double get_horz_vel() const { return _horz_vel; }
	double get_vert_vel() const { return _vert_vel; }
	double get_freq() const { return 1. / _dt; }
	double get_energy_cost(const WindGrid& wind, const Sensor& target_node);

	void set_coord(const Point& p) { _coord->set_pt(p); }
	void set_energy(double eng) { _E = eng; }
	void set_time(double time) { _T = time; }
	void set_freq(double freq) { _dt = 1. / freq; }
	void reset_state(const Point& init_coord, double init_eng, double init_time);

	bool takeoff(const WindGrid& wind, const Point& target, bool force_exe);
	bool landing(const WindGrid& wind, const Point& target, bool force_exe);
	bool cruise(const WindGrid& wind, const Point& target, bool force_exe);
	void return_to_home(const WindGrid& wind, const Point& home);

private:
	// Model parameters
	std::unique_ptr<Point> _coord;
	double _E         = 0;
	double _T         = 0;
	double _dist      = 0;
	const double _norm_alt  = 15;
	const double _hover_alt = 0;
	// M100 specification
	const double _init_E         = 99.9;
	const double _horz_vel       = 10;
	const double _vert_vel       = 3;
	const double _mass           = 3.107;
	const double _prop_area      = pi * 0.021904;
	const double _horz_body_area = 0.152818;
	const double _vert_body_area = pi * 0.248004;
	// Aerodynamics physics constants
	const double _rho_air               = 1.25;
	const double _c_drag                = 0.04;
	const double _horz_wind_force_const = _rho_air * _horz_body_area;
	const double _vert_wind_force_const = _rho_air * _vert_body_area;
	const double _prop_drag_force_const = 0.5 * _rho_air * _c_drag * _prop_area;
	const double _horz_drag_force_const = 0.5 * _rho_air * _c_drag * _horz_body_area;
	const double _vert_drag_force_const = 0.5 * _rho_air * _c_drag * _vert_body_area;
	// System parameters
	double _dt                = 10;
	size_t _task_node_num_min = 10;

	void _calc_plane_vec(
		const Point& target, double& vec_x, double& vec_y, double& angle);
};

#endif // !DRONE_H

