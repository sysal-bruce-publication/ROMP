/* @file drone.cpp
*
*  Copyright (C) Qiuchen Qian, 2021
*  Imperial College, London
*/

#include "drone.h"

using namespace std;

void Drone::resetState(float uav_energy, const Point& coordinate)
{
	this->_coordinate = coordinate;
	this->_energy = uav_energy;
	this->_time = 0;
	this->_distance = 0;
}

void Drone::_calc2DPlaneVec(const Point& target, float& vec_x, float& vec_y, float& angle)
{
	float tar_x = target.getX();
	float tar_y = target.getY();
	float uav_x = this->_coordinate.getX();
	float uav_y = this->_coordinate.getY();

	if (tar_x > uav_x) {
		if (tar_y > uav_y) {
			angle = atan((tar_y - uav_y) / (tar_x - uav_x));
			vec_x = this->_horizontal_velocity * cos(angle);
			vec_y = this->_horizontal_velocity * sin(angle);
		}
		else if (tar_y < uav_y) {
			angle = atan((uav_y - tar_y) / (tar_x - uav_x));
			vec_x = this->_horizontal_velocity * cos(angle);
			vec_y = -1 * this->_horizontal_velocity * sin(angle);
		}
		else {
			angle = 0;
			vec_x = this->_horizontal_velocity;
			vec_y = 0;
		}
	}
	else if (tar_x < uav_x) {
		if (tar_y > uav_y) {
			angle = atan((tar_y - uav_y) / (uav_x - tar_x));
			vec_x = -1 * this->_horizontal_velocity * cos(angle);
			vec_y = this->_horizontal_velocity * sin(angle);
		}
		else if (tar_y < uav_y) {
			angle = atan((uav_y - tar_y) / (uav_x - tar_x));
			vec_x = -1 * this->_horizontal_velocity * cos(angle);
			vec_y = -1 * this->_horizontal_velocity * sin(angle);
		}
		else {
			angle = 0;
			vec_x = -1 * this->_horizontal_velocity;
			vec_y = 0;
		}
	}
	else {
		if (tar_y > uav_y) {
			angle = this->_pi / 2;
			vec_x = 0;
			vec_y = this->_horizontal_velocity;
		}
		else if (tar_y < uav_y) {
			angle = this->_pi / 2;
			vec_x = 0;
			vec_y = -1 * this->_horizontal_velocity;
		}
		else {
			throw runtime_error("[" + to_string(target.getX()) + ", " + to_string(target.getY()) + "] same as current PDV position.\n");
		}
	}
}

bool Drone::ascent(const Point& target, bool force_exe)
{
	float ascent_dist = this->_coordinate.calcZZDist(target);
	float ascent_time = ascent_dist / this->_vertical_velocity;

	//! Initialize parameters needed for each step, record starting PDV states. 
	//! Step_time is total time passed.
	float step_eng = 0, step_time = 0, timestep = static_cast<float>(this->_dt);
	unique_ptr<Point> step_pos(new Point(this->_coordinate.getX(),
		this->_coordinate.getY(), this->_coordinate.getZ()));

	//! Wind vector
	float wind_x = 0, wind_y = 0, wind_z = 0;
	//! Velocity, acceleration and displacement caused by wind force
	float vt_x = 0, vt_y = 0, a_x = 0, a_y = 0, dist_x = 0, dist_y = 0;

	if (ascent_time <= 1.5 * this->_dt) {
		this->_wind_grid->getWindData(this->_time, *step_pos, wind_x, wind_y, wind_z);

		a_x = this->_wind_force_const_horz * wind_x * abs(wind_x) / this->_mass;
		a_y = this->_wind_force_const_horz * wind_y * abs(wind_y) / this->_mass;

		//! [STEP 4] Update trivial axis displacement after time step x = v0t + 1/2 a t^2
		dist_x = 0.5f * a_x * powf(ascent_time, 2);
		dist_y = 0.5f * a_y * powf(ascent_time, 2);

		//! [STEP 6] Update thrust along z-axis according to quadcopter equilibrium
		float thrust = this->_prop_drag_const * powf(this->_vertical_velocity - wind_z, 2)
			+ this->_mass * this->_g / 4;

		//! [STEP 7] From thrust, calculate approximation power
		float mech_power = 4 * sqrtf(powf(thrust, 3) / (2 * this->_rho_air * this->_area_propeller));
		step_eng = mech_power * ascent_time / 3600;
		step_time = ascent_time;

		//! If remain energy is not enough and not required to forcely execute
		if (this->_energy <= step_eng && !force_exe) return false;

		step_pos->setX(step_pos->getX() + dist_x);
		step_pos->setY(step_pos->getY() + dist_y);
		step_pos->setZ(step_pos->getZ() + this->_vertical_velocity * ascent_time);
	}
	else {
		while (step_time < ascent_time) {
			//! [STEP 1] Get wind vector according to time stamp
			this->_wind_grid->getWindData(this->_time + step_time, *step_pos, wind_x, wind_y, wind_z);

			//! [STEP 2] Update time step length
			if (ascent_time - step_time < 2 * this->_dt) timestep = ascent_time - step_time;
			step_time += timestep;

			//! [STEP 3] Update trivial axis acceleration = drag force / pdv mass, if applicable
			a_x = this->_wind_force_const_horz * wind_x * abs(wind_x) / this->_mass;
			a_y = this->_wind_force_const_horz * wind_y * abs(wind_y) / this->_mass;

			//! [STEP 4] Update trivial axis displacement after time step x = v0t + 1/2 a t^2
			dist_x = vt_x * timestep + 0.5f * a_x * powf(timestep, 2);
			dist_y = vt_y * timestep + 0.5f * a_y * powf(timestep, 2);

			//! [STEP 5] Update trivial axis velocity vt = v0 + at after time step
			vt_x += a_x * timestep;
			vt_y += a_y * timestep;

			//! [STEP 6] Update thrust along z-axis according to quadcopter equilibrium
			float thrust = this->_prop_drag_const * powf(this->_vertical_velocity - wind_z, 2)
				+ this->_mass * this->_g / 4;

			//! [STEP 7] From thrust, calculate approximation power
			float mech_power = 4 * sqrtf(powf(thrust, 3) / (2 * this->_rho_air * this->_area_propeller));
			step_eng += mech_power * timestep / 3600;

			//! If remain energy is not enough and not required to forcely execute
			if (this->_energy <= step_eng && !force_exe) return false;

			//! [STEP 8] Update agent position 
			step_pos->setX(step_pos->getX() + dist_x);
			step_pos->setY(step_pos->getY() + dist_y);
			step_pos->setZ(step_pos->getZ() + this->_vertical_velocity * timestep);
		}
	}

	this->_distance += ascent_dist;
	this->_time += step_time;
	this->_energy -= step_eng;
	this->setPt(*step_pos);

	return true;
}

bool Drone::descent(const Point& target, bool force_exe)
{
	float descent_dist = this->_coordinate.calcZZDist(target);
	float descent_time = descent_dist / this->_vertical_velocity;

	//! Initialize parameters needed for each step, record starting PDV states. 
	//! Step_time is total time passed.
	float step_eng = 0, step_time = 0, timestep = static_cast<float>(this->_dt);
	unique_ptr<Point> step_pos(new Point(this->_coordinate.getX(),
		this->_coordinate.getY(), this->_coordinate.getZ()));

	//! Wind vector
	float wind_x = 0, wind_y = 0, wind_z = 0;
	//! Velocity, acceleration and displacement caused by wind force
	float vt_x = 0, vt_y = 0, a_x = 0, a_y = 0, dist_x = 0, dist_y = 0;

	if (descent_time <= 1.5 * this->_dt) {
		this->_wind_grid->getWindData(this->_time, *step_pos, wind_x, wind_y, wind_z);

		a_x = this->_wind_force_const_horz * wind_x * abs(wind_x) / this->_mass;
		a_y = this->_wind_force_const_horz * wind_y * abs(wind_y) / this->_mass;

		//! [STEP 4] Update trivial axis displacement after time step x = v0t + 1/2 a t^2
		dist_x = 0.5f * a_x * powf(descent_time, 2);
		dist_y = 0.5f * a_y * powf(descent_time, 2);

		//! [STEP 6] Update thrust along z-axis according to quadcopter equilibrium
		float thrust = this->_mass * this->_g / 4
			- this->_prop_drag_const * powf(this->_vertical_velocity - wind_z, 2);

		//! [STEP 7] From thrust, calculate approximation power
		float mech_power = 4 * sqrtf(powf(thrust, 3) / (2 * this->_rho_air * this->_area_propeller));
		step_eng = mech_power * descent_time / 3600;
		step_time = descent_time;

		//! If remain energy is not enough and not required to forcely execute
		if (this->_energy <= step_eng && !force_exe) return false;

		step_pos->setX(step_pos->getX() + dist_x);
		step_pos->setY(step_pos->getY() + dist_y);
		step_pos->setZ(step_pos->getZ() - this->_vertical_velocity * descent_time);
	}
	else {
		while (step_time < descent_time) {
			//! [STEP 1] Get wind vector according to time stamp
			this->_wind_grid->getWindData(this->_time + step_time, *step_pos, wind_x, wind_y, wind_z);

			//! [STEP 2] Update time step length
			if (descent_time - step_time < 2 * this->_dt) timestep = descent_time - step_time;
			step_time += timestep;

			//! [STEP 3] Update trivial axis acceleration = drag force / pdv mass, if applicable
			a_x = this->_wind_force_const_horz * wind_x * abs(wind_x) / this->_mass;
			a_y = this->_wind_force_const_horz * wind_y * abs(wind_y) / this->_mass;

			//! [STEP 4] Update trivial axis displacement after time step x = v0t + 1/2 a t^2
			dist_x = vt_x * timestep + 0.5f * a_x * powf(timestep, 2);
			dist_y = vt_y * timestep + 0.5f * a_y * powf(timestep, 2);

			//! [STEP 5] Update trivial axis displacement after time step x = v0t + 1/2 a t^2
			vt_x += a_x * timestep;
			vt_y += a_y * timestep;

			//! [STEP 6] Update thrust along z-axis according to quadcopter equilibrium
			float thrust = this->_mass * this->_g / 4
				- this->_prop_drag_const * powf(this->_vertical_velocity - wind_z, 2);

			//! [STEP 7] From thrust, calculate approximation power [Wh]
			float mech_power = 4 * sqrtf(powf(thrust, 3) / (2 * this->_rho_air * this->_area_propeller));
			step_eng += mech_power * timestep / 3600;

			//! If remain energy is not enough and not required to forcely execute
			if (this->_energy <= step_eng && !force_exe) return false;

			//! [STEP 8] Update agent position 
			step_pos->setX(step_pos->getX() + dist_x);
			step_pos->setY(step_pos->getY() + dist_y);
			step_pos->setZ(step_pos->getZ() - this->_vertical_velocity * timestep);
		}
	}

	this->_distance += descent_dist;
	this->_time += step_time;
	this->_energy -= step_eng;
	this->setPt(*step_pos);

	return true;
}

bool Drone::steadyFlight(const Point& target, bool force_exe)
{
	float steady_dist = this->_coordinate.calcXYDist(target);
	float steady_time = steady_dist / this->_horizontal_velocity;

	//! Initialize parameters needed for each step, record starting PDV states. 
	//! Step_time is total time passed.
	float step_eng = 0, step_time = 0, timestep = static_cast<float>(this->_dt);
	unique_ptr<Point> step_pos(new Point(this->_coordinate.getX(),
		this->_coordinate.getY(), this->_coordinate.getZ()));

	//! Drone steady flight direction variables
	float v_xy_x = 0, v_xy_y = 0, dist_x = 0, dist_y = 0;
	float atan_val = 0;					/* atan_val is always abs(y) / abs(x) */
	try { this->_calc2DPlaneVec(target, v_xy_x, v_xy_y, atan_val); }
	catch (runtime_error& e) { cerr << e.what(); return false; }
	//! Calculate projection of propeller at xz-plane and yz-plane.
	float prop_area_x = this->_pi * powf(0.15f * sinf(atan_val), 2);
	float prop_area_y = this->_pi * powf(0.15f * cosf(atan_val), 2);

	//! Wind vector
	float wind_x = 0, wind_y = 0, wind_z = 0;
	//! Velocity, acceleration and displacement caused by wind force
	float vt_z = 0, a_z = 0, dist_z = 0;

	if (steady_time <= 1.5 * this->_dt) {
		this->_wind_grid->getWindData(this->_time, *step_pos, wind_x, wind_y, wind_z);

		a_z = this->_wind_force_const_vert * wind_z * abs(wind_z) / this->_mass;
		dist_z = 0.5f * a_z * powf(steady_time, 2);

		//! [STEP 4] Steady state flight, no acceleration in xy-plane
		dist_x = v_xy_x * steady_time;
		dist_y = v_xy_y * steady_time;

		//! [STEP 5] Calculate total drag from x and y component
		float drag_x = 0.5f * this->_rho_air * this->_c_drag * prop_area_y * powf(v_xy_x - wind_x, 2);
		float drag_y = 0.5f * this->_rho_air * this->_c_drag * prop_area_x * powf(v_xy_y - wind_y, 2);
		float drag_xy_square = drag_x * drag_x + drag_y * drag_y;

		//! [STEP 6] Update thrust
		float thrust = sqrtf(drag_xy_square + powf(this->_mass * this->_g / 4, 2));

		//! [STEP 7] From thrust, calculate approximation power
		float mech_power = 4 * sqrtf(powf(thrust, 3) / (2 * this->_rho_air * this->_area_propeller));
		step_eng += mech_power * timestep / 3600;

		//! If remain energy is not enough and not required to forcely execute
		if (this->_energy <= step_eng && !force_exe) return false;

		step_pos->setX(step_pos->getX() + dist_x);
		step_pos->setY(step_pos->getY() + dist_y);
		step_pos->setZ(step_pos->getZ() + dist_z);
	}
	else {
		while (step_time < steady_time) {
			//! [STEP 1] Get wind vector according to time stamp
			this->_wind_grid->getWindData(this->_time + step_time, *step_pos, wind_x, wind_y, wind_z);

			//! [STEP 2] Update time step length
			if (steady_time - step_time < 2 * this->_dt) timestep = steady_time - step_time;
			step_time += timestep;

			//! [STEP 3] Update trivial Z-axis movement caused by vertical wind
			a_z = this->_wind_force_const_vert * wind_z * abs(wind_z) / this->_mass;
			dist_z = vt_z * timestep + 0.5f * a_z * powf(timestep, 2);
			vt_z += a_z * timestep;

			//! [STEP 4] Steady state flight, no acceleration in xy-plane
			dist_x = v_xy_x * timestep;
			dist_y = v_xy_y * timestep;

			//! [STEP 5] Calculate total drag from x and y component
			float drag_x = 0.5f * this->_rho_air * this->_c_drag * prop_area_y * powf(v_xy_x - wind_x, 2);
			float drag_y = 0.5f * this->_rho_air * this->_c_drag * prop_area_x * powf(v_xy_y - wind_y, 2);
			float drag_xy_square = drag_x * drag_x + drag_y * drag_y;

			//! [STEP 6] Calculate angle between the thrust and xy-plane, then update thrust
			float thrust = sqrtf(drag_xy_square + powf(this->_mass * this->_g / 4, 2));

			//! [STEP 7] From thrust, calculate approximation power
			float mech_power = 4 * sqrtf(powf(thrust, 3) / (2 * this->_rho_air * this->_area_propeller));
			step_eng += mech_power * timestep / 3600;

			//! If remain energy is not enough and not required to forcely execute
			if (this->_energy <= step_eng && !force_exe) return false;

			step_pos->setX(step_pos->getX() + dist_x);
			step_pos->setY(step_pos->getY() + dist_y);
			step_pos->setZ(step_pos->getZ() + dist_z);
		}
	}

	this->_distance += steady_dist;
	this->_time += step_time;
	this->_energy -= step_eng;
	this->setPt(*step_pos);

	return true;
}

bool Drone::obstacleFlight(float offset, int obs_case, Obstacle* cur_obs,
	const Point& avoid_p, const Point& target, bool force_exe)
{
	int dir_x = 0, dir_y = 0;
	if (avoid_p.getX() > this->_coordinate.getX()) dir_x = 1;
	else if (avoid_p.getX() < this->_coordinate.getX()) dir_x = -1;
	if (avoid_p.getY() > this->_coordinate.getY()) dir_y = 1;
	else if (avoid_p.getY() < this->_coordinate.getY()) dir_y = -1;

	if (!this->steadyFlight(avoid_p, force_exe)) return false;

	if (obs_case != 3 && obs_case != 6 && obs_case != 9 && obs_case != 12) {
		unique_ptr<Point> p2(new Point(avoid_p.getX(), avoid_p.getY(), 0));

		switch (obs_case) {
		case 5:
			// 2 * (offset / sqrt(2))
			if (dir_y == 0) throw runtime_error("In obs_case " + to_string(obs_case) + " dir_y = 0\n");
			p2->setY(avoid_p.getY() + dir_y * (cur_obs->getW() + offset * sqrtf(2)));
			if (!this->steadyFlight(*p2, force_exe)) return false;
			break;
		case 7:
			if (dir_y == 0) throw runtime_error("In obs_case " + to_string(obs_case) + " dir_y = 0\n");
			p2->setY(avoid_p.getY() + dir_y * (cur_obs->getW() + offset * sqrtf(2)));
			if (!this->steadyFlight(*p2, force_exe)) return false;
			break;
		case 10:
			if (dir_y == 0) throw runtime_error("In obs_case " + to_string(obs_case) + " dir_x = 0\n");
			p2->setX(avoid_p.getX() + dir_x * (cur_obs->getL() + offset * sqrtf(2)));
			if (!this->steadyFlight(*p2, force_exe)) return false;
			break;
		case 11:
			if (dir_y == 0) throw runtime_error("In obs_case " + to_string(obs_case) + " dir_x = 0\n");
			p2->setX(avoid_p.getX() + dir_x * (cur_obs->getL() + offset * sqrtf(2)));
			if (!this->steadyFlight(*p2, force_exe)) return false;
			break;
		case 13:
			if (dir_y == 0) throw runtime_error("In obs_case " + to_string(obs_case) + " dir_y = 0\n");
			p2->setY(avoid_p.getY() + dir_y * (cur_obs->getW() + offset * sqrtf(2)));
			if (!this->steadyFlight(*p2, force_exe)) return false;
			break;
		case 14:
			if (dir_y == 0) throw runtime_error("In obs_case " + to_string(obs_case) + " dir_x = 0\n");
			p2->setX(avoid_p.getX() + dir_x * (cur_obs->getL() + offset * sqrtf(2)));
			if (!this->steadyFlight(*p2, force_exe)) return false;
			break;
		default:
			break;
		}
	}

	if (!this->steadyFlight(target, force_exe)) return false;
	else return true;
}

void Drone::returnToHome(const Point& home)
{
	unique_ptr<Point> base;
	// Use base as ascent target first.
	if (this->_coordinate.getZ() < this->_norm_altitude) {
		base = make_unique<Point>(this->_coordinate.getX(), this->_coordinate.getY(), this->_norm_altitude);
		this->ascent(*base, true);
		base->setPtFloat(home.getX(), home.getY(), this->_norm_altitude);
	}
	else {
		base = make_unique<Point>(home.getX(), home.getY(), this->_norm_altitude);
	}

	this->steadyFlight(*base, true);
	base->setZ(0);
	this->descent(*base, true);
}
