/**
* @file drone.h
* Implementation of UAV (take M100 as the model).
*
* @author Qiuchen Qian <qiuchen.qian19@imperial.ac.uk>
*/

#pragma once
#include <iostream>
#include "sensor.h"
#include "wind.h"
#include "obstacle.h"

#ifndef DRONE_H
#define DRONE_H

/* @class		Drone Drone.h "Drone.h"
*  @brief		Implementation of @a Drone class
*
*  The @a Drone class includes basic attributes like Drone position, flight time
*  and energy. Some Drone actions like fly approaching to next target node through
*  GPS localization and UWB localization, return-to-home process, inductive
*  power transfer and flight simulation are included as well.
*
* @author Qiuchen Qian
* @version 5
* @date	2021
* @copyright MIT Public License
*/
class Drone
{
//! @publicsection
public:
	Drone() {};
	/* Construct Drone class with user-defined arguments.
	* @param uav_pos: UAV current coordinate.
	* @param uav_energy: UAV current energy.
	*/
	Drone(const Point& uav_pos, float uav_energy) : _coordinate(uav_pos), _energy(uav_energy) {};
	//! A default destructer which sets PDV attributes to initial state.
	~Drone() {};

	// Get UAV current coordinate.
	Point getPt() const { return _coordinate; }
	// Get UAV remain energy.
	float getE() const { return _energy; }
	// Get UAV flight time.
	float getT() const { return _time; }
	// Get UAV flight distance.
	float getD() const { return _distance; }
	// Get UAV normal flight altitude.
	float getNormAlt() const { return _norm_altitude; }
	// Get UAV hover altitude.
	float getHoverAlt() const { return _hover_altitude; }
	// Get minimum allowed number of requested sensors in single task.
	int getMinTaskNum() const { return _min_task_number; }
	// Get UAV horizontal movement velocity.
	float getVHorz() const { return _horizontal_velocity; }
	// Get UAV vertical movement velocity.
	float getVVert() const { return _vertical_velocity; }
	// Get system simulation time change frequency.
	float getFreq() const { return static_cast<float>(1 / _dt); }

	// Set UAV coordiante with `Point` object.
	void setPt(const Point& p) { _coordinate.setPt(p); }
	// Set UAV energy.
	void setE(float e) { _energy = e; }
	// Set minimum number of requested sensors in single task
	void setMinTaskNum(int val) { _min_task_number = val; }
	// Set wind information when doing UAV simulation with `Grid` object.
	void setWind(Grid* care_wind) { _dt = care_wind->dt; _wind_grid = care_wind; }
	// Set system simulation time change frequency.
	void setFreq(float freq) { _dt = 1 / static_cast<int>(freq); }
	// Reset all UAV state to inital state (normally called after return to home).
	void resetState(float uav_energy, const Point& coordinate);

	/* Calculate energy package needed to charge a sensor node. Note that J to Wh conversion is made,
	* and WPT efficiency is set to 50 %, so divide 3600.
	* Energy E = 1 / (2 * n_{rf2dc} * 3600) * C * (V_{max} - V)^2.
	*
	* @param next_sn: The next target sensor node to charge.
	* @param e_cost: Energy variable to be updated [Wh].
	* @param charged_e: Charged energy [J]. 
	*/
	void calcChrgPkg(const Sensor& next_sn, float& e_cost, float& charged_e)
	{
		charged_e = next_sn.getChrgPkg();
		e_cost = charged_e / 1800;
	}
	/* Update PDV states after vertical movement. If wind included, v_air at
	* x-axis and y-axis should be v_wind because v_pdv_ground is 0. To simplify
	* computation model, acceleration process is ignored.
	*
	* @param target: Target coordinate.
	* @param force_exe: Forcely execute energy cost update or not. If ture,
	* UAV will update energy without restriction. 
	*/
	bool ascent(const Point& target, bool force_exe);
	/* Update PDV states after vertical movement. If wind included, v_air at
	* x-axis and y-axis should be v_wind because v_pdv_ground is 0. To simplify
	* computation model, acceleration process is ignored.
	*
	* @param target: Target coordinate.
	* @param force_exe: Forcely execute energy cost update or not. If ture,
	* UAV will update energy without restriction.
	*/
	bool descent(const Point& target, bool force_exe);
	/* Update PDV states after horizontal movement. If wind included, v_air at
	* z-axis should be v_wind because v_pdv_ground is 0. To simplify
	* computation model, acceleration process is ignored.
	*
	* @param target: Target coordinate.
	* @param force_exe: Forcely execute energy cost update or not. If ture,
	* UAV will update energy without restriction.
	*/
	bool steadyFlight(const Point& target, bool force_exe);
	/* Flight strategy implementation, simulation, and UAV state update.
	*
	* @param offset: The offset to determine replaced safety coordinate.
	* @param obs_case: The collision case can happen in original flight route.
	* @param cur_obs: Current concerned obstacle.
	* @param avoid_p: The first safety coordinate (may have two safety coordinates in some cases).
	* @param target: The original target coordinate.
	*/
	bool obstacleFlight(float offset, int obs_case, Obstacle* cur_obs, 
		const Point& avoid_p, const Point& target, bool force_exe);
	// Forcely execute: ascent() to safety altitude, steadyFlight() to base station and descent().
	void returnToHome(const Point& home);

//! @protectedsection
protected:
	// Mathmatic constantsa
	float _pi = 3.141592653f;			/* Pi */
	float _g = 9.80665f;				/* Gravitational acceleration [m/s^2] */

	// UAV relatde calculation variables
	Point _coordinate = Point();		/* A Point object for UAV position [m] */
	float _energy = 0;					/* UAV remain energy [W] */
	float _time = 0;					/* UAV flight time */
	float _distance = 0;				/* UAV flight distance [m] */
	float _norm_altitude = 15;			/* PDV steady normal flight altitude [m] */
	float _hover_altitude = 1;			/* Relative UAV hovering altitude [m] */

	// M100 specification constants
	float _init_eng = 0;				/* UAV initial energy [Wh]. */
	float _horizontal_velocity = 10;	/* UAV average horizontal movement speed (GPS navigation) [m/s] */
	float _vertical_velocity = 3;		/* UAV average vertical movement speed [m/s] */
	float _mass = 3.107f;				/* M100 mass with battery only [kg] */ 
	// M100 propeller swept out area [m^2]. Area A = Pi * r^2.
	float _area_propeller = _pi * 0.148f * 0.148f; 
	// M100 X- or Y-axis cross section area for horizontal wind [m^2]. 
	// 0.218 * 0.701
	float _pdv_area_horz = 0.153f;
	// M100 Z-axis cross section area for vertical wind [m^2]. 
	// radius is 0.498 m
	float _pdv_area_vert = _pi * 0.498f * 0.498f;

	// Areodynamics physics constants
	float _rho_air = 1.25f;				/* Air density [kg/m^3] */
	float _c_drag = 0.04f;				/* Drag coefficient [dimensionless] */
	float _wind_force_const_horz = _rho_air * _pdv_area_horz;
	float _wind_force_const_vert = _rho_air * _pdv_area_vert;
	float _prop_drag_const = 0.5f * _rho_air * _c_drag * _area_propeller;
	float _horz_drag_const = 0.5f * _rho_air * _c_drag * _pdv_area_horz;
	float _vert_drag_const = 0.5f * _rho_air * _c_drag * _pdv_area_vert;

	// System Hyper-parameters
	int _dt = 10;						/* Sampling time duartion [s] */
	int _min_task_number = 10;			/* Minimum number of sensor nodes to charge */
	bool _inc_obs = false;				/* Include obstacle data or not. */
	Grid* _wind_grid = nullptr;			/* Pointer of `Grid` to store wind data. */

	/* Calculate component of v_zz at XY-plane, with signed value, we can know flight angle.
	* @param target: The target point coordinate
	* @param vec_x: x-axis component of the movement vector 
	* @param vec_y: y_axis component of the movement vector
	* @param angle: The calculated angle 
	*/
	void _calc2DPlaneVec(const Point& target, float& vec_x, float& vec_y, float& angle);
};

#endif // !DRONE_H