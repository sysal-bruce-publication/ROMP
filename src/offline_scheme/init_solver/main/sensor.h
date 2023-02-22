/**
* @file sensor.h
* Implementation of sensor nodes.
*
* @author Qiuchen Qian <qiuchen.qian19@imperial.ac.uk>
*/

#pragma once
#include <stdexcept>
#include "point.h"

#ifndef SENSOR_H
#define SENSOR_H

/*! @class		Sensor sensor.h "sensor.h"
*   @brief		Implementation of @a Sensor class
*
*   The @a Sensor class includes basic attributes like voltage, energy,
*   weight, etc. and actions like simulating _energy consumption, updating weight,
*   etc.
*
*   @author		Qiuchen Qian
*   @version	6
*   @date		2021
*   @copyright	MIT Public License
*/
class Sensor
{
	//! @publicsection
public:
	Sensor() {};
	/* Construct Sensor class with user-defined arguments.
	* @param n_id: ID number to identify sensor node in the network.
	* @param x: X-axis coordinate.
	* @param y: Y-axis coordinate.
	* @param z: Z-axis coordinate.
	* @param v: Current voltage.
	* @param wt: Corresponding prize weight.
	* @param type: Sensor type (false for temperature, true for pressure).
	*/
	Sensor(int n_id, float x, float y, float z, float v, int wt, bool type);
	~Sensor() {};

	// Get sensor node ID.
	int getID() const { return _id; }
	// Get sensor coordinate.
	Point getPt() const { return _coordinate; }
	// Get sensor current prize weight.
	int getWt() const { return _weight; }
	// Get sensor current voltage.
	float getV() const { return _voltage; }
	// Get minimum allowed voltage before requested to be recharged.
	float getVMin() const { return _voltage_min; }
	// Get sensor remain energy.
	float getE() const { return _energy; }
	// Get needed energy package to fully recharge a sensor from current volatge. 
	// Energy level change dE = 0.5 * C * (V_{max} - V_{cur})^2.
	float getChrgPkg() const { return _energy_max - _energy; }

	/* Calculate voltage according to current energy. Voltage level V = sqrt(2 * E / C).
	*
	* @param v:	Voltage to be updated [V].
	*/
	void calcV(float& v) { v = sqrtf(2 * _energy / _capacitance); }
	/* Calculate sensor energy according to current voltage. Energy level E = 0.5 * C * V^2.
	*
	* @param e: Energy to be updated [J].
	*/
	void calcE(float& e) { e = 0.5f * _capacitance * _voltage * _voltage; }
	/* Calculate sensor energy by spent time. Energy consumption E = V * I * t.
	*
	* @param t: Time spent [s].
	* @param e: Energy to be updated [J].
	*/
	void calcE(float t, float& e);
	/* Calculate prize weight by current voltage.
	*
	* @param v:	Current volatge [V].
	* @param w:	Prize weight to be updated.
	*/
	void calcWt(float v, int& wt);

	//! @privatesection
private:
	//! Main variables
	int _id = -1;					/* ID number to identify sensor node in the network. */
	Point _coordinate = Point();	/* A Point object for sensor node position [m] */
	bool _type = false;				/* 1 for pressure sensor type, 0 for temperature sensor type */
	float _capacitance = 0;			/* Capacitance [F] */
	int _weight = 0;				/* Prize weight, ranging from [6, 10] */
	float _energy = 0;				/* Energy of the super capacitor [J] */
	float _energy_max = 0;

	//! Voltage related variables
	float _voltage = 0;				/* Voltage of the super capacitor [V] */
	float _voltage_max = 0;			/* Maximum _voltage of super capacitor [V] */
	float _voltage_min = 0;			/* Minimum _voltage of super capacitor[V] */

	//! Power related constants
	float _power_sense = 0;			/* Average power when in sensing cycle [W] */
	float _power_idle = 0;			/* Average power when in idle cycle [W] */
	float _power_comm = 2.45e-3f;	/* Average power when in communication cycle [W] */

	//! Time related constants
	float _t_sense = 2e-3f;			/* Sense cycle duration [s] */
	float _t_comm = 0.5f;			/* Communication cycle duration [s] */
	float _t_idle = 0;				/* Idle cycle duration [s] */
	float _t_reset = 0;				/* Sensing cycle to reset [s] */
	float _t_change = 0;			/* Sense cycle duration [s] */
};

#endif // SENSOR_H

