#pragma once
#include "Point.h"

#ifndef SENSOR_H
#define SENSOR_H

class Sensor
{
public:
	Sensor() {}
	Sensor(size_t id, double x, double y, double z) : 
		_ID(id), _coord(Point(x, y, z)) {}
	Sensor(size_t id, double x, double y, double z, 
		double v, size_t prize, bool is_pres);
	~Sensor() {}

	size_t get_id() const { return _ID; }
	Point get_pt() const { return _coord; }
	size_t get_prize() const { return _prize; }
	double get_volt() const { return _V; }
	double get_volt_min() const { return _V_min; }
	double get_energy() const { return _E; }
	double get_volt_by_E_and_C() const { return sqrt(2 * _E / _C); }
	double get_charge_package() const { return _E_max - _E; }
	void get_chrg_pkg(double& e_cost, double& charged_e) const
	{
		charged_e = _E_max - _E;
		e_cost = charged_e / 1800;
	}

	void consume_energy_over_time(double t, double& e);

private:
	// Model related variables
	size_t _ID    = 0;			// Sensor node ID
	Point _coord  = Point();	// Sensor node coordinate [m]
	size_t _prize = 0;			// Prize weight, ranging from [6, 10]
	// Energy related variables
	double _C       = 0;		// Capacitance [F]
	double _E       = 0;		// Energy of the super capacitor [J]
	double _E_max   = 0;		// Sensor maximum energy  [J]
	double _V       = 0;		// Voltage of the super capacitor [V]
	double _V_max   = 0;		// Maximum voltage of super capacitor [V]
	double _V_min   = 0;		// Minimum voltage of super capacitor[V]
	double _P_sense = 0;		// Average power when in sensing cycle [W]
	double _P_idle  = 0;		// Average power when in idle cycle [W]
	double _P_comm  = 2.45e-3;	// Average power when in communication cycle [W]
	// Time related variables
	double _T_sense  = 2e-3;	// Sense cycle duration [s]
	double _T_comm   = 0.5;		// Communication cycle duration [s]
	double _T_idle   = 0;		// Idle cycle duration [s]
	double _T_reset  = 0;		// Sensing cycle to reset [s]
	double _T_change = 0;		// Sense cycle duration [s]
};

#endif // !SENSOR_H

