#include "Sensor.h"

Sensor::Sensor(size_t id, double x, double y, double z, double v, size_t prize,
	bool is_pres) : _ID(id), _coord(Point(x, y, z)), _V(v), _prize(prize)
{
	if (is_pres) {  // Pressure sensor parameters
		_T_change = 1;
		_T_reset  = 1;
		_T_idle   = 9.498;
		_C        = 3;
		_V_max    = 5;
		_V_min    = 3.5;
		_P_sense  = 3.96e-3;
		_P_idle   = 1.32e-5;
	}
	else {	// Temperature sensor parameters
		_T_change = 10;
		_T_reset  = 10;
		_T_idle   = 99.498;
		_C        = 6;
		_V_max    = 2.5;
		_V_min    = 1.75;
		_P_sense  = 8.1e-6;
		_P_idle   = 6e-9;
	}
	_E = 0.5 * _C * pow(_V, 2);
	_E_max = 0.5 * _C * pow(_V_max, 2);
}

void Sensor::consume_energy_over_time(double t, double& e)
{
	e -= floor(t / (3 * _T_reset)) * (
		_T_sense * _P_sense + _T_idle * _P_idle + _T_comm * _P_comm);
}
