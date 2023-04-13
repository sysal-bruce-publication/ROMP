/* @file sensor.cpp
*
*  Copyright (C) Qiuchen Qian, 2021
*  Imperial College, London
*/

#include "sensor.h"

Sensor::Sensor(int n_id, float x, float y, float z, float v, int wt, bool type)
{
	// Common public member variables
	this->_id = n_id;
	this->_coordinate = Point(x, y, z);
	this->_voltage = v;
	this->_weight = wt;
	this->_type = type;

	if (type) {
		//! Pressure sensor parameters
		this->_t_change = 1;
		this->_t_reset = 1;
		this->_t_idle = 9.498f;

		this->_capacitance = 3;
		this->_voltage_max = 5;
		this->_voltage_min = 3.5f;
		this->_power_sense = 3.96e-3f;
		this->_power_idle = 1.32e-5f;
	}
	else {
		//! Temperature sensor parameters
		this->_t_change = 10;
		this->_t_reset = 10;
		this->_t_idle = 99.498f;

		this->_capacitance = 6;
		this->_voltage_max = 2.5f;
		this->_voltage_min = 1.75f;
		this->_power_sense = 8.1e-6f;
		this->_power_idle = 6e-9f;
	}

	this->_energy = 0.5f * this->_capacitance * this->_voltage * this->_voltage;
	this->_energy_max = 0.5f * this->_capacitance * this->_voltage_max * this->_voltage_max;
}

void Sensor::calcE(float t, float& e)
{
	int cycles = static_cast<int>(floor(t / (3 * this->_t_reset)));

	e -= (cycles * this->_t_sense * this->_power_sense + cycles * this->_t_idle * this->_power_idle
		+ cycles * this->_t_comm * this->_power_comm);
}

void Sensor::calcWt(float v, int& wt)
{
	if (v < 0 || v > this->_voltage_max) throw std::invalid_argument("Sensor voltage out of range.");
	float low_div = this->_voltage_min / 5;
	float high_div = (this->_voltage_max - this->_voltage_min) / 5;

	if (v >= this->_voltage_max - high_div) wt = 1;
	else if (v > low_div && v <= this->_voltage_min) wt = 10 - static_cast<int>(floor(v / low_div));
	else wt = 10;
}