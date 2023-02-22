#pragma once
#include <fstream>
#include <string>
#include <stdexcept>
#include <filesystem>
#include <memory>
#include <ctime>
#include "Wind.h"
#include "Sensor.h"
#include "Star.h"

#ifndef DATAIO_H
#define DATAIO_H

using std::string;
namespace fs = std::filesystem;
typedef std::pair<size_t, double> IdxVal;

class DataIO
{
public:
	DataIO() {}
	DataIO(const fs::path& inp_dir, const fs::path& out_dir) :
		_inp_dir(inp_dir), _out_dir(out_dir) {}
	~DataIO() {}

	void read_wind_grid(const string grid_fname, const string wind_fname,
		int start_t, int end_t, std::unique_ptr<WindGrid>& wind_grid);
	void read_sensors(const string sn_fname, double& init_E,
		std::vector<Sensor>& sn, double& all_E);
	void read_prev_simu_res(const string simu_fname, size_t num_nodes,  
		std::vector<size_t>& path_time, double& start_T, double& end_T);
	void save_simu_res(const string simu_fname, const string summary_fname,
		const Star& bh, const std::time_t end_time, double all_re_E, double all_de_E);
	void save_exe_time(const string time_fname, const std::time_t end_time,
		double comp_time);

private:
	fs::path _inp_dir;
	fs::path _out_dir;
};

#endif // !DATAIO_H
