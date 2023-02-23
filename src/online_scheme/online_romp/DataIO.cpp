#include "DataIO.h"

using std::ifstream, std::ofstream;
using std::runtime_error, std::invalid_argument;
using std::getline;
using std::stringstream, std::to_string;
using std::vector;

void DataIO::read_wind_grid(const string grid_fname, const string wind_fname,
	int start_t, int end_t, std::unique_ptr<WindGrid>& wind_grid)
{
	if (end_t == 0 || end_t <= start_t) {
		throw invalid_argument("start time: " + to_string(start_t) 
			+ "end time: " + to_string(end_t) + ".\n");
	}
	string grid_fdir = (_inp_dir / grid_fname).string();
	ifstream grid_f(grid_fdir);
	if (!grid_f) { throw runtime_error(grid_fdir + " cannot open."); }

	double x0 = -1, y0 = -1, z0 = -1, t0 = 0;
	size_t cnt = 0;
	Point start_pos;
	int dx = 0, dy = 0, dz = 0, dt = 0;
	int num_xs = 0, num_ys = 0, num_zs = 0, num_ts = 0;
	double start_time = 0, end_time = 0;
	string line;
	while (getline(grid_f, line)) {
		stringstream ss(line);
		for (int i; ss >> i;) {
			if (!cnt) { x0 = i / 100.; }
			else if (cnt == 1) { y0 = i / 100.; }
			else if (cnt == 2 && x0 != -1 && y0 != -1) { z0 = i / 100.; }
			else if (cnt == 3) { t0 = i / 1000.; }
			else if (cnt == 4) { dx = i / 100; }
			else if (cnt == 5) { dy = i / 100; }
			else if (cnt == 6) { dz = i / 100; }
			else if (cnt == 7) { dt = i / 1000; }
			else if (cnt == 8) { num_xs = i; }
			else if (cnt == 9) { num_ys = i; }
			else if (cnt == 10) { num_zs = i; }
			else if (cnt == 11) { num_ts = i; }

			if (ss.peek() == ',') { ss.ignore(); }
			cnt++;
		}
	}
	grid_f.close();
	// Handle wind vector
	start_t -= start_t % dt;
	end_t -= end_t % dt;
	int read_num_ts = static_cast<int>(floor((end_t - start_t) / dt)) + 1;
	int expected_size = num_xs * num_ys * num_zs * read_num_ts;
	vector<WindVec> wind_data; wind_data.reserve(expected_size);
	string wind_fdir = (_inp_dir / wind_fname).string();
	ifstream wind_f(wind_fdir);
	if (!wind_f) { throw runtime_error(wind_fdir + " cannot open."); }
	string first_line; 
	getline(wind_f, first_line);  // remove first line
	string t, uv_x, uv_y, uv_z;

	double measure_start_t = start_t * 1000;
	double measure_end_t = end_t * 1000;
	
	while (getline(wind_f, t, ',')) {
		getline(wind_f, uv_x, ',');
		getline(wind_f, uv_y, ',');
		getline(wind_f, uv_z);

		if (stoi(t) < measure_start_t) continue;
		if (stoi(t) > measure_end_t) break;
		wind_data.push_back(WindVec(stod(t) / 1000., stod(uv_x) / 100., 
			stod(uv_y) / 100., stod(uv_z) / 100.));
	}
	wind_f.close();
	if (wind_data.size() != expected_size) {
		throw runtime_error("Exact size " + to_string(wind_data.size())
			+ " not match expected size " + to_string(expected_size));
	}

	wind_grid = std::make_unique<WindGrid>(x0, y0, z0, num_xs, num_ys, num_zs,
		num_ts, dx, dy, dz, dt, start_t, end_t);
	wind_grid->set_wind_vec(wind_data);
}

void DataIO::read_sensors(const string sn_fname, double& init_E, 
	vector<Sensor>& sn, double& all_E)
{
	string sn_fdir = (_inp_dir / sn_fname).string();
	ifstream sn_f(sn_fdir);
	if (!sn_f) throw runtime_error(sn_fdir + " cannot open.\n");

	string line;
	getline(sn_f, line);
	size_t cnt = 0;
	string id, x, y, z, flag, volt, prize;
	if (!sn.empty()) { sn.clear(); }
	while (getline(sn_f, id, ',')) {
		getline(sn_f, x, ',');
		getline(sn_f, y, ',');
		getline(sn_f, z, ',');
		getline(sn_f, flag, ',');
		getline(sn_f, volt, ',');
		getline(sn_f, prize);

		if (cnt == 0) {
			sn.push_back(Sensor(0, stod(x) / 100, stod(y) / 100, stod(z) / 100));
			init_E = stod(volt);
		}
		else if (cnt == 1) {
			Sensor node = Sensor(1, stod(x) / 100, stod(y) / 100, stod(z) / 100);
			sn.push_back(node);
		}
		else {
			Sensor node = Sensor(cnt, stod(x) / 100, stod(y) / 100, stod(z) / 100,
				stod(volt), stoi(prize), stoi(flag));
			sn.push_back(node);
			all_E += node.get_charge_package();
		}
		cnt++;
	}
	sn_f.close();
}

void DataIO::read_prev_simu_res(const string simu_fname, size_t num_nodes, 
	std::vector<size_t>& path, double& start_T, double& end_T)
{
	string simu_fdir = (_inp_dir / simu_fname).string();
	ifstream simu_f(simu_fdir);
	if (!simu_f) throw std::runtime_error(simu_fdir + " cannot open.\n");

	string line; getline(simu_f, line);  // Skip time stamp
	// Solution index
	getline(simu_f, line);
	if (!path.empty()) { path.clear(); }
	path.reserve(num_nodes);
	stringstream ss_sol(line); 
	for (size_t i; ss_sol >> i;) {
		path.push_back(i);
		if (ss_sol.peek() == ',') { ss_sol.ignore(); }
	}
	path.shrink_to_fit();

	getline(simu_f, line);  // Simulated energy
	getline(simu_f, line);  // Simulated time
	stringstream ss_time(line); 
	vector<double> simu_time; simu_time.reserve(path.size());
	for (double i; ss_time >> i;) {
		simu_time.push_back(i);
		if (ss_time.peek() == ',') { ss_time.ignore(); }
	}
	start_T = simu_time[0] ;
	end_T = simu_time[simu_time.size() - 1];
	simu_f.close();
}

void DataIO::save_simu_res(const string simu_fname, const string summary_fname,
	const Star& bh, const std::time_t end_time, double all_re_E, double all_de_E)
{
	string simu_fdir = (_out_dir/ simu_fname).string();
	ofstream simu_f(simu_fdir, std::ios::out);
	if (!simu_f) { throw std::runtime_error(simu_fdir + " cannot open.\n"); }

	simu_f << end_time << "\n";
	for (size_t i = 0; i < bh.path_E_T.size(); i++) {
		simu_f << bh.path_E_T[i].first;
		if (i != bh.path_E_T.size() - 1) { simu_f << ","; }
		else { simu_f << "\n"; }
	}
	for (size_t i = 0; i < bh.path_E_T.size(); i++) {
		simu_f << bh.path_E_T[i].second.first;
		if (i != bh.path_E_T.size() - 1) { simu_f << ","; }
		else { simu_f << "\n"; }
	}
	for (size_t i = 0; i < bh.path_E_T.size(); i++) {
		simu_f << bh.path_E_T[i].second.second;
		if (i != bh.path_E_T.size() - 1) { simu_f << ","; }
		else { simu_f << "\n"; }
	}
	simu_f.close();

	string sum_fdir = (_out_dir / summary_fname).string();
	ofstream sum_f(sum_fdir, std::ios::app);
	if (!sum_f) { throw std::runtime_error(simu_fdir + " cannot open.\n"); }

	sum_f << end_time << "," << bh.metric << "," << bh.cost << "," << all_de_E
		<< "," << bh.prize << "," << all_re_E << ",";
	int feas = bh.feasible ? 1 : 0;
	sum_f << feas << "\n";
	sum_f.close();
}

void DataIO::save_exe_time(const string time_fname, const std::time_t end_time,
	double comp_time)
{
	string time_fdir = (_out_dir / time_fname).string();
	ofstream sum_f(time_fdir, std::ios::app);
	if (!sum_f) { throw std::runtime_error(time_fdir + " cannot open.\n"); }

	sum_f << end_time << "," << comp_time << "\n";
	sum_f.close();
}
