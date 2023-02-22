#include "mpi.h"
#include "bha.h"

using namespace std;

// Some global variables
fs::path inp_dir = fs::current_path();
fs::path out_dir = fs::current_path();
float all_nodes_energy = 0, exe_tot_time = 0;
int init_sol_len = 0;
vector<int> initial_solution, alg_solution;
vector<Sensor> initial_nodes;

// BHA Hyper-parameters
string node_file_name = "nodes.csv";
string sol_file_name = "init_sol.txt";
bool inc_obs = false;
int eng_wt = 50;
int pop = 50;
int gen = 50;
int ap = 50;
int search_num = 5;
float target_metric = 100.3f;
float target_time = 1.f;
double alg_time = 0;

float compute_avg(float* array, int num) 
{
	float sum = 0.f;
	for (int i = 0; i < num; i++) sum += array[i];
	return sum / num;
}

void find_maximum(vector<float> array, int num, int& max_idx, float& max_val)
{
	max_idx = 0;
	max_val = array[max_idx];

	for (int i = 1; i < num; i++) {
		if (array[i] > max_val) { max_val = array[i]; max_idx = i; }
	}
}

void readInitSols(const string& sol_fname, int& sol_len, vector<int>& init_sol)
{
	// Initialization
	if (init_sol.size()) init_sol.clear();

	fs::path fname = inp_dir;
	fname /= sol_fname;
	ifstream file(fname);
	if (!file) throw runtime_error(sol_fname + " cannot open.\n");
	// Second line includes solution generated by initial solver 
	// (excluded start and end index).
	string line;
	getline(file, line);
	stringstream ss2(line);
	for (int i; ss2 >> i;) {
		init_sol.push_back(i);
		if (ss2.peek() == ',') ss2.ignore();
	}
	file.close();
	sol_len = static_cast<int>(init_sol.size());
}

void readInitNodes(const string& node_fname, float& eng_, 
	Point& start_, Point& end_, vector<Sensor>& init_nodes, float& all_e)
{
	// Initialization
	all_e = 0;
	if (init_nodes.size()) init_nodes.clear();

	fs::path fname = inp_dir;
	fname /= node_fname;
	ifstream file(fname);
	if (!file) throw runtime_error(node_fname + " cannot open.\n");
	// first line is column name.
	string line;
	getline(file, line);
	// Other lines include sensor node attributes.
	int cnt = 0; 
	string id, xs, ys, zs, flags, volts, weights;
	while (getline(file, id, ',')) {
		getline(file, xs, ',');
		getline(file, ys, ',');
		getline(file, zs, ',');
		getline(file, flags, ',');
		getline(file, volts, ',');
		getline(file, weights);

		if (cnt == 0 && stoi(id) == -1) {
			start_.setPtFloat(stof(xs) / 100, stof(ys) / 100, stof(zs) / 100);
			eng_ = stof(volts);
		}
		else if (cnt > 0 && stoi(id) != -1) {
			init_nodes.push_back(Sensor(stoi(id),
				stof(xs) / 100, stof(ys) / 100, stof(zs) / 100,
				stof(volts), stoi(weights), stoi(flags)));
			all_e += init_nodes[cnt - 1].getChrgPkg();
		}
		else if (stoi(id) == -1) end_.setPtFloat(stof(xs) / 100, stof(ys) / 100, stof(zs) / 100);
		else throw runtime_error("Input node data error.\n");
		cnt++;
	}
	file.close();
}

void savePlan(time_t end_time, const vector<int>& final_sol, 
	float init_m, float final_m, float init_de, float final_de, float all_de, 
	float init_re, float final_re, float all_re)
{
	fs::path plan_fname = out_dir;
	plan_fname /= "plan.txt";
	ofstream ofile(plan_fname, ios::app);
	if (!ofile) throw runtime_error("plan.txt cannot open.\n");

	ofile << end_time << ",";
	for (unsigned i = 0; i < final_sol.size(); i++) {
		ofile << final_sol[i];
		if (i != final_sol.size() - 1) ofile << ",";
		else ofile << "\n";
	}
	ofile.close();

	fs::path sum_fname = out_dir;
	sum_fname /= "summary.csv";
	ofstream sumfile(sum_fname, ios::app);
	if (!sumfile) throw runtime_error("summary.csv cannot open.\n");
	
	if (final_de == 0 && final_re == 0) {
		sumfile << end_time << ",-1,-1,-1,-1,-1,-1,-1,-1,-1\n";
		sumfile.close();
		return;
	}
	sumfile << end_time << "," << init_m << "," << final_m << "," 
		<< init_de << "," << final_de << "," << all_de << ","
		<< init_re << "," << final_re << "," << all_re << ","
		<< final_re / (3600 * final_de) << "\n";
	sumfile.close();
}

void saveTime(time_t end_time, float avg_time, float tot_time) 
{
	fs::path time_fname = out_dir;
	time_fname /= "bha_time.txt";
	ofstream ofile(time_fname, ios::app);
	if (!ofile) throw runtime_error("bha_time.txt cannot open.\n");

	ofile << end_time << "," << avg_time << "," << tot_time << "\n";
	ofile.close();
}

void saveCompTime(time_t end_time, float comp_metric, double comp_time)
{
	fs::path time_fname = out_dir;
	time_fname /= "comp_time.txt";
	ofstream ofile(time_fname, ios::app);
	if (!ofile) throw runtime_error("comp_time.txt cannot open.\n");

	ofile << end_time << "," << comp_metric << "," << comp_time << "\n";
	ofile.close();
}

void checkInputParams(int total_node_num)
{
	if (eng_wt < 0 || eng_wt > 100) {
		throw std::out_of_range(
			"Recharged energy weight is out of range [0, 100].\n");
	}
	// if (pop < 1 || pop > 1000) {
	// 	throw std::out_of_range("Population number is out of range [1, 200].\n");
	// }
	// if (gen < 1 || gen > 5000) {
	// 	throw std::out_of_range("Generation number is out of range [1, 200].\n");
	// }
	if (ap < 0 || ap > 100) {
		throw std::out_of_range(
			"Attract probability is out of range [0, 100].\n");
	}
	if (search_num < 1 || search_num > total_node_num) {
		throw std::out_of_range("Search number is out of range [1, " 
			+ to_string(total_node_num) + "].\n");
	}
}

void saveSimuRes(time_t end_time, const vector<int>& path, const vector<float>& path_eng, 
	const vector<float>& path_time, const vector<int>& path_prize)
{
	string simu_res_fname = (out_dir / "simu_res.txt").string();
	ofstream ofile(simu_res_fname, ios::out);
	if (!ofile) throw runtime_error(simu_res_fname + " cannot open.\n");
	ofile << end_time << "\n";
	for (size_t i = 0; i < path.size(); i++) {
		if (i != path.size() - 1) { ofile << path[i] << ","; }
		else { ofile << path[i] << "\n"; }
	}
	for (size_t i = 0; i < path_eng.size(); i++) {
		if (i != path_eng.size() - 1) { ofile << path_eng[i] << ","; }
		else { ofile << path_eng[i] << "\n"; }
	}
	for (size_t i = 0; i < path_time.size(); i++) {
		if (i != path_time.size() - 1) { ofile << path_time[i] << ","; }
		else { ofile << path_time[i] << "\n"; }
	}
	for (size_t i = 0; i < path_prize.size(); i++) {
		if (i != path_prize.size() - 1) { ofile << path_prize[i] << ","; }
		else { ofile << path_prize[i] << "\n"; }
	}
}

int main(int argc, char* argv[]) 
{
	MPI_Init(&argc, &argv);
	int root_rank = 0, world_rank = 0;
	MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
	int world_size = 0;
	MPI_Comm_size(MPI_COMM_WORLD, &world_size);
	if (world_size < 1 || world_size > 4) {
		cerr << "Maximum process number 4.\n";
		MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE); 
	};

	time_t task_time_id = chrono::system_clock::to_time_t(
		chrono::system_clock::now());
	// Receive input parameters
	inp_dir = argv[1];
	out_dir = argv[2];
	node_file_name = argv[3];	
	sol_file_name = argv[4];
	inc_obs = stoi(argv[5]);
	eng_wt = stoi(argv[6]);
	pop = stoi(argv[7]);
	gen = stoi(argv[8]);
	ap = stoi(argv[9]);
	search_num = stoi(argv[10]);
	target_metric = stof(argv[11]);
	target_time = stof(argv[12]) - 0.1f;

	// Read node information.
	unique_ptr<Point> uav_start(new Point());
	unique_ptr<Point> uav_end(new Point());	
	float uav_eng = 0;
	try { 
		readInitNodes(node_file_name, uav_eng, *uav_start, *uav_end, 
			initial_nodes, all_nodes_energy); 
	}
	catch (runtime_error& e) { 
		cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
	}
	
	// Read initial solution.
	try { readInitSols(sol_file_name, init_sol_len, initial_solution); }
	catch (runtime_error& e) { 
		cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
	}

	// Read wind information.
	Grid* temp_wind = new Grid();
	try { 
		fs::path grid_fname = inp_dir;
		grid_fname /= "grid_info.txt";
		temp_wind->readGridInfo(grid_fname);
	}
	catch (runtime_error& e) { 
		cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
	}
	try {
		fs::path wind_fname = inp_dir;
		wind_fname /= "wind_vector.csv";
		temp_wind->readWindData(0, 2400, wind_fname);
	}
	catch (exception& e) { 
		cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
	}

	// Read obstacle information.
	Obstacle* temp_obs = new Obstacle();
	if (inc_obs) {
		try {
			fs::path obs_fname = inp_dir;
			obs_fname /= "obs_info.txt";
			temp_obs->readObsInfo(obs_fname);
		}
		catch (runtime_error& e) { 
			cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
		}
	}

	// Initialize BHA, pass parameters and finish serial computing.
	unique_ptr<Bha> temp_bha;
	vector<int> proc_sol(init_sol_len);
	float proc_metric = 0, proc_de = 0, proc_re = 0;
	float init_metric = 0, init_de = 0, init_re = 0; 
	temp_bha = make_unique<Bha>(eng_wt, pop, gen, ap, search_num); 
	temp_bha->setInitSol(initial_solution);
	temp_bha->setInitE(uav_eng);
	temp_bha->setInitPt(uav_start);
	temp_bha->setEndPt(uav_end);
	temp_bha->setWindGrid(temp_wind);
	if (inc_obs) temp_bha->setObstacle(temp_obs);

	// If no enough node to evolve.
	if (initial_solution.size() <= search_num) {
		if (world_rank == root_rank) {
			cout << "BHA: In " << node_file_name 
				<< ", no enough nodes to evolve. Follow initial solution.\n";
			cout.flush();
			temp_bha->fitness(initial_nodes, initial_solution, all_nodes_energy, 
				init_metric, init_de, init_re);
			try {
				savePlan(task_time_id, initial_solution, init_metric, init_metric,
					init_de, init_de, uav_eng, 
					init_re, init_re, all_nodes_energy);
				vector<float> path_eng; vector<float> path_time; vector<int> path_prize;
				temp_bha->getBhEstEngTimePrize(initial_nodes, initial_solution, path_eng, path_time, path_prize);
				saveSimuRes(task_time_id, initial_solution, path_eng, path_time, path_prize);	
				saveCompTime(task_time_id, init_metric, 0);
			}
			catch (runtime_error& e) {
				cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
			}
		}

		MPI_Finalize();
		return EXIT_SUCCESS;
	}

	try { checkInputParams(static_cast<int>(initial_nodes.size())); }
	catch (out_of_range& e) {
		cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
	}

	// BHA main calculation process
	vector<float> world_metrics;
	vector<int> last_bh_sol(init_sol_len);
	vector<int> better_times;
	float last_bh_metric = 0, last_bh_de = 0, last_bh_re = 0;

	MPI_Barrier(MPI_COMM_WORLD);
	double start_time = MPI_Wtime();
	temp_bha->initialization(initial_nodes, all_nodes_energy, 
		init_metric, init_de, init_re);
	// better_times.push_back(temp_bha->checkBhMet());
	// Gather all metrics and output file.
	for (int i = 0; i < gen; i++) {
		temp_bha->singleGenEvolve(initial_nodes, all_nodes_energy);
		temp_bha->getBhMet(proc_metric);
		// better_times.push_back(temp_bha->checkBhMet());

		// if (world_rank == root_rank) {
		// 	alg_time = MPI_Wtime() - start_time;
		// 	if (alg_time >= target_time && target_time > 0.1) {
		// 		saveCompTime(task_time_id, proc_metric, alg_time);
		// 		delete temp_wind;
		// 		delete temp_obs;
		// 		MPI_Abort(MPI_COMM_WORLD, 100);
		// 	}
		// }

		if (proc_metric >= target_metric) {
			alg_time = MPI_Wtime() - start_time;
			saveCompTime(task_time_id, proc_metric, alg_time);
			delete temp_wind;
			delete temp_obs;
			MPI_Abort(MPI_COMM_WORLD, 200);
		}
		if (world_size > 1 && ((i + 1) % 10 == 0 || i == gen - 1)) {
			int best_rank = 0;
			float best_metric = 0;
			world_metrics.resize(world_size);

			MPI_Allgather(&proc_metric, 1, MPI_FLOAT, world_metrics.data(), 1,
				MPI_FLOAT, MPI_COMM_WORLD);
			// Obtain the best black hole solution from that best process.
			find_maximum(world_metrics, world_size, best_rank, best_metric);
			if (i != gen - 1) {
				if (world_rank == best_rank) temp_bha->getBhSol(proc_sol);
				MPI_Bcast(proc_sol.data(), init_sol_len, MPI_INT, 
					best_rank, MPI_COMM_WORLD);

				// Update New black hole algorithm from the best process.
				if (world_rank != best_rank && proc_metric != best_metric) {
					temp_bha->updateBhFromProc(best_metric, proc_sol);
				}
			}
			else {
				if (best_rank == root_rank) {
					if (world_rank == root_rank) {
						temp_bha->getAllBhRes(proc_sol, proc_metric, proc_de, proc_re);
						last_bh_sol = proc_sol;
						last_bh_metric = proc_metric;
						last_bh_de = proc_de;
						last_bh_re = proc_re;
					}
				}
				else {
					if (world_rank == best_rank) {
						temp_bha->getAllBhRes(proc_sol, proc_metric, proc_de, proc_re);
						MPI_Send(proc_sol.data(), init_sol_len, MPI_INT,
							root_rank, 0, MPI_COMM_WORLD);
						MPI_Send(&proc_metric, 1, MPI_FLOAT,
							root_rank, 1, MPI_COMM_WORLD);
						MPI_Send(&proc_de, 1, MPI_FLOAT,
							root_rank, 2, MPI_COMM_WORLD);
						MPI_Send(&proc_re, 1, MPI_FLOAT,
							root_rank, 3, MPI_COMM_WORLD);
					}
					else if (world_rank == root_rank) {
						MPI_Recv(proc_sol.data(), init_sol_len, MPI_INT, best_rank,
							0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						MPI_Recv(&proc_metric, 1, MPI_FLOAT, best_rank,
							1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						MPI_Recv(&proc_de, 1, MPI_FLOAT, best_rank,
							2, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						MPI_Recv(&proc_re, 1, MPI_FLOAT, best_rank,
							3, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
						last_bh_sol = proc_sol;
						last_bh_metric = proc_metric;
						last_bh_de = proc_de;
						last_bh_re = proc_re;
					}
				}
			}
		}
	}
	alg_time = MPI_Wtime() - start_time;
	MPI_Barrier(MPI_COMM_WORLD);

	if (world_rank == root_rank) {
		// Save execution result
		try {
			if (world_size == 1) {
				temp_bha->getAllBhRes(proc_sol, proc_metric, proc_de, proc_re);
				last_bh_metric = proc_metric;
				savePlan(task_time_id, proc_sol, init_metric, proc_metric,
				init_de, proc_de, uav_eng, 
				init_re, proc_re, all_nodes_energy);
				vector<float> path_eng; vector<float> path_time; vector<int> path_prize;
				temp_bha->getBhEstEngTimePrize(initial_nodes, proc_sol, path_eng, path_time, path_prize);
				saveSimuRes(task_time_id, proc_sol, path_eng, path_time, path_prize);		
			}
			else{
				savePlan(task_time_id, last_bh_sol, init_metric, last_bh_metric,
				init_de, last_bh_de, uav_eng, 
				init_re, last_bh_re, all_nodes_energy);
				vector<float> path_eng; vector<float> path_time; vector<int> path_prize;
				temp_bha->getBhEstEngTimePrize(initial_nodes, last_bh_sol, path_eng, path_time, path_prize);
				saveSimuRes(task_time_id, last_bh_sol, path_eng, path_time, path_prize);	
			}
			saveCompTime(task_time_id, last_bh_metric, alg_time);
		}
		catch (runtime_error& e) {
			cerr << e.what(); MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
		}
	}

	delete temp_wind;
	delete temp_obs;

	MPI_Finalize();
	return EXIT_SUCCESS;
}
