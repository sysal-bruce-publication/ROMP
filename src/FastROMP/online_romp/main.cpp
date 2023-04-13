#include <iostream>
#include <chrono>
#include "mpi.h"
#include "DataIO.h"
#include "BlackHole.h"
#include <stdint.h>
#include <limits.h>

#if SIZE_MAX == UCHAR_MAX
#define MPI_SIZE_T MPI_UNSIGNED_CHAR
#elif SIZE_MAX == USHRT_MAX
#define MPI_SIZE_T MPI_UNSIGNED_SHORT
#elif SIZE_MAX == UINT_MAX
#define MPI_SIZE_T MPI_UNSIGNED
#elif SIZE_MAX == ULONG_MAX
#define MPI_SIZE_T MPI_UNSIGNED_LONG
#elif SIZE_MAX == ULLONG_MAX
#define MPI_SIZE_T MPI_UNSIGNED_LONG_LONG
#else
#error "what is happening here?"
#endif

namespace ch = std::chrono;
using std::make_unique, std::unique_ptr;
using std::stoi, std::stod;
using std::vector;

void find_maximum(const vector<double>& array, size_t num_proc, 
	size_t& max_idx, double& max_val)
{
	max_idx = 0;
	max_val = array[max_idx];

	for (size_t i = 1; i < num_proc; i++) {
		if (array[i] > max_val) { max_val = array[i]; max_idx = i; }
	}
}

int main(int argc, char* argv[])
{
	time_t task_time_id = ch::system_clock::to_time_t(ch::system_clock::now());

	MPI_Init(&argc, &argv);
	int root_rank = 0, world_rank = 0;
	MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
	int world_size = 0;
	MPI_Comm_size(MPI_COMM_WORLD, &world_size);
	if (world_size < 1 || world_size > 4) {
		std::cerr << "Maximum process number 4.\n";
		MPI_Abort(MPI_COMM_WORLD, EXIT_FAILURE);
	};

	// fs::path inp_dir = fs::current_path() /= "inp";
	// fs::path out_dir = fs::current_path() /= "out";
	//string node_fname = "re_iter1.csv";
	//string prev_simu_fname = "re_simu_res1.txt";
	//string curr_simu_fname = "re_simu_res2.txt";
	//string summary_fname = "summary.csv";
	//size_t energy_weight = 50;
	//size_t num_pops = 10;
	//size_t num_gens = 10;
	//size_t num_srch = 10;
	//double attr_prob = 0.75;

	fs::path inp_dir = argv[1];
	fs::path out_dir = argv[2];
	string node_fname = argv[3];
	string prev_simu_fname = argv[4];
	string curr_simu_fname = argv[5];
	string summary_fname = "summary.csv";
	string time_fname = "comp_time.txt";
	size_t energy_weight = stoi(argv[6]);
	size_t num_pops = stoi(argv[7]);
	size_t num_gens = stoi(argv[8]);
	size_t num_srch = stoi(argv[9]);
	double attr_prob = stod(argv[10]);

	// Read sensor node file
	unique_ptr<DataIO> io_mngr = make_unique<DataIO>(inp_dir, out_dir);
	double init_uav_energy = 0, all_rechargeable_energy = 0;
	std::vector<Sensor> sensors;
	io_mngr->read_sensors(
		node_fname, init_uav_energy, sensors, all_rechargeable_energy);
	// Previous simulated path
	std::vector<size_t> prev_path;
	double start_time = 0, end_time = 0;
	io_mngr->read_prev_simu_res(
		prev_simu_fname, sensors.size(), prev_path, start_time, end_time);
	// Read wind grid
	unique_ptr<WindGrid> wind_grid;
	io_mngr->read_wind_grid("grid_info.txt", "wind_vector.csv",
		static_cast<int>(std::max(start_time - 10, 0.)),
		static_cast<int>(end_time + 300), wind_grid);
	// Declare BHA instance
	unique_ptr<BlackHole> bha = make_unique<BlackHole>(
		num_pops, num_srch, energy_weight, attr_prob, init_uav_energy, start_time);
	bha->set_sensors(sensors);
	// Declare MPI process and world metrics.
	double proc_metric = 0, proc_dis_E = 0, proc_re_E = 0;
	vector<size_t> proc_sol;
	vector<double> world_metrics, world_dis_E, world_re_E;
	vector<size_t> world_best_bh_sol;
	double world_best_metric = 0, world_best_dis_E = 0, world_best_re_E = 0;
	MPI_Barrier(MPI_COMM_WORLD);
	double alg_start_t = MPI_Wtime();
	// Compute the readjusted plan
	double init_metric = 0, init_discharged_energy = 0, init_recharged_energy = 0;
	bha->check_prev_path(*wind_grid, prev_path, start_time, all_rechargeable_energy,
		init_metric, init_discharged_energy, init_recharged_energy);
	bha->drop_node_until_feasible(*wind_grid);
	bha->add_node_until_budget(*wind_grid);
	if (bha->get_num_nodes() - 2 <= num_srch) {
		if (world_rank == root_rank) {
			double alg_time = MPI_Wtime() - alg_start_t;
			std::cout << "No enough nodes for BHA.\n";
			io_mngr->save_simu_res(curr_simu_fname, summary_fname, bha->get_bh(),
			task_time_id, all_rechargeable_energy, init_uav_energy);
			io_mngr->save_exe_time(time_fname, task_time_id, alg_time);
		}
		MPI_Finalize();
		return EXIT_SUCCESS;
	}

	// BHA main calculation process
	bha->initialize_population_solutions(*wind_grid, all_rechargeable_energy);
	for (size_t i = 0; i < num_gens; i++) {
		bha->single_generation_evolvtion(*wind_grid, all_rechargeable_energy);
		if (world_size > 1 && ((i + 1) % 10 == 0 || i == num_gens - 1)) {
			// Exchange the BH metric between each process.
			proc_metric = bha->get_bh_metric();
			world_metrics.resize(world_size);
			MPI_Allgather(&proc_metric, 1, MPI_DOUBLE, world_metrics.data(), 1,
				MPI_DOUBLE, MPI_COMM_WORLD);
			// Exchange the BH discharged energy between each process.
			proc_dis_E = bha->get_bh_dis_E();
			world_dis_E.resize(world_size);
			MPI_Allgather(&proc_dis_E, 1, MPI_DOUBLE, world_dis_E.data(), 1,
				MPI_DOUBLE, MPI_COMM_WORLD);
			// Exchange the BH recharged energy between each process.
			proc_re_E = bha->get_bh_re_E();
			world_re_E.resize(world_size);
			MPI_Allgather(&proc_re_E, 1, MPI_DOUBLE, world_re_E.data(), 1,
				MPI_DOUBLE, MPI_COMM_WORLD);
			// Find the process id with the best metric
			size_t best_rank = 0; double best_metric = 0;
			find_maximum(world_metrics, world_size, best_rank, best_metric);
			if (world_rank == best_rank) { proc_sol = bha->get_bh_sol(); }
			else {
				if (!proc_sol.empty()) proc_sol.clear();
				proc_sol.resize(bha->get_bh_sol().size());
			}
			MPI_Bcast(proc_sol.data(), proc_sol.size(), 
				MPI_SIZE_T, best_rank, MPI_COMM_WORLD);
			// Update new black hole from best behaved process.
			if (world_rank != best_rank && proc_metric != best_metric && proc_dis_E <= init_uav_energy) {
				bha->set_bh_from_proc(
					proc_sol, proc_metric, proc_dis_E, proc_re_E);
			}
		}
	}
	// Lastly, synchronize the solution path to black hole
	bha->synchronize_black_hole(*wind_grid, start_time, all_rechargeable_energy,
		proc_metric);
	double alg_time = MPI_Wtime() - alg_start_t;
	world_metrics.resize(world_size);
	MPI_Allgather(&proc_metric, 1, MPI_DOUBLE, world_metrics.data(), 1,
		MPI_DOUBLE, MPI_COMM_WORLD);
	size_t best_rank = 0; double best_metric = 0;
	find_maximum(world_metrics, world_size, best_rank, best_metric);
	if (world_rank == best_rank) {
		// We only want the root rank to save the world-best bh, so if
		// the root rank is the best one, then we can directly save bh data.
		io_mngr->save_simu_res(curr_simu_fname, summary_fname, bha->get_bh(),
			task_time_id, all_rechargeable_energy, init_uav_energy);
		io_mngr->save_exe_time(time_fname, task_time_id, alg_time);
	}
	MPI_Finalize();
	return EXIT_SUCCESS;
}
                                                                             