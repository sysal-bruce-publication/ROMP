#include <cstdint>

#include "drone.h"
#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace std;
namespace fs = std::filesystem;

// Global variables
fs::path inp_dir;
fs::path out_dir;
vector<vector<int64_t>> dist_mat;
vector<int64_t> prizes;
int uav_capacity = 0, remain_eng_min = 0, start_idx = 0, end_idx = 0;
vector<int> initial_solution;
string node_file_name = "nodes.csv";
string sol_file_name = "init_sol.txt";
bool inc_obs = false;
float exe_tot_time = 0, exe_alg_time = 0;
bool uni_penalty = false;
bool is_rerun = true;

void readInitNodes(const string& node_fname, vector<Sensor>& init_nodes, vector<int64_t>& prizes, 
    Point& start_, Point& end_, float& eng_, float& all_e, int& prize_sum)
{
    // Initialization
    all_e = 0;
    prize_sum = 0;
    if (init_nodes.size()) init_nodes.clear();
    if (prizes.size()) prizes.clear();

    fs::path fname = inp_dir;
    fname /= node_fname;
    ifstream file(fname);
    if (!file) throw runtime_error(fname.generic_string() + " cannot open.\n");
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
            init_nodes.push_back(Sensor(-1, stof(xs) / 100, stof(ys) / 100, stof(zs) / 100, 0, 0, 0));
        }
        else if (cnt > 0 && stoi(id) != -1) {
            init_nodes.push_back(Sensor(stoi(id),
                stof(xs) / 100, stof(ys) / 100, stof(zs) / 100,
                stof(volts), stoi(weights), stoi(flags)));
            prizes.push_back(stoi(weights));
            prize_sum += stoi(weights);
            all_e += init_nodes[cnt - 1].getChrgPkg();
        }
        else if (stoi(id) == -1) {
            end_.setPtFloat(stof(xs) / 100, stof(ys) / 100, stof(zs) / 100);
            if (stof(xs) / 100 != start_.getX() && stof(ys) / 100 != start_.getY()) {
                init_nodes.push_back(Sensor(-1, stof(xs) / 100, stof(ys) / 100, stof(zs) / 100, 0, 0, 0));
                end_idx = static_cast<int>(init_nodes.size() - 1);
            }
            file.close();
            break;
        }
        else throw runtime_error("Input node data error.\n");
        cnt++;
    }
    file.close();
}

void readSimuRes(float& simu_eng, float& simu_time, int& simu_prize)
{
    string simu_fdir = (out_dir / "simu_res.txt").string();
    ifstream file(simu_fdir);
    if (!file) throw std::runtime_error(simu_fdir + " cannot open.\n");

    string line, value;
    // Simulation path energy 
    getline(file, line);
    stringstream ss(line);
    getline(ss, value, ',');
    simu_eng = stof(value);
    // Simulation path time
    getline(file, line);
    stringstream ss1(line);
    getline(ss1, value, ',');
    simu_time = stof(value);
    // Simulation path prize
    getline(file, line);
    stringstream ss2(line);
    simu_prize = 0;
    while (getline(ss2, value, ',')) {
        simu_prize += stoi(value);
    }

    file.close();
}

void calcDistMat(const vector<Sensor>& sn_list, vector<vector<int64_t>>& dist_mat)
{
    // Initialize dist_mat
    dist_mat.resize(sn_list.size());
    for (unsigned i = 0; i < dist_mat.size(); i++) dist_mat[i].resize(sn_list.size());

    for (unsigned i = 0; i < sn_list.size() - 1; i++) {
        for (unsigned j = i + 1; j < sn_list.size(); j++) {
            int64_t temp_d = round(sn_list[i].getPt().calcXYDist(sn_list[j].getPt()));
            dist_mat[i][j] = temp_d;
            dist_mat[j][i] = dist_mat[i][j];
        }
    }
}

void savePlan(const string& sol_fname, const vector<int>& final_sol)
{
    fs::path plan_fname = inp_dir;
    plan_fname /= sol_fname;
    ofstream file(plan_fname);
    if (!file) throw runtime_error(sol_fname + " cannot open.\n");

    for (unsigned i = 0; i < final_sol.size(); i++) {
        file << final_sol[i] - 1;
        if (i != final_sol.size() - 1) file << ",";
        else file << "\n";
    }
    file.close();
}

void saveTime(time_t end_time, float alg_time, float tot_time) 
{
	fs::path time_fname = out_dir;
	time_fname /= "or_time.txt";
	ofstream ofile(time_fname, ios::app);
	if (!ofile) throw runtime_error("or_time.txt cannot open.\n");

	ofile << end_time << "," << alg_time << "," << tot_time << "\n";
	ofile.close();
}

void saveExecution(int iteration, int cur_bgt, double cur_eng)
{
    string exe_fname = (out_dir / "or_exe.txt").string();
    ofstream ofile (exe_fname, ios::app);
    if (!ofile) throw runtime_error("or_time.txt cannot open.\n");
    ofile << iteration << "," << cur_bgt << "," << cur_eng << "\n";
    ofile.close();
}

float checkTask(const vector<Sensor>& sn_list, const vector<int>& sol, 
    const Point& end_pt, Drone temp_pdv)
{
	// Create target point list from solution index list. 
	vector<Point> temp_plan;
	for (unsigned i = 0; i < sol.size(); i++) {
		temp_plan.push_back(sn_list[sol[i]].getPt());
	}

	// Create PDV, target coordinate and avoid coordinate object.
	unique_ptr<Point> avoid(new Point());
    unique_ptr<Point> temp_target(new Point(temp_pdv.getPt().getX(), 
		temp_pdv.getPt().getY(), temp_pdv.getNormAlt()));

	// UAV ascent to normal flight altitude
    temp_pdv.ascent(*temp_target, true);

	int cnt = 0;
	do {
		int this_cn = sol[cnt];
		temp_target->setX(temp_plan[0].getX());
		temp_target->setY(temp_plan[0].getY());
		temp_pdv.steadyFlight(*temp_target, true);
		temp_target->setZ(temp_pdv.getHoverAlt());
        temp_pdv.descent(*temp_target, true);

		float wpt_cost = 0, wpt_charged = 0;
		temp_pdv.calcChrgPkg(sn_list[this_cn], wpt_cost, wpt_charged);
		temp_pdv.setE(temp_pdv.getE() - wpt_cost);

		// UAV ascents to safety altitude.
		temp_target->setZ(temp_pdv.getNormAlt());
        temp_pdv.ascent(*temp_target, true);

		temp_plan.erase(temp_plan.begin());
		cnt++;
	} while (temp_plan.size());

	temp_pdv.returnToHome(end_pt);
	return temp_pdv.getE();
}

int calcSpeedUp(float cost, float budget)
{
    if (cost - budget >= 80) { return 40; }
    else if (cost - budget >= 60) { return 30; }
    else if (cost - budget >= 40) { return 24; }
    else if (cost - budget >= 20) { return 20; }
    else if (cost - budget >= 18) { return 16; }
    else if (cost - budget >= 16) { return 12; }
    else if (cost - budget >= 14) { return 8; }
    else if (cost - budget >= 12) { return 4; }
    else if (cost - budget >= 10) { return 2; }
    else if (cost - budget >= 8) { return 0; }
    else if (cost - budget >= 6) { return -2; }
    else if (cost - budget >= 4) { return -4; }
    else if (cost - budget >= 2) { return -6; }
    else if (cost == budget) { return -7; }
    else if (cost - budget < 0) { return -8; }
    else if (cost - budget <= -2) { return -9; }
}

namespace operations_research 
{
    struct DataModelTSP {
        const vector<vector<int64_t>> distance_matrix = dist_mat;
        const int num_vehicles = 1;
        const vector<RoutingIndexManager::NodeIndex> starts {
            RoutingIndexManager::NodeIndex{ start_idx }
        };
        const vector<RoutingIndexManager::NodeIndex> ends {
            RoutingIndexManager::NodeIndex{ end_idx }
        };
    };

    struct DataModelOP {
        const vector<vector<int64_t>> distance_matrix = dist_mat;
        const vector<int64_t> demands = prizes;
        const vector<int64_t> vehicle_capacities{ uav_capacity };
        const int num_vehicles = 1;
        const vector<RoutingIndexManager::NodeIndex> starts {
            RoutingIndexManager::NodeIndex{ start_idx }
        };
        const vector<RoutingIndexManager::NodeIndex> ends {
            RoutingIndexManager::NodeIndex{ end_idx }
        };
    };

    void getSolution(const RoutingIndexManager& manager, const RoutingModel& routing, 
        const Assignment& solution, vector<int>& sol) 
    {
        if (sol.size()) sol.clear();
        int64_t index = routing.Start(0);
        int cnt = 0;
        while (routing.IsEnd(index) == false) {
            if (cnt == 0) { cnt++; index = solution.Value(routing.NextVar(index)); continue; }
            int64_t node_index = manager.IndexToNode(index).value();
            sol.push_back(node_index);
            index = solution.Value(routing.NextVar(index));
        }
    }

    int calSolTSP()
    {
        DataModelTSP data;
        RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                            data.starts, data.ends);
        RoutingModel routing(manager);
        const int transit_callback_index = routing.RegisterTransitCallback(
            [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                // Convert from routing variable Index to distance matrix NodeIndex.
                auto from_node = manager.IndexToNode(from_index).value();
                auto to_node = manager.IndexToNode(to_index).value();
                return data.distance_matrix[from_node][to_node];
            });
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
        RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
        search_parameters.set_first_solution_strategy(
            FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        const Assignment* solution = routing.SolveWithParameters(search_parameters);
        if (routing.status() == 1) {
            getSolution(manager, routing, *solution, initial_solution);
            return 1;
        }
        else {
            throw runtime_error("NO SOLUTION FOUND\n");
        }

    }

    int calSolOP() 
    {
        DataModelOP data;
        RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles, data.starts, data.ends);
        RoutingModel routing(manager);

        const int transit_callback_index = routing.RegisterTransitCallback(
            [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                // Convert from routing variable Index to distance matrix NodeIndex.
                auto from_node = manager.IndexToNode(from_index).value();
                auto to_node = manager.IndexToNode(to_index).value();
                return data.distance_matrix[from_node][to_node];
            });
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

        const int demand_callback_index = routing.RegisterUnaryTransitCallback(
            [&data, &manager](int64_t from_index) -> int64_t {
                // Convert from routing variable Index to demand NodeIndex.
                int from_node = manager.IndexToNode(from_index).value();
                return data.demands[from_node];
            });
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,    // transit callback index
            int64_t{0},               // null capacity slack
            data.vehicle_capacities,  // vehicle maximum capacities
            true,                     // start cumul to zero
            "Capacity");

        if (uni_penalty) {
            int64_t penalty{1000};
            if (start_idx == end_idx) {
                for (int i = 1; i < data.distance_matrix.size(); ++i) {
                    routing.AddDisjunction(
                        {manager.NodeToIndex(RoutingIndexManager::NodeIndex(i))}, penalty);
                }
            }
            else {
                for (int i = 1; i < data.distance_matrix.size() - 1; ++i) {
                    routing.AddDisjunction(
                        {manager.NodeToIndex(RoutingIndexManager::NodeIndex(i))}, penalty);
                }
            }
        }
        else{
            if (start_idx == end_idx) {
                for (int i = 1; i < data.distance_matrix.size(); ++i) {
                    int p = prizes[i - 1];
                    int64_t penalty{1000 * p};
                    routing.AddDisjunction(
                        {manager.NodeToIndex(RoutingIndexManager::NodeIndex(i))}, penalty);
                }
            }
            else {
                for (int i = 1; i < data.distance_matrix.size() - 1; ++i) {
                    int p = prizes[i - 1];
                    int64_t penalty{1000 * p};
                    routing.AddDisjunction(
                        {manager.NodeToIndex(RoutingIndexManager::NodeIndex(i))}, penalty);
                }
            }
        }

        RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
        search_parameters.set_first_solution_strategy(
            FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        search_parameters.set_local_search_metaheuristic(
            LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        search_parameters.mutable_time_limit()->set_seconds(10);

        const Assignment* solution = routing.SolveWithParameters(search_parameters);
        if (routing.status() == 1) {
            getSolution(manager, routing, *solution, initial_solution);
            return 1;
        }
        else {
            throw runtime_error("NO SOLUTION FOUND\n");
        }
    }
}

void getStartBudget(int prize_sum, double simu_eng, int simu_bgt, 
    double exact_eng, int& start_bgt, bool& is_decrease)
{
    is_decrease = true;
    if (simu_eng >= exact_eng) { 
        int err = calcSpeedUp(exact_eng, simu_eng);
        start_bgt = simu_bgt - err;
    }
    else {
        is_decrease = false;
        if (exact_eng - simu_eng <= 4) { start_bgt = simu_bgt; }
        else if (exact_eng - simu_eng <= 6) { start_bgt = simu_bgt + 2; }
        else if (exact_eng - simu_eng <= 8) { start_bgt = simu_bgt + 4; }
        else if (exact_eng - simu_eng <= 10) { start_bgt = simu_bgt + 8; }
        else { start_bgt = simu_bgt + 10; }

        start_bgt = std::min(prize_sum, start_bgt);
    }
}

int main(int argc, char** argv) 
{
    auto start_tot = chrono::high_resolution_clock::now();

    inp_dir = argv[1];
    out_dir = argv[2];
    node_file_name = argv[3];
    sol_file_name = argv[4];
    inc_obs = stoi(argv[5]);
    remain_eng_min = stoi(argv[6]);
    uni_penalty = stoi(argv[7]);
    is_rerun = stoi(argv[8]);
    bool no_speedup = stoi(argv[9]);

    float uav_eng = 0, nodes_eng = 0;
    int prize_sum = 0;
    unique_ptr<Point> uav_start(new Point());
    unique_ptr<Point> uav_end(new Point());
    vector<Sensor> initial_nodes;

    // Read initial node data and calculate assoicated distance matrix.
    try {
        readInitNodes(node_file_name, initial_nodes, prizes, *uav_start, *uav_end, 
        uav_eng, nodes_eng, prize_sum);
    }
    catch (runtime_error& e) { cerr << e.what(); return EXIT_FAILURE; }
    calcDistMat(initial_nodes, dist_mat);

    	// Read wind information.
	Grid* temp_wind = new Grid();
	try { 
		fs::path grid_fname = inp_dir;
		grid_fname /= "grid_info.txt";
		temp_wind->readGridInfo(grid_fname);
	}
	catch (runtime_error& e) { 
		cerr << e.what(); return EXIT_FAILURE;
	}
	try {
		fs::path wind_fname = inp_dir;
		wind_fname /= "wind_vector.csv";
		temp_wind->readWindData(0, 2400, wind_fname);
	}
	catch (exception& e) { 
		cerr << e.what(); return EXIT_FAILURE;
	}
    
    unique_ptr<Drone> uav = make_unique<Drone>(initial_nodes[0].getPt(), uav_eng);
    uav->setWind(temp_wind);

    float remain_eng_rth = 0;
    int speedup = 0;
    int solver_status = 0;
    int tsp_reduced_prize = 0;

    auto alg_start = chrono::high_resolution_clock::now();
    if (no_speedup) {
        try { solver_status = operations_research::calSolTSP(); }
        catch(runtime_error& e) { cerr << e.what() << '\n'; return EXIT_FAILURE; }
        auto alg_end = chrono::high_resolution_clock::now();
        exe_alg_time = static_cast<float>(
            chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;

        if (solver_status == 1 && initial_solution.size() != 0) {  // If found a solution
            remain_eng_rth = checkTask(initial_nodes, initial_solution, *uav_end, *uav);
            saveExecution(1, prize_sum, uav_eng - remain_eng_rth);
            if (remain_eng_rth >= remain_eng_min - 5) {
                savePlan(sol_file_name, initial_solution);

                auto end_tot = chrono::high_resolution_clock::now();
                exe_tot_time = static_cast<float>(
                    chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
                time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
                saveTime(end_time, exe_alg_time, exe_tot_time);
                return EXIT_SUCCESS;
            }
            else {
                int cnt = 2;
                for (int i = 500; i > 0; i -= 1) {
                    uav_capacity = i;
                    try { solver_status = operations_research::calSolOP(); }
                    catch(runtime_error& e) { cerr << e.what() << '\n'; return EXIT_FAILURE; }
                    exe_alg_time = static_cast<float>(
                        chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;
                    if (solver_status == 1) {
                        if (initial_solution.size() != 0) {
                            remain_eng_rth = checkTask(initial_nodes, initial_solution, *uav_end, *uav);
                            if (remain_eng_rth >= remain_eng_min) {
                                savePlan(sol_file_name, initial_solution);

                                auto end_tot = chrono::high_resolution_clock::now();
                                exe_tot_time = static_cast<float>(
                                    chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
                                time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
                                saveTime(end_time, exe_alg_time, exe_tot_time);
                                saveExecution(cnt, i, uav_eng - remain_eng_rth);
                                return EXIT_SUCCESS;
                            }
                        }
                    }
                    saveExecution(cnt, i, uav_eng - remain_eng_rth);
                    cnt++;
                }
            }
        }
    }

    if (is_rerun) {
        try { solver_status = operations_research::calSolTSP(); }
        catch(runtime_error& e) { cerr << e.what() << '\n'; return EXIT_FAILURE; }

        if (solver_status == 1 && initial_solution.size() != 0) {  // If found a solution
            remain_eng_rth = checkTask(initial_nodes, initial_solution, *uav_end, *uav);
            // saveExecution(1, prize_sum, uav_eng - remain_eng_rth);
            if (remain_eng_rth >= remain_eng_min) {
                auto alg_end = chrono::high_resolution_clock::now();
                exe_alg_time = static_cast<float>(
                    chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;
                savePlan(sol_file_name, initial_solution);

                auto end_tot = chrono::high_resolution_clock::now();
                exe_tot_time = static_cast<float>(
                    chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
                time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
                saveTime(end_time, exe_alg_time, exe_tot_time);
                return EXIT_SUCCESS;
            }
            else {  // OP
                // int cnt = 2;
                // auto sim_count_start = chrono::high_resolution_clock::now();
                tsp_reduced_prize = calcSpeedUp(uav_eng - remain_eng_rth, uav_eng);
                for (int i = prize_sum - tsp_reduced_prize; i >= 0; i -= (7 + speedup)) {
                    // auto sim_count_end = chrono::high_resolution_clock::now();
                    // float sim_count = static_cast<float>(
                    //     chrono::duration_cast<chrono::milliseconds>(sim_count_end - sim_count_start).count()) / 1000;
                    // if (sim_count >= 15) {  // Time limit 15 seconds
                    //     savePlan(sol_file_name, initial_solution);

                    //     auto end_tot = chrono::high_resolution_clock::now();
                    //     exe_tot_time = static_cast<float>(
                    //         chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
                    //     time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
                    //     saveTime(end_time, exe_alg_time, exe_tot_time);
                    //     return EXIT_SUCCESS;
                    // }
                    uav_capacity = i;
                    auto alg_start = chrono::high_resolution_clock::now();
                    try { solver_status = operations_research::calSolOP(); }
                    catch(runtime_error& e) { cerr << e.what() << '\n'; return EXIT_FAILURE; }

                    if (solver_status == 1) {
                        // && i >= 30
                        if (initial_solution.size() != 0) {
                            remain_eng_rth = checkTask(initial_nodes, initial_solution, *uav_end, *uav);
                            if (remain_eng_rth >= remain_eng_min) {
                                auto alg_end = chrono::high_resolution_clock::now();
                                exe_alg_time = static_cast<float>(
                                    chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;
                                savePlan(sol_file_name, initial_solution);

                                auto end_tot = chrono::high_resolution_clock::now();
                                exe_tot_time = static_cast<float>(
                                    chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
                                time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
                                saveTime(end_time, exe_alg_time, exe_tot_time);
                                // saveExecution(cnt, i, uav_eng - remain_eng_rth);
                                return EXIT_SUCCESS;
                            }
                            speedup = calcSpeedUp(uav_eng - remain_eng_rth, uav_eng);
                        }
                        // else if (initial_solution.size() != 0 && i < 30) {
                        //     initial_solution.pop_back();
                        //     savePlan(sol_file_name, initial_solution);

                        //     auto alg_end = chrono::high_resolution_clock::now();
                        //     exe_alg_time = static_cast<float>(
                        //             chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;
                        //     auto end_tot = chrono::high_resolution_clock::now();
                        //     exe_tot_time = static_cast<float>(
                        //         chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
                        //     time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
                        //     saveTime(end_time, exe_alg_time, exe_tot_time);
                        //     saveExecution(cnt, i, uav_eng - remain_eng_rth);
                        //     return EXIT_SUCCESS;
                        // }
                        // saveExecution(cnt, i, uav_eng - remain_eng_rth);
                        // cnt++;
                    }
                    else {
                        cerr << "No solution found with OR-Tools. \n";
                        return EXIT_FAILURE;
                    }
                }
            }
        }
        else {
            cerr << "No solution found with OR-Tools. \n";
            return EXIT_FAILURE;
        }
        cerr << "Exceed maximum trial times, no solution found. \n";
        return EXIT_FAILURE;
    }
    // else {
    //     float simu_eng, simu_time;
    //     int simu_bgt;
    //     readSimuRes(simu_eng, simu_time, simu_bgt);
    //     bool is_decrease = true;
    //     int start_bgt = 0;
    //     getStartBudget(prize_sum, simu_eng, simu_bgt, uav_eng, start_bgt, is_decrease);
    //     uav = make_unique<Drone>(initial_nodes[0].getPt(), uav_eng);
    //     int cnt = 1;
    //     if (is_decrease) {
    //         auto sim_count_start = chrono::high_resolution_clock::now();
    //         for (int i = start_bgt; i >= 0; i -= (7 + speedup)) {
    //             uav_capacity = i;
    //             saveExecution(cnt + 1, uav_capacity); cnt++;   
    //             auto alg_start = chrono::high_resolution_clock::now();
    //             try { solver_status = operations_research::calSolOP(); }
    //             catch(runtime_error& e) { cerr << e.what() << '\n'; return EXIT_FAILURE; }
    //             auto alg_end = chrono::high_resolution_clock::now();
    //             exe_alg_time = static_cast<float>(
    //                 chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;

    //             if (solver_status == 1) {
    //                 if (initial_solution.size() != 0 && i >= 30) {
    //                     remain_eng_rth = checkTask(initial_nodes, initial_solution, *uav_end, *uav);
    //                     if (remain_eng_rth >= remain_eng_min) {
    //                         savePlan(sol_file_name, initial_solution);

    //                         auto end_tot = chrono::high_resolution_clock::now();
    //                         exe_tot_time = static_cast<float>(
    //                             chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
    //                         time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
    //                         saveTime(end_time, exe_alg_time, exe_tot_time);
    //                         return EXIT_SUCCESS;
    //                     }
    //                     else {
    //                         speedup = calcSpeedUp(remain_eng_rth, remain_eng_min);
    //                     }
    //                 }
    //                 else if (initial_solution.size() == 0) { speedup = -8; }
    //                 else if (initial_solution.size() != 0 && i < 30) {
    //                     initial_solution.pop_back();
    //                     savePlan(sol_file_name, initial_solution);

    //                     auto end_tot = chrono::high_resolution_clock::now();
    //                     exe_tot_time = static_cast<float>(
    //                         chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
    //                     time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
    //                     saveTime(end_time, exe_alg_time, exe_tot_time);
    //                     return EXIT_SUCCESS;
    //                 }
    //             }
    //             else {
    //                 cerr << "No solution found with OR-Tools. \n";
    //                 return EXIT_FAILURE;
    //             }         
    //         }
    //     }
    //     else {
    //         try { solver_status = operations_research::calSolTSP(); }
    //         catch(runtime_error& e) { cerr << e.what() << '\n'; return EXIT_FAILURE; }  
    //         auto alg_end = chrono::high_resolution_clock::now();
    //         exe_alg_time = static_cast<float>(
    //             chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;

    //         if (solver_status == 1 && initial_solution.size() != 0) {  // If found a solution
    //             remain_eng_rth = checkTask(initial_nodes, initial_solution, *uav_end, *uav);
    //             tsp_reduced_prize = calcSpeedUp(remain_eng_rth, remain_eng_min);
    //             if (remain_eng_rth >= remain_eng_min - 5) {
    //                 savePlan(sol_file_name, initial_solution);

    //                 auto end_tot = chrono::high_resolution_clock::now();
    //                 exe_tot_time = static_cast<float>(
    //                     chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
    //                 time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
    //                 saveTime(end_time, exe_alg_time, exe_tot_time);
    //                 saveExecution(cnt + 1, prize_sum);
    //                 return EXIT_SUCCESS;
    //             }
    //         }
    //         else {  // OP
    //             auto sim_count_start = chrono::high_resolution_clock::now();
    //             for (int i = prize_sum - tsp_reduced_prize; i > start_bgt; i -= (7 + speedup)) { 
    //                 uav_capacity = i;
    //                 saveExecution(cnt + 1, uav_capacity); cnt++;    
    //                 auto alg_start = chrono::high_resolution_clock::now();
    //                 try { solver_status = operations_research::calSolOP(); }
    //                 catch(runtime_error& e) { cerr << e.what() << '\n'; return EXIT_FAILURE; }
    //                 auto alg_end = chrono::high_resolution_clock::now();
    //                 exe_alg_time = static_cast<float>(
    //                     chrono::duration_cast<chrono::milliseconds>(alg_end - alg_start).count()) / 1000;

    //                 if (solver_status == 1) {
    //                     if (initial_solution.size() != 0 && i >= 30) {
    //                         remain_eng_rth = checkTask(initial_nodes, initial_solution, *uav_end, *uav);
    //                         if (remain_eng_rth >= remain_eng_min) {
    //                             savePlan(sol_file_name, initial_solution);

    //                             auto end_tot = chrono::high_resolution_clock::now();
    //                             exe_tot_time = static_cast<float>(
    //                                 chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
    //                             time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
    //                             saveTime(end_time, exe_alg_time, exe_tot_time);
    //                             return EXIT_SUCCESS;
    //                         }
    //                         else {
    //                             speedup = calcSpeedUp(remain_eng_rth, remain_eng_min);
    //                         }
    //                     }
    //                     else if (initial_solution.size() == 0) { speedup = -8; }
    //                     else if (initial_solution.size() != 0 && i < 30) {
    //                         initial_solution.pop_back();
    //                         savePlan(sol_file_name, initial_solution);

    //                         auto end_tot = chrono::high_resolution_clock::now();
    //                         exe_tot_time = static_cast<float>(
    //                             chrono::duration_cast<chrono::milliseconds>(end_tot - start_tot).count()) / 1000;
    //                         time_t end_time = chrono::system_clock::to_time_t(chrono::system_clock::now());
    //                         saveTime(end_time, exe_alg_time, exe_tot_time);
    //                         return EXIT_SUCCESS;
    //                     }
    //                 }           
    //             } 
    //         }
    //     }
    // }
}