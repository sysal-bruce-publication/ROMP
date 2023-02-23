#pragma once
#include <map>
#include <iostream>
#include "Drone.h"
#include "Star.h"
#include "Utils.h"

#ifndef BLACKHOLE_H
#define BLACKHOLE_H

typedef std::pair<size_t, double> IdxVal;
typedef std::pair<IdxPair, double> IdxpairVal;
typedef std::multimap<double, IdxpairVal, std::greater<double>> ValIdxpairval;
typedef std::multimap<double, IdxVal> ValIdxVal;
typedef std::pair<IdxPair, IdxPair> DualPair;
typedef std::multimap<double, DualPair> ValDualpair;
typedef std::multimap<double, size_t, std::greater<double>> ValIdx;
typedef std::pair<double, double> ValPair;

class BlackHole
{
public:
	BlackHole() {}
	BlackHole(size_t num_pops, size_t num_srch, size_t wt_energy, double prob_attr, 
		double init_uav_E, double init_uav_T) : _num_pops(num_pops), 
		_num_srch(num_srch), _wt_E(wt_energy), _prob_attr(prob_attr), 
		_bgt(init_uav_E), _init_uav_T(init_uav_T) {}
	~BlackHole() {}

	size_t get_num_nodes() const { return _sn.size(); }
	double get_bh_metric() const { return _tars_metric[_bh_id]; }
	double get_bh_dis_E() const { return _tars_disE_reE[_bh_id].first; }
	double get_bh_re_E() const { return _tars_disE_reE[_bh_id].second; }
	std::vector<size_t> get_bh_sol() const { return _tars_idx[_bh_id]; }
	Star get_bh() const { return *_bh; }
	void exist_non_feasible(int world_id) const
	{
		for (size_t i = 0; i < _num_pops; i++) {
			if (!_tars_feas[i] && _tars_metric[i] >= -50) {
				std::cout << world_id << " " << i << " " << _tars_metric[i] << "\n";
				std::cout.flush();
			}
		}
	}

	void set_sensors(std::vector<Sensor>& sn);
	void set_bh_from_proc(const std::vector<size_t>& sol_idx,
		double metric, double dis_E, double re_E);

	void check_prev_path(const WindGrid& wind, const std::vector<size_t>& prev_path,
		double t_start, double all_rechargeable_energy, double& init_metric,
		double& init_discharged_energy, double& init_recharged_energy);
	void drop_node_until_feasible(const WindGrid& wind);
	void add_node_until_budget(const WindGrid& wind);
	void initialize_population_solutions(
		const WindGrid& wind, double all_rechargeable_energy);
	void attraction(std::vector<size_t>& tar_vec);
	void fitness(const WindGrid& wind, const std::vector<size_t>& tar_vec, double all_e,
		double& metric, double& disE, double& reE, bool& feasible);
	void single_generation_evolvtion(
		const WindGrid& wind, double all_recharable_energy);
	void synchronize_black_hole(const WindGrid& wind, double start_time,
		double all_rechargeable_energy, double& bh_metric);

private:
	const size_t _start_idx = 0;
	const size_t _end_idx = 1;
	const size_t _num_depots = 2;
	const size_t _num_pops = 0;
	const size_t _num_srch = 0;
	const size_t _min_prize = 6;
	const size_t _max_prize = 10;
	const size_t _max_2opt = 100;
	const size_t _wt_E = 50;
	const double _prob_attr = 0;
	const double _bgt = 0;
	const double _init_uav_T = 0;

	std::unique_ptr<Star> _bh;  // For preprocessing
	std::vector<Sensor> _sn;
	std::vector<std::vector<double>> _dists;
	size_t _bh_id = 0;
	std::vector<size_t> _bh_sol;
	std::vector<std::vector<size_t>> _tars_idx;  // Initial solution to evolve.
	std::vector<double> _tars_metric;
	std::vector<ValPair> _tars_disE_reE;		// UAV discharged and recharged energy
	std::vector<bool> _tars_feas;				// Solution feasibility

	void _update_path_E_T_for_drop(const WindGrid& wind, size_t idx2drop,
		size_t node2drop);
	double _get_energy_cost(const WindGrid& wind, const Point& start_pt, 
		double start_t, const Sensor& target_sn) const;
	void _get_energy_cost_and_time(const WindGrid& wind, const Point& start_pt,
		double start_t, const Sensor& target_sn, double& energy_cost, 
		double& time_cost) const;
	void _calc_addval_map(const WindGrid& wind, ValIdxpairval& addval_map);
	void _update_path_E_T_for_add(const WindGrid& wind, size_t idx2add,
		size_t node2add);

	size_t _find_a_candidate(const std::vector<size_t>& pop_sol, size_t idx2update);
	std::vector<size_t> _find_candidates(
		const std::vector<size_t>& pop_sol, size_t idx2update);
	void _init_one_new_sol(std::vector<size_t>& star_k);
	void _check_out_bh(const WindGrid& wind, 
		const std::vector<size_t>& bh_sol, double all_re_E);
};

#endif // !BLACKHOLE_H
