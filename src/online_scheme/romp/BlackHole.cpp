#include "BlackHole.h"

using std::vector;
using std::unique_ptr, std::make_unique;
using std::make_pair;

void BlackHole::set_sensors(vector<Sensor>& sn)
{
	_sn = std::move(sn);
}

void BlackHole::set_bh_from_proc(const std::vector<size_t>& sol_idx,
	double metric, double dis_E, double re_E)
{
	_tars_metric[_bh_id] = metric;
	_tars_disE_reE[_bh_id] = make_pair(dis_E, re_E);
	_tars_idx[_bh_id] = sol_idx;
}

void BlackHole::check_prev_path(const WindGrid& wind, const vector<size_t>& prev_path, 
	double t_start, double all_recharable_energy, double& init_metric,
	double& init_discharged_energy, double& init_recharged_energy)
{
	vector<Point> prev_pts; prev_pts.reserve(prev_path.size());
	for (size_t i = 1; i < prev_path.size() - 1; i++) {
		prev_pts.push_back(_sn[prev_path[i]].get_pt());
	}
	_bh = make_unique<Star>(_sn.size()); 
	_bh->path_E_T.push_back(make_pair(_start_idx, make_pair(_bgt, _init_uav_T)));
	unique_ptr<Point> init_pt = make_unique<Point>(_sn[_start_idx].get_pt());
	unique_ptr<Drone> uav = make_unique<Drone>(*init_pt, _bgt, _init_uav_T);
	unique_ptr<Point> target = make_unique<Point>(
		init_pt->get_x(), init_pt->get_y(), uav->get_norm_alt());

	uav->takeoff(wind, *target, true);
	size_t cnt = 1;  // Skip the start node
	init_metric = init_discharged_energy = init_recharged_energy = 0;
	do {
		size_t this_sn = prev_path[cnt];
		target->set_x(prev_pts[0].get_x());
		target->set_y(prev_pts[0].get_y());
		uav->cruise(wind, *target, true);

		target->set_z(uav->get_hover_alt());
		uav->landing(wind, *target, true);
		double ipt_cost = 0, ipt_charged = 0;
		_sn[this_sn].get_chrg_pkg(ipt_cost, ipt_charged);
		uav->set_energy(uav->get_energy() - ipt_cost);
		_bh->prize += ipt_charged;
		_bh->path_E_T.push_back(make_pair(
			prev_path[cnt], make_pair(uav->get_energy(), uav->get_time())));
		_bh->mask_visit(this_sn);
		
		target->set_z(uav->get_norm_alt());
		uav->takeoff(wind, *target, true);
		prev_pts.erase(prev_pts.begin());
		cnt++;
	} while (!prev_pts.empty());
	// And return to home
	uav->return_to_home(wind, _sn[_end_idx].get_pt());
	_bh->path_E_T.push_back(make_pair(
		1, make_pair(uav->get_energy(), uav->get_time())));
	_bh->cost = _bgt - uav->get_energy();
	_bh->metric = _wt_E * init_recharged_energy / all_recharable_energy -
		(100 - _wt_E) * init_discharged_energy / _bgt;
	_bh->feasible = (_bh->cost <= _bgt ? true : false);
	// Update initial solution performance evaluation.
	init_discharged_energy = _bh->cost;
	init_recharged_energy = _bh->prize;
	init_metric = _bh->metric;
}

void BlackHole::_update_path_E_T_for_drop(const WindGrid& wind, 
	size_t idx2drop, size_t node2drop)
{
	// We only need to update the nodes after idx2drop.
	size_t len2update = _bh->path_E_T.size() - idx2drop;
	size_t update_start_idx = idx2drop - 1;
	_bh->prize -= _sn[node2drop].get_charge_package();
	_bh->unmask_visit(node2drop);
	_bh->path_E_T.erase(_bh->path_E_T.begin() + idx2drop);
	vector<Point> pts2update; pts2update.reserve(len2update);
	for (size_t i = idx2drop; i < _bh->path_E_T.size() - 1; i++) {
		pts2update.push_back(_sn[_bh->path_E_T[i].first].get_pt());
	}
	// The node at idx2drop - 1 remains same, so we use its data as UAV initial state.
	unique_ptr<Point> init_pt = make_unique<Point>(
		_sn[_bh->path_E_T[update_start_idx].first].get_pt());
	unique_ptr<Drone> uav = make_unique<Drone>(*init_pt, 
		_bh->path_E_T[update_start_idx].second.first,
		_bh->path_E_T[update_start_idx].second.second);
	unique_ptr<Point> target = make_unique<Point>(
		init_pt->get_x(), init_pt->get_y(), uav->get_norm_alt());
	uav->takeoff(wind, *target, true);
	size_t cnt = idx2drop;  // Skip the start node
	do {
		size_t this_sn = _bh->path_E_T[cnt].first;
		target->set_x(pts2update[0].get_x());
		target->set_y(pts2update[0].get_y());
		uav->cruise(wind, *target, true);

		target->set_z(uav->get_hover_alt());
		uav->landing(wind, *target, true);
		double ipt_cost = 0, ipt_charged = 0;
		_sn[this_sn].get_chrg_pkg(ipt_cost, ipt_charged);
		uav->set_energy(uav->get_energy() - ipt_cost);
		_bh->path_E_T[cnt].second.first = uav->get_energy();
		_bh->path_E_T[cnt].second.second = uav->get_time();

		target->set_z(uav->get_norm_alt());
		uav->takeoff(wind, *target, true);
		pts2update.erase(pts2update.begin());
		cnt++;
	} while (!pts2update.empty());
	// And return to home
	uav->return_to_home(wind, _sn[_end_idx].get_pt());
	_bh->path_E_T[_bh->path_E_T.size() - 1].second.first = uav->get_energy();
	_bh->path_E_T[_bh->path_E_T.size() - 1].second.second = uav->get_time();
	_bh->cost = _bgt - uav->get_energy();
	_bh->feasible = (_bh->cost <= _bgt ? true : false);
	// We didn't update the metric here
}

void BlackHole::drop_node_until_feasible(const WindGrid& wind)
{
	while (!_bh->feasible) {  // First iteration, we can still use path_E_T
		ValIdxVal dropval_map; 
		for (size_t i = 1; i < _bh->path_E_T.size() - 1; i++) {
			// previous energy is higher
			double cost_prev2i = _bh->path_E_T[i - 1].second.first
				- _bh->path_E_T[i].second.first;
			double cost_i2next = _bh->path_E_T[i].second.first 
				- _bh->path_E_T[i + 1].second.first;
			Point prev_pt = _sn[_bh->path_E_T[i - 1].first].get_pt();
			double prev_t = _bh->path_E_T[i - 1].second.first;
			unique_ptr<Drone> temp_uav = make_unique<Drone>(prev_pt, 0, prev_t);
			Sensor next_sn = _sn[_bh->path_E_T[i + 1].first];
			double cost_prev2next = temp_uav->get_energy_cost(wind, next_sn);
			double cost_diff = std::max(1e-5, cost_prev2i + cost_i2next - cost_prev2next);
			size_t sn_idx = _bh->path_E_T[i].first;
			dropval_map.insert(make_pair(
				_sn[sn_idx].get_prize() / cost_diff, make_pair(sn_idx, cost_diff)));
		}
		size_t node2drop = dropval_map.begin()->second.first;
		size_t idx2drop = 0;
		auto it = std::find_if(_bh->path_E_T.begin(), _bh->path_E_T.end(),
			[node2drop](const IdxValpair& item) {
				if (node2drop == item.first) return true;
				return false;});
		size_t index = std::distance(_bh->path_E_T.begin(), it);
		if (index != _bh->path_E_T.size()) { idx2drop = it - _bh->path_E_T.begin(); }
		_update_path_E_T_for_drop(wind, idx2drop, node2drop);
	}
}

double BlackHole::_get_energy_cost(const WindGrid& wind, const Point& start_pt, 
	double start_t, const Sensor& target_sn) const
{
	unique_ptr<Drone> uav = make_unique<Drone>(start_pt, 0, start_t);
	unique_ptr<Point> target_pt = make_unique<Point>(
		start_pt.get_x(), start_pt.get_y(), uav->get_norm_alt());
	uav->takeoff(wind, *target_pt, true);
	target_pt->set_x(target_sn.get_pt().get_x());
	target_pt->set_y(target_sn.get_pt().get_y());
	uav->cruise(wind, *target_pt, true);
	double ipt_cost = 0, ipt_charged = 0;
	target_sn.get_chrg_pkg(ipt_cost, ipt_charged);
	uav->set_energy(uav->get_energy() - ipt_cost);
	target_pt->set_z(uav->get_hover_alt());
	uav->landing(wind, *target_pt, true);
	return -uav->get_energy();
}

void BlackHole::_get_energy_cost_and_time(const WindGrid& wind,
	const Point& start_pt, double start_t, const Sensor& target_sn, 
	double& energy_cost, double& time_cost) const
{
	unique_ptr<Drone> uav = make_unique<Drone>(start_pt, 0, start_t);
	unique_ptr<Point> target_pt = make_unique<Point>(
		start_pt.get_x(), start_pt.get_y(), uav->get_norm_alt());
	uav->takeoff(wind, *target_pt, true);
	target_pt->set_x(target_sn.get_pt().get_x());
	target_pt->set_y(target_sn.get_pt().get_y());
	uav->cruise(wind, *target_pt, true);
	double ipt_cost = 0, ipt_charged = 0;
	target_sn.get_chrg_pkg(ipt_cost, ipt_charged);
	uav->set_energy(uav->get_energy() - ipt_cost);
	target_pt->set_z(uav->get_hover_alt());
	uav->landing(wind, *target_pt, true);
	energy_cost = -uav->get_energy();
	time_cost = uav->get_time() - start_t;
}

void BlackHole::_calc_addval_map(const WindGrid& wind, ValIdxpairval& addval_map)
{
	for (size_t v = _num_depots; v < _sn.size(); v++) {
		if (!_bh->is_node_visit(v)) {
			// cost, {node, node index in path}
			std::multimap<double, IdxPair> cost_nbrs;
			for (size_t i = 0; i < _bh->path_E_T.size(); i++) {
				size_t idx = _bh->path_E_T[i].first;
				Point start_pt = _sn[idx].get_pt();
				double start_t = _bh->path_E_T[i].second.second;
				double visit_cost = _get_energy_cost(wind, start_pt, start_t, _sn[v]);
				cost_nbrs.insert(make_pair(visit_cost, make_pair(idx, i)));
			}
			vector<IdxPair> nbr_pairs;  // {node, node index in path}
			for (auto it = cost_nbrs.begin(); it != cost_nbrs.end(); ++it) {
				nbr_pairs.push_back(make_pair(it->second.first, it->second.second));
				if (nbr_pairs.size() > 2) { break; }  // Find at most 3 neighbors
			}
			// {{nbr, adj}, {nbr index in path, adj index in path}}, add_cost
			ValDualpair cost_nbr_pairs;
			for (size_t i = 0; i < nbr_pairs.size(); i++) {
				size_t nbr = nbr_pairs[i].first, nbr_idx = nbr_pairs[i].second;
				Point nbr_pt = _sn[nbr].get_pt();
				double nbr_t = _bh->path_E_T[nbr_idx].second.second;
				if (nbr_idx < _bh->path_E_T.size() - 1) {  // If insert at nbr's right
					size_t next_idx = nbr_idx + 1;
					double cost_nbr2next = _bh->path_E_T[nbr_idx].second.first
						- _bh->path_E_T[next_idx].second.first;
					double cost_nbr2v = 0, time_nbr2v = 0; 
					_get_energy_cost_and_time(
						wind, nbr_pt, nbr_t, _sn[v], cost_nbr2v, time_nbr2v);
					Sensor adj_sn = _sn[_bh->path_E_T[next_idx].first];
					double cost_v2next = _get_energy_cost(
						wind, _sn[v].get_pt(), nbr_t + time_nbr2v, adj_sn);
					double add_cost = cost_nbr2v + cost_v2next - cost_nbr2next;
					cost_nbr_pairs.insert(make_pair(add_cost, make_pair(
						make_pair(nbr, _bh->path_E_T[next_idx].first), 
						make_pair(nbr_idx, next_idx))));
				}
				if (nbr_idx > 0) {  // If insert at nbr's left
					size_t prev_idx = nbr_idx - 1;
					double cost_prev2nbr = _bh->path_E_T[prev_idx].second.first 
						- _bh->path_E_T[nbr_idx].second.first;
					Point prev_pt = _sn[_bh->path_E_T[prev_idx].first].get_pt();
					double prev_t = _bh->path_E_T[prev_idx].second.second;
					double cost_prev2v = 0, time_prev2v = 0;
					_get_energy_cost_and_time(
						wind, prev_pt, prev_t, _sn[v], cost_prev2v, time_prev2v);
					double cost_v2nbr = _get_energy_cost(
						wind, _sn[v].get_pt(), prev_t + time_prev2v, _sn[nbr]);
					double add_cost = cost_prev2v + cost_v2nbr - cost_prev2nbr;
					cost_nbr_pairs.insert(make_pair(add_cost, make_pair(
						make_pair(_bh->path_E_T[prev_idx].first, nbr),
						make_pair(prev_idx, nbr_idx))));
				}
			}
			// Get the smallest addcost of this node v.
			double addcost_v = 1e6; size_t idx2add_v = 0;
			addcost_v = cost_nbr_pairs.begin()->first;
			idx2add_v = std::max(  // left, node to add, right
				cost_nbr_pairs.begin()->second.second.first,
				cost_nbr_pairs.begin()->second.second.second);
			// If the addcost within budget constraint, add it into the addval_map
			if (addcost_v + _bh->cost <= _bgt) {
				addval_map.insert(make_pair(_sn[v].get_prize() / addcost_v,
					make_pair(make_pair(v, idx2add_v), addcost_v)));
			}
		}
	}
}

void BlackHole::_update_path_E_T_for_add(const WindGrid& wind, 
	size_t idx2add, size_t node2add)
{
	// We only need to update the nodes at and after idx2add.
	size_t len2update = _bh->path_E_T.size() - idx2add;
	size_t update_start_idx = idx2add - 1;
	_bh->prize += _sn[node2add].get_charge_package();
	_bh->mask_visit(node2add);
	_bh->path_E_T.insert(_bh->path_E_T.begin() + idx2add, 
		make_pair(node2add, make_pair(0, 0)));  // Add 0 values first 
	vector<Point> pts2update; pts2update.reserve(len2update);
	for (size_t i = idx2add; i < _bh->path_E_T.size() - 1; i++) {
		pts2update.push_back(_sn[_bh->path_E_T[i].first].get_pt());
	}
	// The node at idx2drop - 1 remains same, so we use its data as UAV initial state.
	unique_ptr<Point> init_pt = make_unique<Point>(
		_sn[_bh->path_E_T[update_start_idx].first].get_pt());
	unique_ptr<Drone> uav = make_unique<Drone>(*init_pt,
		_bh->path_E_T[update_start_idx].second.first,
		_bh->path_E_T[update_start_idx].second.second);
	unique_ptr<Point> target = make_unique<Point>(
		init_pt->get_x(), init_pt->get_y(), uav->get_norm_alt());
	uav->takeoff(wind, *target, true);
	size_t cnt = idx2add;  // Already inserted the node in the solution path
	do {
		size_t this_sn = _bh->path_E_T[cnt].first;
		target->set_x(pts2update[0].get_x());
		target->set_y(pts2update[0].get_y());
		uav->cruise(wind, *target, true);

		target->set_z(uav->get_hover_alt());
		uav->landing(wind, *target, true);
		double ipt_cost = 0, ipt_charged = 0;
		_sn[this_sn].get_chrg_pkg(ipt_cost, ipt_charged);
		uav->set_energy(uav->get_energy() - ipt_cost);
		_bh->path_E_T[cnt].second.first = uav->get_energy();
		_bh->path_E_T[cnt].second.second = uav->get_time();

		target->set_z(uav->get_norm_alt());
		uav->takeoff(wind, *target, true);
		pts2update.erase(pts2update.begin());
		cnt++;
	} while (!pts2update.empty());
	// And return to home
	uav->return_to_home(wind, _sn[_end_idx].get_pt());
	_bh->path_E_T[_bh->path_E_T.size() - 1].second.first = uav->get_energy();
	_bh->path_E_T[_bh->path_E_T.size() - 1].second.second = uav->get_time();
	_bh->cost = _bgt - uav->get_energy();
	_bh->feasible = (_bh->cost <= _bgt ? true : false);
	// We didn't update the metric here
}

void BlackHole::add_node_until_budget(const WindGrid& wind)
{
	while (!_bh->is_all_visit() && _bh->feasible) {
		size_t node2add = 0, idx2add = 0;
		ValIdxpairval addval_map;
		_calc_addval_map(wind, addval_map);
		if (!addval_map.empty() && addval_map.begin()->first > 0) {
			// Get the node v and index to add in current path (whose 
			// addval is positive and the largest).
			node2add = addval_map.begin()->second.first.first;
			idx2add = addval_map.begin()->second.first.second;
		}
		else { break; }
		_update_path_E_T_for_add(wind, idx2add, node2add);
	}
}

size_t BlackHole::_find_a_candidate(const vector<size_t>& pop_sol, size_t idx2update)
{
	size_t prev_node = pop_sol[idx2update - 1];
	size_t next_node = pop_sol[idx2update + 1];
	ValIdx metric_map;  // distance cost, node id
	for (size_t i = 2; i < _sn.size(); i++) {
		if (i == prev_node || i == next_node) { continue; }
		size_t prize_diff = _sn[i].get_prize() - _min_prize;
		double dist_diff = _dists[prev_node][i] + _dists[i][next_node];
		double metric_i = _wt_E * prize_diff - (100 - _wt_E) * log10(dist_diff);
		metric_map.insert(make_pair(metric_i, i));
	}

	vector<size_t> final_nodes; final_nodes.reserve(_num_srch);
	for (size_t i = 0; i < _num_srch; i++) {
		final_nodes.push_back(metric_map.begin()->second);
		metric_map.erase(metric_map.begin());
	}
	unique_ptr<Utils> utils = make_unique<Utils>();
	vector<size_t> out;
	utils->vector_unbiased_sample(final_nodes, 1, out);
	return out[0];
}

void BlackHole::_init_one_new_sol(vector<size_t>& pop_sol)
{
	unique_ptr<Utils> utils = make_unique<Utils>();
	// Exclude start and end node
	for (size_t i = 1; i < pop_sol.size() - 1; i++) {
		double prob = utils->sample_rand_num(0., 1.);
		if (prob > _prob_attr) { continue; }
		// Find a candidate node
		size_t cand_node = _find_a_candidate(pop_sol, i);
		auto it = std::find_if(pop_sol.begin(), pop_sol.end(),
			[cand_node](size_t elem) {
				if (elem == cand_node) { return true; }
				else { return false; }
			});
		size_t idx = std::distance(pop_sol.begin(), it);
		if (idx == pop_sol.size()) { pop_sol[i] = cand_node; }
		else { std::swap(pop_sol[i], pop_sol[idx]); }
	}
}

void BlackHole::initialize_population_solutions(
	const WindGrid& wind, double all_rechargeable_energy)
{
	// Population solution path
	if (!_tars_idx.empty()) { _tars_idx.clear(); } _tars_idx.reserve(_num_pops); 
	if (!_bh_sol.empty()) { _bh_sol.clear(); } _bh_sol.reserve(_bh->path_E_T.size());
	for (size_t i = 0; i < _bh->path_E_T.size(); i++) {
		_bh_sol.push_back(_bh->path_E_T[i].first);
	}
	_tars_idx.push_back(_bh_sol);
	// Population solution metric
	if (!_tars_metric.empty()) { _tars_metric.clear(); }
	_tars_metric.reserve(_num_pops);
	_bh->metric = _wt_E * _bh->prize / all_rechargeable_energy
		- (100 - _wt_E) * _bh->cost / _bgt;
	_tars_metric.push_back(_bh->metric);
	// Population discharged and recharged energy
	if (!_tars_disE_reE.empty()) { _tars_disE_reE.reserve(_num_pops); }
	_tars_disE_reE.reserve(_num_pops); 
	_tars_disE_reE.push_back(make_pair(_bh->cost, _bh->prize));
	// Population solution feasibility
	if (!_tars_feas.empty()) { _tars_feas.reserve(_num_pops); }
	_tars_feas.reserve(_num_pops);
	_tars_feas.push_back(true);
	// Calculate the distance matrix in advance.
	unique_ptr<Utils> utils = make_unique<Utils>();
	utils->calc_distance_matrix(_sn, _dists);
	// Start initialization
	double best_met = _tars_metric[0];
	for (size_t k = 1; k < _num_pops; k++) {
		vector<size_t> pop_sol = _bh_sol;
		_init_one_new_sol(pop_sol);
		utils->two_opt(_dists, _max_2opt, pop_sol);
		_tars_idx.push_back(pop_sol);
		double pop_metirc = 0, pop_dis_E = 0, pop_re_E = 0;
		bool pop_feasible = false;
		fitness(wind, pop_sol, all_rechargeable_energy, pop_metirc, 
			pop_dis_E, pop_re_E, pop_feasible);
		_tars_metric.push_back(pop_metirc);
		_tars_disE_reE.push_back(make_pair(pop_dis_E, pop_re_E));
		_tars_feas.push_back(pop_feasible);
		if (pop_metirc > best_met && pop_feasible) { 
			_bh_id = k; best_met = pop_metirc; 
		}
	}
}

vector<size_t> BlackHole::_find_candidates(
	const vector<size_t>& pop_sol, size_t idx2update)
{
	size_t prev_node = pop_sol[idx2update - 1];
	size_t next_node = pop_sol[idx2update + 1];
	ValIdx metric_map;  // distance cost, node id
	for (size_t i = 2; i < _sn.size(); i++) {
		if (i == prev_node || i == next_node) { continue; }
		size_t prize_diff = _sn[i].get_prize() - _min_prize;
		double dist_diff = _dists[prev_node][i] + _dists[i][next_node];
		double metric_i = _wt_E * prize_diff - (100 - _wt_E) * log10(dist_diff);
		metric_map.insert(make_pair(metric_i, i));
	}

	vector<size_t> final_nodes; final_nodes.reserve(_num_srch);
	for (size_t i = 0; i < _num_srch; i++) {
		final_nodes.push_back(metric_map.begin()->second);
		metric_map.erase(metric_map.begin());
	}
	return final_nodes;
}

void BlackHole::attraction(vector<size_t>& tar_vec)
{
	double dx = 0, dy = 0, rand_fac = 0;
	unique_ptr<Utils> utils = make_unique<Utils>();
	for (size_t i = 1; i < tar_vec.size() - 1; i++) {
		double prob = utils->sample_rand_num(0., 1.);
		if (prob > _prob_attr || tar_vec[i] == _tars_idx[_bh_id][i]) { continue; }
		// Attraction		
		rand_fac = utils->sample_rand_num(0., 1.);
		unique_ptr<Point> virtual_pt = make_unique<Point>(
			_sn[_tars_idx[_bh_id][i]].get_pt() - _sn[tar_vec[i]].get_pt());
		dx = rand_fac * (virtual_pt->get_x());
		dy = rand_fac * (virtual_pt->get_y());
		virtual_pt->set_x(_sn[_tars_idx[_bh_id][i]].get_pt().get_x() + dx);
		virtual_pt->set_y(_sn[_tars_idx[_bh_id][i]].get_pt().get_y() + dy);
		// Fit virtual_pt into feasible point
		vector<size_t> cand_vec; cand_vec.reserve(_num_srch);
		cand_vec = _find_candidates(tar_vec, i);
		ValIdx dist_map;
		for (const size_t& node : cand_vec) {
			double dist_square = (*virtual_pt - _sn[node].get_pt()).get_xy_square();
			dist_map.insert(make_pair(dist_square, node));
		}
		size_t argmin_node = dist_map.begin()->second;
		auto it = std::find_if(tar_vec.begin(), tar_vec.end(),
			[argmin_node](size_t elem) {
				if (elem == argmin_node) { return true; }
				else { return false; }
			});
		size_t idx = std::distance(tar_vec.begin(), it);
		if (idx == tar_vec.size()) { tar_vec[i] = argmin_node; }
		else { std::swap(tar_vec[i], tar_vec[idx]); }
	}
}

void BlackHole::fitness(const WindGrid& wind, const vector<size_t>& tar_vec, 
	double all_E, double& metric, double& dis_E, double& re_E, bool& feasible)
{
	metric = dis_E = re_E = 0;
	vector<Point> tar_pts; tar_pts.reserve(tar_vec.size());
	for (size_t i = 1; i < tar_vec.size() - 1; i++) {
		tar_pts.push_back(_sn[tar_vec[i]].get_pt());
	}

	unique_ptr<Point> init_pt = make_unique<Point>(_sn[_start_idx].get_pt());
	unique_ptr<Drone> uav = make_unique<Drone>(*init_pt, _bgt, _init_uav_T);
	unique_ptr<Point> target = make_unique<Point>(
		init_pt->get_x(), init_pt->get_y(), uav->get_norm_alt());

	uav->takeoff(wind, *target, true);
	size_t cnt = 1;  // Skip the start node
	do {
		size_t this_sn = tar_vec[cnt];
		target->set_x(tar_pts[0].get_x());
		target->set_y(tar_pts[0].get_y());
		uav->cruise(wind, *target, true);

		target->set_z(uav->get_hover_alt());
		uav->landing(wind, *target, true);
		double ipt_cost = 0, ipt_charged = 0;
		_sn[this_sn].get_chrg_pkg(ipt_cost, ipt_charged);
		uav->set_energy(uav->get_energy() - ipt_cost);
		re_E += ipt_charged;

		target->set_z(uav->get_norm_alt());
		uav->takeoff(wind, *target, true);
		tar_pts.erase(tar_pts.begin());
		cnt++;
	} while (!tar_pts.empty());
	// And return to home
	uav->return_to_home(wind, _sn[_end_idx].get_pt());
	double m_prize = _wt_E * re_E / all_E;
	dis_E = _bgt - uav->get_energy();
	feasible = (dis_E <= _bgt ? true : false);
	double m_cost = 100 - _wt_E - (100 - _wt_E) * dis_E / _bgt;
	double penalty = (feasible ? 0 : -100);
	metric = m_prize + m_cost + penalty;
}

void BlackHole::single_generation_evolvtion(
	const WindGrid& wind, double all_recharable_energy)
{
	unique_ptr<Utils> utils = make_unique<Utils>();
	for (size_t k = 0; k < _num_pops; k++) {
		if (k == _bh_id) { continue; }
		vector<size_t> temp_tar = _tars_idx[k];
		attraction(temp_tar);
		utils->two_opt(_dists, _max_2opt, temp_tar);
		double temp_met = 0, temp_dis_E = 0, temp_re_E = 0;
		bool temp_feasible = false;
		fitness(wind, temp_tar, all_recharable_energy, temp_met, 
			temp_dis_E, temp_re_E, temp_feasible);
		if (temp_met > _tars_metric[k] && temp_feasible) {
			_tars_idx[k] = temp_tar;
			_tars_metric[k] = temp_met;
			_tars_disE_reE[k] = make_pair(temp_dis_E, temp_re_E);
			_tars_feas[k] = temp_feasible;
		}
	}
	// Find updated black hole index
	size_t bh_id = static_cast<size_t>(std::max_element(
		_tars_metric.begin(), _tars_metric.end()) - _tars_metric.begin());
	if (_tars_feas[bh_id]) { _bh_id = bh_id; }
	double upquart_met = utils->find_vec_up_quart(_tars_metric);
	for (size_t k = 0; k < _num_pops; k++) {
		if (k == _bh_id) { continue; }
		if (_tars_metric[k] < upquart_met) {
			_tars_idx[k] = _tars_idx[_bh_id];
			_init_one_new_sol(_tars_idx[k]);
			utils->two_opt(_dists, _max_2opt, _tars_idx[k]);
			double temp_met = 0, temp_dis_E = 0, temp_re_E = 0; 
			bool temp_feasible = false;
			fitness(wind, _tars_idx[k], all_recharable_energy, temp_met,
				temp_dis_E, temp_re_E, temp_feasible);
			_tars_metric[k] = temp_met;
			_tars_disE_reE[k] = make_pair(temp_dis_E, temp_re_E);
			_tars_feas[k] = temp_feasible;
		}
	}
	// Update black hole index and related attributes again.
	bh_id = static_cast<size_t>(std::max_element(
		_tars_metric.begin(), _tars_metric.end()) - _tars_metric.begin());
	if (_tars_feas[bh_id]) { _bh_id = bh_id; }
}

void BlackHole::_check_out_bh(const WindGrid& wind, 
	const vector<size_t>& bh_sol, double all_re_E)
{
	_bh->cost = _bh->prize = _bh->metric = 0;
	vector<Point> bh_pts; bh_pts.reserve(bh_sol.size());
	for (size_t i = 1; i < bh_sol.size() - 1; i++) {
		bh_pts.push_back(_sn[bh_sol[i]].get_pt());
	}
	if (!_bh->path_E_T.empty()) { _bh->path_E_T.clear(); } 
	_bh->path_E_T.reserve(bh_sol.size());
	_bh->path_E_T.push_back(make_pair(_start_idx, make_pair(_bgt, _init_uav_T)));
	unique_ptr<Point> init_pt = make_unique<Point>(_sn[_start_idx].get_pt());
	unique_ptr<Drone> uav = make_unique<Drone>(*init_pt, _bgt, _init_uav_T);
	unique_ptr<Point> target = make_unique<Point>(
		init_pt->get_x(), init_pt->get_y(), uav->get_norm_alt());

	uav->takeoff(wind, *target, true);
	size_t cnt = 1;  // Skip the start node
	do {
		size_t this_sn = bh_sol[cnt];
		target->set_x(bh_pts[0].get_x());
		target->set_y(bh_pts[0].get_y());
		uav->cruise(wind, *target, true);

		target->set_z(uav->get_hover_alt());
		uav->landing(wind, *target, true);
		double ipt_cost = 0, ipt_charged = 0;
		_sn[this_sn].get_chrg_pkg(ipt_cost, ipt_charged);
		uav->set_energy(uav->get_energy() - ipt_cost);
		_bh->prize += ipt_charged;
		_bh->path_E_T.push_back(make_pair(
			bh_sol[cnt], make_pair(uav->get_energy(), uav->get_time())));
		_bh->mask_visit(this_sn);

		target->set_z(uav->get_norm_alt());
		uav->takeoff(wind, *target, true);
		bh_pts.erase(bh_pts.begin());
		cnt++;
	} while (!bh_pts.empty());
	// And return to home
	uav->return_to_home(wind, _sn[_end_idx].get_pt());
	_bh->path_E_T.push_back(make_pair(
		1, make_pair(uav->get_energy(), uav->get_time())));
	_bh->cost = _bgt - uav->get_energy();
	_bh->feasible = (_bh->cost <= _bgt ? true : false);	
	drop_node_until_feasible(wind);
	add_node_until_budget(wind);
	double penalty = (_bh->feasible ? 0 : -100);
	_bh->metric = _wt_E * _bh->prize / all_re_E + 100 - _wt_E - (100 - _wt_E) * _bh->cost / _bgt + penalty;
}

void BlackHole::synchronize_black_hole(const WindGrid& wind, double start_time,
	double all_rechargeable_energy, double& bh_metric)
{
	vector<size_t> bh_sol = _tars_idx[_bh_id];
	_check_out_bh(wind, bh_sol, all_rechargeable_energy);
	bh_metric = _bh->metric;
}
