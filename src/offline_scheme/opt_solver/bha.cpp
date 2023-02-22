#include "bha.h"

using namespace std;

float Bha::findUpQuart(vector<float> v)
{
	if (v.size() <= 5) return 0;
	int n = static_cast<int>(v.size() * 0.75);
	nth_element(v.begin(), v.begin() + n, v.end(), greater{});
	return v[n];
}

vector<int> Bha::findCandidates(vector<Sensor> sn_list, vector<int> cur_vec, int cur_idx)
{
	// Find the index of previous element and next element in the solution list.
	int prev_idx = -1, next_idx = -1;
	if (cur_idx != 0 && cur_idx != cur_vec.size() - 1) {
		prev_idx = sn_list[cur_vec[cur_idx - 1]].getID();
		next_idx = sn_list[cur_vec[cur_idx + 1]].getID();
	}
	else if (cur_idx == 0) { next_idx = sn_list[cur_vec[1]].getID(); }
	else { prev_idx = sn_list[cur_vec[cur_idx - 1]].getID(); }

	vector<int> possible_candidates;
	vector<float> simple_metric, sorted_metric;
	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (i == prev_idx || i == next_idx) continue;
		float prev_dist = 0, next_dist = 0;

		if (prev_idx != -1 && next_idx != -1) {
			prev_dist = sn_list[i].getPt().calcXYDist(sn_list[prev_idx].getPt());
			next_dist = sn_list[i].getPt().calcXYDist(sn_list[next_idx].getPt());
		}
		else if (prev_idx == -1 && next_idx != -1) {
			prev_dist = sn_list[i].getPt().calcXYDist(this->_init_coord);
			next_dist = sn_list[i].getPt().calcXYDist(sn_list[next_idx].getPt());
		}
		else if (prev_idx != -1 && next_idx == -1) {
			prev_dist = sn_list[i].getPt().calcXYDist(sn_list[prev_idx].getPt());
			next_dist = sn_list[i].getPt().calcXYDist(this->_end_coord);
		}
		else throw runtime_error("Finding candidates pool, solution data error.\n");
		simple_metric.push_back(this->_wt_eng * (sn_list[i].getWt() - 6)
			- (100 - this->_wt_eng) * log10f(prev_dist + next_dist));
		possible_candidates.push_back(i);
	}

	sorted_metric = simple_metric;
	sort(sorted_metric.begin(), sorted_metric.end(), greater<float>());

	float select_metric = 0;
	if (this->_search_num < sorted_metric.size()) select_metric = sorted_metric[this->_search_num];
	else select_metric = sorted_metric[sorted_metric.size() - 1];

	vector<int> final;
	int cnt = 0;
	for (unsigned i = 0; i < simple_metric.size(); i++) {
		if (simple_metric[i] >= select_metric) {
 			if (cnt == this->_search_num) {
				return final;
			}
			final.push_back(possible_candidates[i]);
			cnt++;
		}
	}
	return final;
}

void Bha::initOneNewSol(vector<Sensor> sn_list, vector<int>& cur_sol)
{
	vector<int> candidates;
	unique_ptr<Algs> temp_alg(new Algs());
	for (unsigned i = 0; i < cur_sol.size(); i++) {
		int ap_factor = temp_alg->randInt(100);
		if (ap_factor > this->_attr_prob) continue;

		// If the scenario is OP case.
		if (cur_sol.size() < sn_list.size() - this->_search_num) {
			candidates.clear();
			candidates = this->findCandidates(sn_list, cur_sol, i);

			//! The selected index in the vector of candidates
			int chose_idx = temp_alg->randInt(static_cast<int>(candidates.size() - 1));
			int chose_node_id = candidates[chose_idx];
			int appear_num = static_cast<int>(count(cur_sol.begin(), cur_sol.end(), chose_node_id));
			if (appear_num == 0) cur_sol[i] = chose_node_id;
			else if (appear_num == 1) {
				int swap_idx = static_cast<int>(find(cur_sol.begin(), cur_sol.end(), chose_node_id)
					- cur_sol.begin());
				swap(cur_sol[i], cur_sol[swap_idx]);
			}
			else throw runtime_error("Data index exists duplicate.\n");
		}
		//! For TSP, the concern is distance limit. Thus initial solution is good enough, so 
		//! the idea of initialization is simple in order to reduce computation cost, and we want
		//! to reduce the probability to swap or replace element.
		else {
			int chosen_idx = temp_alg->randInt(static_cast<int>(sn_list.size() - 1));
			if (chosen_idx == cur_sol[i] || temp_alg->randInt(100) >= 80) break;
			auto it = find(cur_sol.begin(), cur_sol.end(), chosen_idx);
			if (it != cur_sol.end()) {
				int index_in_cur_sol = static_cast<int>(it - cur_sol.begin());
				swap(cur_sol[i], cur_sol[index_in_cur_sol]);
			}
			else cur_sol[i] = chosen_idx;
		}
	}
}

void Bha::initPopNewSol(vector<Sensor> sn_list)
{
	// Clear pop sols
	this->_tars_idx.resize(this->_population);
	this->_tars_idx[0] = this->_bh_sol;

	vector<int> pop_sol;
	for (unsigned i = 1; i < this->_tars_idx.size(); i++) {
		pop_sol = this->_bh_sol;
		try { this->initOneNewSol(sn_list, pop_sol); }
		catch (runtime_error& e) { cerr << e.what(); return; }
		
		this->_tars_idx[i] = pop_sol;
	}
}

void Bha::attract(int bh_num, vector<Sensor> sn_list, vector<int>& tar_vec)
{
	float dx = 0, dy = 0, rand_fac = 0;
	unique_ptr<Algs> temp_alg(new Algs());

	vector<int> candidates;
	for (unsigned i = 0; i < tar_vec.size(); i++) {
		if (temp_alg->randInt(100) > this->_attr_prob || tar_vec[i] == this->_tars_idx[bh_num][i]) {
			continue;
		}

		candidates.clear();
		candidates = this->findCandidates(sn_list, tar_vec, i);

		rand_fac = temp_alg->randFloat();
		unique_ptr<Point> temp_p(new Point());
		temp_p->setPt(sn_list[tar_vec[i]].getPt());
		dx = rand_fac * (sn_list[this->_tars_idx[bh_num][i]].getPt().getX() - temp_p->getX());
		dy = rand_fac * (sn_list[this->_tars_idx[bh_num][i]].getPt().getY() - temp_p->getY());
		temp_p->setX(temp_p->getX() + dx); 
		temp_p->setY(temp_p->getY() + dy);

		// Find distance list of candidates.
		vector<Point> candidates_pt;
		for (unsigned i = 0; i < candidates.size(); i++) {
			candidates_pt.push_back(sn_list[candidates[i]].getPt());
		}
		vector<float> d_list = temp_p->calcXYDist(candidates_pt);

		int selected_idx = -1, min_idx = -1;
		// If we have exactly same point in the `candidates_pt`, then
		// select another minimum distance cost point.
		min_idx = static_cast<int>(min_element(d_list.begin(), d_list.end()) - d_list.begin());

		// If the selected point already existed in the target vector, 
		// then we change their position. 
		int final_idx = candidates[min_idx];
		auto it = find(tar_vec.begin(), tar_vec.end(), final_idx);
		if (it != tar_vec.end()) {
			int index = static_cast<int>(find(tar_vec.begin(), tar_vec.end(), final_idx)
				- tar_vec.begin());
			swap(tar_vec[i], tar_vec[index]);
		} 
		else tar_vec[i] = final_idx;
	}
}

void Bha::fitness(const vector<Sensor>& sn_list, const vector<int>& idx_list, float all_e,
	float& metric, float& discharged_e, float& recharged_e)
{
	metric = 0;
	discharged_e = 0;
	recharged_e = 0;
	vector<Point> temp_plan;
	for (unsigned i = 0; i < idx_list.size(); i++) {
		temp_plan.push_back(sn_list[idx_list[i]].getPt());
	}

	unique_ptr<Drone> temp_pdv(new Drone(this->_init_coord, this->_uav_energy));
	temp_pdv->setWind(this->_wind_grid);
	unique_ptr<Point> temp_target(new Point(this->_init_coord.getX(), 
		this->_init_coord.getY(), temp_pdv->getNormAlt()));
	unique_ptr<Point> avoid(new Point());

	// First ascent to safety altitude.
	temp_pdv->ascent(*temp_target, true);

	float obs_offset = 5;
	int cnt = 0;
	do {
		int this_cn = idx_list[cnt];
		temp_target->setX(temp_plan[0].getX());
		temp_target->setY(temp_plan[0].getY());
		
		int obs_case = 0;
		if (this->_obs->getH() != 0) {
			// For further usage, if want to include multiple obstacles.
			//for (unsigned i = 0; i < this->_obs.size(); i++) {
			avoid->setPt(this->_obs->findAvoidPoint(temp_pdv->getPt(), 
				*temp_target, obs_offset, obs_case));
			//	if (!obs_case) { obs_id = i; break; }
			//}

			if (obs_case != 0) {
				temp_pdv->obstacleFlight(obs_offset, obs_case, this->_obs, *avoid, *temp_target, true);
			}
		}

		if (obs_case == 0) temp_pdv->steadyFlight(*temp_target, true);
		
		// UAV perform Wireless Power Transfer (WPT). 
		// UAV descents to sensor node.
		temp_target->setZ(temp_pdv->getHoverAlt());
		temp_pdv->descent(*temp_target, true);
		// Update energy package neeeded for wireless power transfer.
		float wpt_cost = 0, wpt_charged = 0;
		temp_pdv->calcChrgPkg(sn_list[this_cn], wpt_cost, wpt_charged);
		temp_pdv->setE(temp_pdv->getE() - wpt_cost);
		recharged_e += wpt_charged;

		// UAV ascents to safety altitude.
		temp_target->setZ(temp_pdv->getNormAlt());
		temp_pdv->ascent(*temp_target, true);

		// Finish one visit.
		temp_plan.erase(temp_plan.begin());
		cnt++;
	} while (temp_plan.size());

	temp_pdv->returnToHome(this->_end_coord);
	float prize = this->_wt_eng * recharged_e / all_e;
	discharged_e = this->_uav_energy - temp_pdv->getE();
	float cost = (100 - this->_wt_eng) * discharged_e / this->_uav_energy;
	metric = prize - cost;
}

void Bha::initialization(vector<Sensor> sn_list, float all_e, 
	float& or_met, float& or_dis, float& or_re)
{
	this->initPopNewSol(sn_list);
	this->_tars_metric.resize(this->_population);
	this->_tars_dischrg.resize(this->_population);
	this->_tars_rechrg.resize(this->_population);
	
	// Find black hole index and its fitness metric.
	float temp_metric = 0;
	for (int i = 0; i < this->_population; i++) {
		this->fitness(sn_list, this->_tars_idx[i], all_e,
			this->_tars_metric[i], this->_tars_dischrg[i], this->_tars_rechrg[i]);
	
		if (i == 0) temp_metric = this->_tars_metric[0];
		if (i > 0 && this->_tars_metric[i] > temp_metric) {
			this->_bh_idx = i;
			temp_metric = this->_tars_metric[i];
		}
	}
	or_met = this->_tars_metric[0];
	or_dis = this->_tars_dischrg[0];
	or_re = this->_tars_rechrg[0];
}

void Bha::singleGenEvolve(vector<Sensor> sn_list, float all_e) {
	for (int j = 0; j < this->_population; j++) {
		// sum_fitness includes the fitness metric of Black Hole
		// No attract executed to Black Hole
		if (j == _bh_idx) continue;

		vector<int> temp_tar = _tars_idx[j];
		float temp_met = 0, temp_dis = 0, temp_re = 0;
		this->attract(this->_bh_idx, sn_list, temp_tar);
		this->fitness(sn_list, temp_tar, all_e, temp_met, temp_dis, temp_re);
		if (temp_met > this->_tars_metric[j]) {
			this->_tars_idx[j] = temp_tar;
			this->_tars_metric[j] = temp_met;
			this->_tars_dischrg[j] = temp_dis;
			this->_tars_rechrg[j] = temp_re;
		}
	}

	// Find updated black hole index
	this->_bh_idx = static_cast<int>(
		max_element(this->_tars_metric.begin(), this->_tars_metric.end())
		- this->_tars_metric.begin());
	float upquart_met = this->findUpQuart(_tars_metric);

	for (int j = 0; j < this->_population; j++) {
		// Black hole solution should not be re-initialized.
		if (j == this->_bh_idx) continue;

		if (this->_tars_metric[j] < upquart_met) {
			this->_tars_idx[j] = _tars_idx[this->_bh_idx];
			this->initOneNewSol(sn_list, this->_tars_idx[j]);
			this->fitness(sn_list, this->_tars_idx[j], all_e, 
				this->_tars_metric[j], this->_tars_dischrg[j], 
				this->_tars_rechrg[j]);
		}
	}

	// Update black hole index and related attributes again.
	this->_bh_idx = static_cast<int>(
		max_element(this->_tars_metric.begin(), this->_tars_metric.end())
		- this->_tars_metric.begin());
	this->fitness(sn_list, _tars_idx[this->_bh_idx], all_e, this->_bh_metric,
		this->_bh_discharged_e, this->_bh_recharged_e);
}

void Bha::getBhEstEngTimePrize(const vector<Sensor>& sn_list, const vector<int> bh_sol,
	vector<float>& path_eng, vector<float>& path_time, vector<int>& path_prize)
{
	if (!path_eng.empty()) { path_eng.clear(); } path_eng.reserve(bh_sol.size() + 1);
	if (!path_time.empty()) { path_time.clear(); } path_time.reserve(bh_sol.size() + 1);
	if (!path_prize.empty()) { path_prize.clear(); } path_prize.reserve(bh_sol.size() + 1);
	vector<Point> temp_pts; temp_pts.reserve(bh_sol.size());
	for (size_t i = 0; i < bh_sol.size(); i++) {
		temp_pts.push_back(sn_list[bh_sol[i]].getPt());
	}
	unique_ptr<Drone> uav = make_unique<Drone>(_init_coord, _uav_energy);
	uav->setWind(_wind_grid);
	unique_ptr<Point> target_pt = make_unique<Point>(
		_init_coord.getX(), _init_coord.getY(), uav->getNormAlt());
	
	uav->ascent(*target_pt, true);
	int cnt = 0;
	int prize = 0;
	do {
		int sn_idx = bh_sol[cnt];
		target_pt->setX(temp_pts[0].getX());
		target_pt->setY(temp_pts[0].getY());

		uav->steadyFlight(*target_pt, true);
		target_pt->setZ(uav->getHoverAlt());
		uav->descent(*target_pt, true);
		float wpt_cost = 0, wpt_charged = 0;
		uav->calcChrgPkg(sn_list[sn_idx], wpt_cost, wpt_charged);
		uav->setE(uav->getE() - wpt_cost);
		path_eng.push_back(uav->getE());
		path_time.push_back(uav->getT());
		path_prize.push_back(sn_list[sn_idx].getWt());

		target_pt->setZ(uav->getNormAlt());
		uav->ascent(*target_pt, true);

		temp_pts.erase(temp_pts.begin());
		cnt++;
	} while (!temp_pts.empty());

	uav->returnToHome(_end_coord);
	path_eng.push_back(uav->getE());
	path_time.push_back(uav->getT());
	path_prize.push_back(0);
}