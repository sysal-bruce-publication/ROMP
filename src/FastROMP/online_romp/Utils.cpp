#include "Utils.h"

using std::vector;

size_t Utils::sample_rand_num(size_t min_val, size_t max_val) const
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> distb(
		static_cast<int>(min_val), static_cast<int>(max_val));
	return static_cast<size_t>(distb(gen));
}

double Utils::sample_rand_num(const double min_val, const double max_val) const
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<double> distb(min_val, max_val);
	return distb(gen);
}

void Utils::calc_distance_matrix(const vector<Sensor>& sn,
	vector<vector<double>>& dist_mat) const
{
	dist_mat = vector<vector<double>>(sn.size(), vector<double>(sn.size()));
	double dist = 0;
	for (size_t i = 0; i < dist_mat.size() - 1; i++) {
		for (size_t j = i + 1; j < dist_mat.size(); j++) {
			dist = (sn[i].get_pt() - sn[j].get_pt()).get_xy_norm2();
			dist_mat[i][j] = dist_mat[j][i] = dist;
		}
	}
}

void Utils::vector_unbiased_sample(const vector<size_t>& vec_in, 
	const size_t num_elems, vector<size_t>& vec_out) const
{
	std::sample(vec_in.begin(), vec_in.end(), std::back_inserter(vec_out),
		num_elems, std::mt19937{ std::random_device{}() });
}

double Utils::_get_delta_cost(const vector<vector<double>>& cost_mat,
	const vector<size_t>& path, const size_t v1, const size_t v2) const
{
	return cost_mat[path[v1]][path[v2]]
		- cost_mat[path[v1]][path[(v1 + 1) % path.size()]]
		+ cost_mat[path[(v1 + 1) % path.size()]][path[(v2 + 1) % path.size()]]
		- cost_mat[path[v2]][path[(v2 + 1) % path.size()]];
}

void Utils::two_opt(const vector<vector<double>>& cost_mat, 
	size_t max_srch, vector<size_t>& path)
{
	bool improved = true;
	size_t cnt = 0;
	double cost_diff = 0;

	while (improved && cnt < max_srch) {
		improved = false;
		for (size_t i = 1; i < path.size() - 2; i++) {  // Include end node
			for (size_t j = i + 1; j < path.size() - 1; j++) {
				cost_diff = _get_delta_cost(cost_mat, path, i, j);
				if (cost_diff < 0) {
					vec_reverse(path, i, j);
					improved = true;
					cnt++;
				}                                                    
			}
		}
	}
}

double Utils::find_vec_up_quart(vector<double> vec) const
{
	size_t v_len = static_cast<size_t>(vec.size() * 0.75);
	std::nth_element(vec.begin(), vec.begin() + v_len, vec.end(), std::greater{});
	return vec[v_len];
}