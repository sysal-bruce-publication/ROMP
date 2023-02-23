#pragma once
#include <random>
#include <vector>
#include "Sensor.h"

#ifndef UTILS_H
#define UTILS_H

class Utils
{
public:
	Utils() {}
	~Utils() {}

	size_t sample_rand_num(size_t min_val, size_t max_val) const;
	double sample_rand_num(double min_val, double max_val) const;
	void calc_distance_matrix(const std::vector<Sensor>& sensors, 
		std::vector<std::vector<double>>& dist_mat) const;
	void vector_unbiased_sample(const std::vector<size_t>& vec_in, 
		const size_t num_elems, std::vector<size_t>& out) const;
	template <typename T, typename A>
	inline void vec_reverse(std::vector<T, A>& vec,
		const std::size_t v1, const std::size_t v2)
	{
		std::reverse(vec.begin() + v1 + 1, vec.begin() + v2 + 1);
	}
	void two_opt(const std::vector<std::vector<double>>& cost_mat,
		size_t max_srch, std::vector<size_t>& path);
	double find_vec_up_quart(std::vector<double> vec) const;
private:
	double _get_delta_cost(const std::vector<std::vector<double>>& cost_mat,
		const std::vector<size_t>& path, const size_t v1, const size_t v2) const;
};

#endif // !UTILS_H

