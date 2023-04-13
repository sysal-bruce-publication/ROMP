#pragma once
#include <vector>
#include <algorithm>

#ifndef STAR_H
#define STAR_H

typedef std::pair<double, double> ValPair;
typedef std::pair<size_t, ValPair> IdxValpair;

class Star
{
public:
	Star(size_t num_nodes) 
	{
		reset_visit(num_nodes);
		path_E_T.reserve(num_nodes);
	}
	Star(const Star& star_k);
	~Star() {}

	double cost = 1e6;
	double prize = 0;
	double metric = 0;
	bool feasible = false;
	std::vector<IdxValpair> path_E_T;  // The node idx and time

	bool is_node_visit(size_t idx) const { return _visit[idx]; }
	bool is_all_visit() const
	{
		return std::all_of(_visit.begin(), _visit.end(), [](bool v) { return v; });
	}
	std::vector<bool> get_visit() const { return _visit; }

	void set_visit(const Star& star_k) { _visit = star_k.get_visit(); }
	void reset_visit(size_t num_nodes);
	void mask_visit(size_t node_id) { _visit[node_id] = true; }
	void unmask_visit(size_t node_id) { _visit[node_id] = false; }

private:
	const size_t _start_node = 0;
	const size_t _end_node = 1;
	std::vector<bool> _visit;
};


#endif // !STAR_H

