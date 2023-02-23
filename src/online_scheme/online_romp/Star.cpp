#include "Star.h"

Star::Star(const Star& star_k)
{
	cost = star_k.cost;
	prize = star_k.prize;
	metric = star_k.metric;
	_visit = star_k.get_visit();
	path_E_T = star_k.path_E_T;
	feasible = star_k.feasible;
}

void Star::reset_visit(size_t num_nodes)
{
	if (!_visit.empty()) { _visit.clear(); }
	_visit.resize(num_nodes);
	_visit[_start_node] = _visit[_end_node] = true;
}
