#include "algs.h"

using namespace std;

int Algs::randInt(int limit)
{
	unsigned seed = static_cast<int>(chrono::system_clock::now().time_since_epoch().count());
	static default_random_engine generator_int(seed);

	uniform_int_distribution<int> distribution(0, limit);
	return distribution(generator_int);
}

float Algs::randFloat()
{
	mt19937_64 rng;
	//! initialize the random number generator with time-dependent seed
	uint64_t timeSeed = chrono::high_resolution_clock::now().time_since_epoch().count();
	seed_seq ss{ uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
	rng.seed(ss);
	//! initialize a uniform distribution between 0 and 1
	uniform_real_distribution<float> unif(0, 1);

	return unif(rng);
}
