#ifndef LASERTAG_H
#define LASERTAG_H

#include "base/base_tag.h"

/* =============================================================================
 * LaserTag class
 * =============================================================================*/

namespace despot {

class LaserTag: public BaseTag {
private:
	static int NBEAMS;
	static int BITS_PER_READING;

	double noise_sigma_;
	double unit_size_;
	std::vector<std::vector<std::vector<double> > > reading_distributions_;

public:
	LaserTag();
	LaserTag(std::string params_file);
	LaserTag(std::string params_file, double noise_sigma);
	double LaserRange(const State& state, int dir) const;
	void Init();
	int GetReading(int);
	void NoiseSigma(double noise_sigma);

	bool Step(State& state, double random_num, ACT_TYPE action, double& reward) const;
	bool Step(State& state, double random_num, ACT_TYPE action,
		double& reward, OBS_TYPE& obs) const;
	double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;

	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;

	static int GetReading(OBS_TYPE obs, OBS_TYPE dir);
	static void SetReading(OBS_TYPE& obs, OBS_TYPE reading, OBS_TYPE dir);
	int GetBucket(double noisy) const;

	void Observe(const Belief* belief, ACT_TYPE action, std::map<OBS_TYPE, double>& obss) const;

	friend std::ostream& operator<<(std::ostream& os, const LaserTag& lasertag);
};

} // namespace despot

#endif
