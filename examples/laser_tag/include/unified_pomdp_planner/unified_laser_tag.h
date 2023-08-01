//
// Created by bhuvanesh on 25.07.23.
//

#ifndef DESPOT_WS1_UNIFIED_LASER_TAG_H
#define DESPOT_WS1_UNIFIED_LASER_TAG_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/floor.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>
#include "tag_state.h"

namespace despot {
/* ==============================================================================
 * UnifiedLaserTag class
 * ==============================================================================*/

    class LaserTag_U: public MDP,
                             public BeliefMDP,
                             public StateIndexer,
                             public StatePolicy,
                             public MMAPInferencer {
        friend class TagState_U;
        friend class TagSHRPolicy;
        friend class TagSPParticleUpperBound;
        friend class TagManhattanUpperBound;
    private:
        static int NBEAMS;
        static int BITS_PER_READING;

        double noise_sigma_;
        double unit_size_;
        std::vector<std::vector<std::vector<double> > > reading_distributions_;

    protected:

        std::vector<TagState_U*> states_;
        // rob_[s]: robot cell index for state s
        // opp_[s]: opponent cell index for state s

        std::vector<std::vector<std::vector<State> > > transition_probabilities_; //state, action, [state, weight]


        mutable MemoryPool<TagState_U> memory_pool_;

    protected:
        std::map<int, double> OppTransitionDistribution(int state) const;

        void ReadConfig(std::istream& is);
        void Init(std::istream& is);

        const TagState_U& MostLikelyState(const std::vector<State*>& particles) const;
        const State* GetMMAP(const std::vector<State*>& particles) const;
        void PrintTransitions() const;

    protected:
        std::string RandomMap(int height, int width, int obstacles);
        std::string BenchmarkMap();
        int NextRobPosition(int rob, int opp, ACT_TYPE a) const;

        mutable std::vector<int> default_action_;

    public:
        bool robot_pos_unknown_;
        static LaserTag_U* current_;

        LaserTag_U();
        LaserTag_U(std::string params_file);
//        virtual ~LaserTag_U(); // virtual ~BaseTag();
        double LaserRange(const State& state, int dir) const;
        void Init();
        int GetReading(int);
        void NoiseSigma(double noise_sigma);

        bool Step(State& state, double random_num, ACT_TYPE action, double& reward) const;
        bool Step(State& state, double random_num, ACT_TYPE action,
                  double& reward, OBS_TYPE& obs) const;
        inline int NumActions() const {
            return 5;
        }
        inline ACT_TYPE TagAction() const {
            return 4;
        }
        int NumStates() const;
        inline int GetIndex(const State* state) const {
            return state->state_id;
        }
        inline Coord GetRobPos(const State* state) const {
            return floor_.GetCell(rob_[state->state_id]);
        }
        inline int StateIndexToOppIndex(int index) const {
            return index % floor_.NumCells();
        }
        inline int StateIndexToRobIndex(int index) const {
            return index / floor_.NumCells();
        }
        inline int RobOppIndicesToStateIndex(int rob, int opp) const {
            return rob * floor_.NumCells() + opp;
        }
        inline const State* GetState(int index) const {
            return states_[index];
        }

        double ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const;
        const std::vector<State>& TransitionProbability(int s, ACT_TYPE a) const;
        double Reward(int s, ACT_TYPE a) const;

        State* CreateStartState(std::string type = "DEFAULT") const;
        Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

        inline double GetMaxReward() const {
            return TAG_REWARD;
        }
        ParticleUpperBound* CreateParticleUpperBound(std::string name = "DEFAULT") const;
        ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
                                                     std::string particle_bound_name = "DEFAULT") const;
        BeliefUpperBound* CreateBeliefUpperBound(std::string name = "DEFAULT") const;

        inline ValuedAction GetBestAction() const {
            return ValuedAction(0, -1);
        }
        ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
                                                     std::string particle_bound_name = "DEFAULT") const;
        BeliefLowerBound* CreateBeliefLowerBound(std::string name = "DEFAULT") const;

        void PrintState(const State& state, std::ostream& out = std::cout) const;
        void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
        void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
        void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

        static int GetReading(OBS_TYPE obs, OBS_TYPE dir);
        static void SetReading(OBS_TYPE& obs, OBS_TYPE reading, OBS_TYPE dir);
        int GetBucket(double noisy) const;

        State* Allocate(int state_id, double weight) const;
        State* Copy(const State* particle) const;
        void Free(State* particle) const;
        int NumActiveParticles() const;

        const Floor& floor() const;

        int MostLikelyAction(const std::vector<State*>& particles) const;

        void ComputeDefaultActions(std::string type) const;
        int GetAction(const State& tagstate) const;

        Belief* Tau(const Belief* belief, ACT_TYPE action, OBS_TYPE obs) const;
        void Observe(const Belief* belief, ACT_TYPE action,
                     std::map<OBS_TYPE, double>& obss) const;
        double StepReward(const Belief* belief, ACT_TYPE action) const;

        friend std::ostream& operator<<(std::ostream& os, const LaserTag_U& lasertag);

        Coord MostLikelyOpponentPosition(const std::vector<State*>& particles) const;

        Coord MostLikelyRobPosition(const std::vector<State*>& particles) const;

        OBS_TYPE same_loc_obs_;
        static double TAG_REWARD;
        Floor floor_;
        std::vector<int> opp_;
        std::vector<int> rob_;
    };

}

#endif //DESPOT_WS1_UNIFIED_LASER_TAG_H
