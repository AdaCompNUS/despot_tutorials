//
// Created by bhuvanesh on 25.07.23.
//
#include <queue>
#include <unified_laser_tag.h>
#include <cmath>
#include <despot/util/coord.h>
#include <despot/util/floor.h>
#include <despot/solver/pomcp.h>
#include "tag_belief.h"
#include "tag_state.h"
#include "tag_shr_policy.h"
#include "tag_manhattan_upperbound.h"
#include "tag_sp_particleupperbound.h"
#include "tag_blind_beliefpolicy.h"

using namespace std;

namespace despot {
    LaserTag_U* LaserTag_U::current_ = nullptr;

    /* ==============================================================================
 * LaserTag_U class
 * ==============================================================================*/
    double LaserTag_U::TAG_REWARD = 10;

    const OBS_TYPE ONE = 1;
    int LaserTag_U::NBEAMS = 8;
    int LaserTag_U::BITS_PER_READING = 7;

    string LaserTag_U::RandomMap(int height, int width, int obstacles) {
        string map(height * (width + 1) - 1, '.');
        for (int h = 1; h < height; h++)
            map[h * (width + 1) - 1] = '\n';

        for (int i = 0; i < obstacles;) {
            int p = Random::RANDOM.NextInt(map.length());
            if (map[p] != '\n' && map[p] != '#') {
                map[p] = '#';
                i++;
            }
        }

        return "mapSize = " + to_string(height) + " " + to_string(width) + "\n" + to_string(map);
    }
    string LaserTag_U::BenchmarkMap() {
        int height=7; int width=11; int obstacles=8;
        string map(height * (width + 1) - 1, '.');
        for (int h = 1; h < height; h++)
            map[h * (width + 1) - 1] = '\n';

        int obstacles_list [] = {1+2*(width+1), 3+4*(width+1), 3+0*(width+1), 5+0*(width+1), 6+4*(width+1), 9+4*(width+1), 9+1*(width+1), 10+6*(width+1)};
        for (int i = 0; i < obstacles;) {
            int p = obstacles_list[i];
            assert (map[p] != '\n' && map[p] != '#');
            map[p] = '#';
            i++;
        }

        return "mapSize = " + to_string(height) + " " + to_string(width) + "\n" + to_string(map);
    }

    void LaserTag_U::ReadConfig(istream& is) {
        string line, key, val;
        while (is >> key >> val) {
            if (key == "mapSize") {
                int nrows, ncols;
                is >> nrows >> ncols;
                floor_ = Floor(nrows, ncols);

                for (int y = 0; y < nrows; y++) {
                    is >> line;
                    cout<<line<<endl;
                    for (int x = 0; x < ncols; x++) {
                        if (line[x] != '#') {
                            floor_.AddCell(Coord(x, y));
                        }
                    }
                }

                floor_.ComputeDistances();
            } else if (key == "width-height-obstacles") {
                int h, w, o;
                is >> h >> w >> o;
                istringstream iss(RandomMap(h, w, o));
                ReadConfig(iss);
            }
        }
    }

    LaserTag_U::LaserTag_U():
        noise_sigma_(0.5),
        unit_size_(1.0) {
        current_ = this;
        istringstream iss(BenchmarkMap());
        Init(iss);
        robot_pos_unknown_ = false;
    }

    LaserTag_U::LaserTag_U(string params_file) :
        noise_sigma_(0.5),
        unit_size_(1.0) {
        current_ = this;
        ifstream fin(params_file.c_str(), ifstream::in);
        Init(fin); //not sure whether this will be called
        robot_pos_unknown_ = false;
    }
//    LaserTag_U::LaserTag_U(string params_file, double noise_sigma) :
//        unit_size_(1.0) {
//        Init();
//        robot_pos_unknown_ = false;
//        noise_sigma_ = noise_sigma;
//    }
    double LaserTag_U::LaserRange(const State& state, int dir) const {
        Coord rob = floor_.GetCell(rob_[state.state_id]), opp = floor_.GetCell(
                opp_[state.state_id]);
        int d = 1;
        while (true) {
            Coord coord = rob + Compass::DIRECTIONS[dir] * d;
            if (floor_.GetIndex(coord) == -1 || coord == opp)
                break;
            d++;
        }
        int x = Compass::DIRECTIONS[dir].x, y = Compass::DIRECTIONS[dir].y;

        return d * sqrt(x * x + y * y);
    }

    void LaserTag_U::Init(istream& is) {
        ReadConfig(is);
        TagState_U* state;
        states_.resize(NumStates());
        rob_.resize(NumStates());
        opp_.resize(NumStates());
        for (int rob = 0; rob < floor_.NumCells(); rob++) {
            for (int opp = 0; opp < floor_.NumCells(); opp++) {
                int s = RobOppIndicesToStateIndex(rob, opp);
                state = new TagState_U(s);
                states_[s] = state;
                rob_[s] = rob;
                opp_[s] = opp;
            }
        }

        // Build transition matrix
        transition_probabilities_.resize(NumStates());
        for (int s = 0; s < NumStates(); s++) {
            transition_probabilities_[s].resize(NumActions());

            const map<int, double>& opp_distribution = OppTransitionDistribution(s);
            for (int a = 0; a < NumActions(); a++) {
                transition_probabilities_[s][a].clear();

                int next_rob = NextRobPosition(rob_[s], opp_[s], a);

                if (!(a == TagAction() && Coord::ManhattanDistance(floor_.GetCell(rob_[s]),floor_.GetCell(opp_[s]))<=1)) { // No transition upon termination
                    for (map<int, double>::const_iterator it = opp_distribution.begin();
                         it != opp_distribution.end(); it++) {
                        State next;
                        next.state_id = RobOppIndicesToStateIndex(next_rob,
                                                                  it->first);
                        next.weight = it->second;

                        transition_probabilities_[s][a].push_back(next);
                    }
                }
            }
        }

        for (int i = 0; i < NBEAMS; i++)
            SetReading(same_loc_obs_, 101, i);

        reading_distributions_.resize(NumStates());

        for (int s = 0; s < NumStates(); s++) {
            reading_distributions_[s].resize(NBEAMS);

            for (int d = 0; d < NBEAMS; d++) {
                double dist = LaserRange(*states_[s], d);
                for (int reading = 0; reading < dist / unit_size_; reading++) {
                    double min_noise = reading * unit_size_ - dist;
                    double max_noise = min(dist, (reading + 1) * unit_size_) - dist;
                    double prob =
                            2
                            * (gausscdf(max_noise, 0, noise_sigma_)
                               - (reading > 0 ?
                                  gausscdf(min_noise, 0, noise_sigma_) : 0 /*min_noise = -infty*/));

                    reading_distributions_[s][d].push_back(prob);
                }
            }
        }
    }
//    LaserTag_U::~LaserTag_U() {
//    }

    bool LaserTag_U::Step(State& s, double random_num, ACT_TYPE action,
                       double& reward) const {

        Random random(random_num);
        random_num = random.NextDouble();

        TagState_U& state = static_cast<TagState_U&>(s);

        bool terminal = false;
        if (action == TagAction()) {
            double distance=Coord::ManhattanDistance(floor_.GetCell(rob_[state.state_id]),floor_.GetCell(opp_[state.state_id]));
            if (distance<=1) {
                reward = TAG_REWARD;
                terminal = true;
            } else {
                reward = -TAG_REWARD;
            }
        } else {
            reward = -1;
        }

        const vector<State>& distribution =
                transition_probabilities_[state.state_id][action];
        double sum = 0;
        for (int i = 0; i < distribution.size(); i++) {
            const State& next = distribution[i];
            sum += next.weight;
            if (sum >= random_num) {
                state.state_id = next.state_id;
                break;
            }
        }

        return terminal;
    }

    bool LaserTag_U::Step(State& state, double random_num, ACT_TYPE action, double& reward,
                        OBS_TYPE& obs) const {
        Random random(random_num);
        bool terminal = Step(state, random.NextDouble(), action, reward);

        if (terminal) {
            obs = same_loc_obs_;
        } else {
            if (rob_[state.state_id] == opp_[state.state_id])
                obs = same_loc_obs_;
            else {
                const vector<vector<double> >& distribution = reading_distributions_[state.state_id];

                obs = 0;
                for (int dir = 0; dir < NBEAMS; dir++) {
                    double mass = random.NextDouble();
                    int reading = 0;
                    for (; reading < distribution[dir].size(); reading++) {
                        mass -= distribution[dir][reading];
                        if (mass < Globals::TINY)
                            break;
                    }
                    SetReading(obs, reading, dir);
                }
            }
        }

        return terminal;
    }

    double LaserTag_U::ObsProb(OBS_TYPE obs, const State& state, ACT_TYPE action) const {
        if (rob_[state.state_id] == opp_[state.state_id])
            return obs == same_loc_obs_;

        double prod = 1.0;
        for (int dir = 0; dir < NBEAMS; dir++) {
            int reading = GetReading(obs, dir);
            if (reading >= LaserRange(state, dir) / unit_size_)
                return 0;
            double prob_mass = reading_distributions_[state.state_id][dir][reading];
            prod *= prob_mass;
        }

        return prod;
    }

    int LaserTag_U::NumStates() const {
        return floor_.NumCells() * floor_.NumCells();
    }

    const vector<State>& LaserTag_U::TransitionProbability(int s, ACT_TYPE a) const {
        return transition_probabilities_[s][a];
    }

    int LaserTag_U::NextRobPosition(int rob, int opp, ACT_TYPE a) const {
        Coord pos = floor_.GetCell(rob) + Compass::DIRECTIONS[a];
        if (a != TagAction() && floor_.Inside(pos) && pos!=floor_.GetCell(opp))
            return floor_.GetIndex(pos);

        return rob;
    }

    const Floor& LaserTag_U::floor() const {
        return floor_;
    }
    map<int, double> LaserTag_U::OppTransitionDistribution(int state) const {
        Coord rob = floor_.GetCell(rob_[state]), opp = floor_.GetCell(opp_[state]);

        map<int, double> distribution;

        if (opp.x == rob.x) {
            int index =
                    floor_.Inside(opp + Coord(1, 0)) ?
                    floor_.GetIndex(opp + Coord(1, 0)) : floor_.GetIndex(opp);
            distribution[index] += 0.2;

            index =
                    floor_.Inside(opp + Coord(-1, 0)) ?
                    floor_.GetIndex(opp + Coord(-1, 0)) : floor_.GetIndex(opp);
            distribution[index] += 0.2;
        } else {
            int dx = opp.x > rob.x ? 1 : -1;
            int index =
                    floor_.Inside(opp + Coord(dx, 0)) ?
                    floor_.GetIndex(opp + Coord(dx, 0)) : floor_.GetIndex(opp);
            distribution[index] += 0.4;
        }

        if (opp.y == rob.y) {
            int index =
                    floor_.Inside(opp + Coord(0, 1)) ?
                    floor_.GetIndex(opp + Coord(0, 1)) : floor_.GetIndex(opp);
            distribution[index] += 0.2;

            index =
                    floor_.Inside(opp + Coord(0, -1)) ?
                    floor_.GetIndex(opp + Coord(0, -1)) : floor_.GetIndex(opp);
            distribution[index] += 0.2;
        } else {
            int dy = opp.y > rob.y ? 1 : -1;
            int index =
                    floor_.Inside(opp + Coord(0, dy)) ?
                    floor_.GetIndex(opp + Coord(0, dy)) : floor_.GetIndex(opp);
            distribution[index] += 0.4;
        }

        distribution[floor_.GetIndex(opp)] += 0.2;

        return distribution;
    }

    void LaserTag_U::PrintTransitions() const {
        for (int s = 0; s < NumStates(); s++) {
            cout << "State " << s << endl;
            PrintState(*GetState(s));
            for (int a = 0; a < NumActions(); a++) {
                cout << "Applying action " << a << " on " << endl;
                for (int i = 0; i < transition_probabilities_[s][a].size();
                     i++) {
                    const State& next = transition_probabilities_[s][a][i];
                    cout << s << "-" << a << "-" << next.state_id << "-"
                         << next.weight << endl;
                    PrintState(*GetState(next.state_id));
                }
            }
        }
    }

    State* LaserTag_U::CreateStartState(string type) const {
        int n = Random::RANDOM.NextInt(states_.size());
        return new TagState_U(*states_[n]);
    }

    ParticleUpperBound* LaserTag_U::CreateParticleUpperBound(string name) const {
        if (name == "TRIVIAL") {
            return  new TrivialParticleUpperBound(this);
        } else if (name == "MDP") {
            return new MDPUpperBound(this, *this);
        } else if (name == "SP" || name == "DEFAULT") {
            return new TagSPParticleUpperBound(this);
        } else if (name == "MANHATTAN") {
            return new TagManhattanUpperBound(this);
        } else {
            if (name != "print")
                cerr << "Unsupported particle lower bound: " << name << endl;
            cerr << "Supported types: TRIVIAL, MDP, SP, MANHATTAN (default to SP)" << endl;
            exit(1);
            return nullptr;
        }
    }
    ScenarioUpperBound* LaserTag_U::CreateScenarioUpperBound(string name,
                                                          string particle_bound_name) const {
        if (name == "TRIVIAL" || name == "DEFAULT" || name == "MDP" ||
            name == "SP" || name == "MANHATTAN") {
            return CreateParticleUpperBound(name);
        } else if (name == "LOOKAHEAD") {
            return new LookaheadUpperBound(this, *this,
                                           CreateParticleUpperBound(particle_bound_name));
        } else {
            if (name != "print")
                cerr << "Unsupported upper bound: " << name << endl;
            cerr << "Supported types: TRIVIAL, MDP, SP, MANHATTAN, LOOKAHEAD (default to SP)" << endl;
            cerr << "With base upper bound: LOOKAHEAD" << endl;
            exit(1);
            return NULL;
        }
    }
    BeliefUpperBound* LaserTag_U::CreateBeliefUpperBound(string name) const {
        if (name == "TRIVIAL") {
            return new TrivialBeliefUpperBound(this);
        } else if (name == "DEFAULT" || name == "MDP") {
            return new MDPUpperBound(this, *this);
        } else if (name == "MANHATTAN") {
            return new TagManhattanUpperBound(this);
        } else {
            if (name != "print")
                cerr << "Unsupported belief upper bound: " << name << endl;
            cerr << "Supported types: TRIVIAL, MDP, MANHATTAN (default to MDP)" << endl;
            exit(1);
            return NULL;
        }
    }

    ScenarioLowerBound* LaserTag_U::CreateScenarioLowerBound(string name, string
    particle_bound_name) const {
        const DSPOMDP* model = this;
        const StateIndexer* indexer = this;
        const StatePolicy* policy = this;
        const MMAPInferencer* mmap_inferencer = this;
        if (name == "TRIVIAL") {
            return new TrivialParticleLowerBound(model);
        } else if (name == "RANDOM") {
            return new RandomPolicy(model,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "SHR"
                   || (name == "DEFAULT" && same_loc_obs_ != floor_.NumCells())) {
            return new TagSHRPolicy(model,
                                    CreateParticleLowerBound(particle_bound_name));
//        } else if (name == "HM") {
//            return new TagHistoryModePolicy(model,
//                                            CreateParticleLowerBound(particle_bound_name));
        } else if (name == "MMAP-MDP") {
            ComputeDefaultActions("MDP");
            return new MMAPStatePolicy(model, *mmap_inferencer,
                                       *policy, CreateParticleLowerBound(particle_bound_name));
        } else if (name == "MMAP-SP") {
            ComputeDefaultActions("SP");
            return new MMAPStatePolicy(model, *mmap_inferencer,
                                       *policy, CreateParticleLowerBound(particle_bound_name));
        } else if (name == "MODE-MDP"
                   || (name == "DEFAULT" && same_loc_obs_ == floor_.NumCells())) {
            ComputeDefaultActions("MDP");
            return new ModeStatePolicy(model, *indexer, *policy,
                                       CreateParticleLowerBound(particle_bound_name));
        } else if (name == "MODE-SP") {
            ComputeDefaultActions("SP");
            return new ModeStatePolicy(model, *indexer, *policy,
                                       CreateParticleLowerBound(particle_bound_name));
        } else if (name == "MAJORITY-MDP") {
            ComputeDefaultActions("MDP");
            return new MajorityActionPolicy(model, *policy,
                                            CreateParticleLowerBound(particle_bound_name));
        } else if (name == "MAJORITY-SP") {
            ComputeDefaultActions("SP");
            return new MajorityActionPolicy(model, *policy,
                                            CreateParticleLowerBound(particle_bound_name));
        } else {
            if (name != "print")
                cerr << "Unsupported lower bound: " << name << endl;
            cerr << "Supported types: TRIVIAL, RANDOM, SHR, MODE-MDP, MODE-SP, MAJORITY-MDP, MAJORITY-SP (default to MODE-MDP)" << endl;
            cerr << "With base lower bound: except TRIVIAL" << endl;
            exit(1);
            return NULL;
        }
    }

    BeliefLowerBound* LaserTag_U::CreateBeliefLowerBound(string name) const {
        if (name == "TRIVIAL") {
            return new TrivialBeliefLowerBound(this);
        } else if (name == "DEFAULT" || name == "BLIND") {
            return new TagBlindBeliefPolicy(this);
        } else {
            cerr << "Unsupported belief lower bound: " << name << endl;
            exit(1);
            return NULL;
        }
    }

    void LaserTag_U::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
        for (int i = 0; i < NBEAMS; i++)
            out << GetReading(obs, i) << " ";
        out << endl;
    }
    void LaserTag_U::PrintState(const State& s, ostream& out) const {
        const TagState_U& state = static_cast<const TagState_U&>(s);

        int aindex = rob_[state.state_id];
        int oindex = opp_[state.state_id];

        for (int y = floor_.num_rows()-1; y >= 0; y--) {
            for (int x = 0; x < floor_.num_cols(); x++) {
                int index = floor_.GetIndex(x, y);
                if (index == Floor::INVALID)
                    out << "#";
                else if (index == aindex && index == oindex)
                    out << "Q";
                else if (index == aindex)
                    out << "R";
                else if (index == oindex)
                    out << "O";
                else
                    out << ".";
            }
            out << endl;
        }
    }
    void LaserTag_U::PrintBelief(const Belief& belief, ostream& out) const {

        const vector<State*>& particles =
                static_cast<const ParticleBelief&>(belief).particles();

        int max_prob_id=-1;
        double max_prob=0;
        float rob_pos_map[11][7];	float opp_pos_map[11][7];
        for(int i=0;i<11;i++)
            for(int j=0;j<7;j++)
            {
                rob_pos_map[i][j]=0;
                opp_pos_map[i][j]=0;
            }
        for (int i = 0; i < particles.size(); i++) {
            State* particle = particles[i];
            if(particle->weight > max_prob){
                max_prob = particle->weight;
                max_prob_id = i;
            }
            int rob= rob_[static_cast<TagState_U*>(particle)->state_id];
            int opp= opp_[static_cast<TagState_U*>(particle)->state_id];
            Coord rob_pos=floor_.GetCell(rob);
            Coord opp_pos=floor_.GetCell(opp);
            rob_pos_map[rob_pos.x][rob_pos.y]+=particle->weight;
            opp_pos_map[opp_pos.x][opp_pos.y]+=particle->weight;
        }

        int rob= rob_[static_cast<TagState_U*>(particles[max_prob_id])->state_id];
        int opp= opp_[static_cast<TagState_U*>(particles[max_prob_id])->state_id];
        out << "Maximum likelihood robot position:";
        out << floor_.GetCell(rob) << endl;

        out << "Maximum likelihood target position:";
        out << floor_.GetCell(opp) << endl;

        out << "Robot position belief:" << endl;
        for(int j=0;j<7;j++){
            for(int i=0;i<11;i++){
                cout.precision(3);
                cout.width(6);
                cout  << rob_pos_map[i][6-j] << "  ";
            }
            cout<< endl;
        }
        out << "Target position belief:" << endl;
        for(int j=0;j<7;j++){
            for(int i=0;i<11;i++){
                cout.precision(3);
                cout.width(6);
                cout << opp_pos_map[i][6-j] << "  ";
            }
            cout<< endl;
        }
    }
    void LaserTag_U::PrintAction(ACT_TYPE action, ostream& out) const {
        switch(action) {
            case 0: out << "North" << endl; break;
            case 1: out << "East" << endl; break;
            case 2: out << "South" << endl; break;
            case 3: out << "West" << endl; break;
            case 4: out << "Tag" << endl; break;
            default: out << "Wrong action" << endl; exit(1);
        }
    }


    int LaserTag_U::GetReading(OBS_TYPE obs, OBS_TYPE dir) {
        return (obs >> (dir * BITS_PER_READING)) & ((ONE << BITS_PER_READING) - 1);
    }

    void LaserTag_U::SetReading(OBS_TYPE& obs, OBS_TYPE reading, OBS_TYPE dir) {
        // Clear bits
        obs &= ~(((ONE << BITS_PER_READING) - 1) << (dir * BITS_PER_READING));
        // Set bits
        obs |= reading << (dir * BITS_PER_READING);
    }

    int LaserTag_U::GetBucket(double noisy) const {
        return (int) std::floor(noisy / unit_size_);
    }

    Belief* LaserTag_U::InitialBelief(const State* start, string type) const {
        //assert(start != NULL);

        vector<State*> particles;
        int N = floor_.NumCells();
        double wgt = 1.0 / N / N;
        for (int rob = 0; rob < N; rob++) {
            for (int opp = 0; opp < N; opp++) {
                TagState_U* state = static_cast<TagState_U*>(Allocate(
                        RobOppIndicesToStateIndex(rob, opp), wgt));
                particles.push_back(state);
            }
        }
        cout<<"Initial belief particles size:"<<particles.size()<<endl; // 4761 particles - matching with Java
        ParticleBelief* belief = new ParticleBelief(particles, this);
        belief->state_indexer(this);
        return belief;
    }


    State* LaserTag_U::Allocate(int state_id, double weight) const {
        TagState_U* state = memory_pool_.Allocate();
        state->state_id = state_id;
        state->weight = weight;
        return state;
    }

    State* LaserTag_U::Copy(const State* particle) const {
        TagState_U* state = memory_pool_.Allocate();
        *state = *static_cast<const TagState_U*>(particle);
        state->SetAllocated();
        return state;
    }

    void LaserTag_U::Free(State* particle) const {
        memory_pool_.Free(static_cast<TagState_U*>(particle));
    }

    int LaserTag_U::NumActiveParticles() const {
        return memory_pool_.num_allocated();
    }


    void LaserTag_U::ComputeDefaultActions(string type) const {
        if (type == "MDP") {
            const_cast<LaserTag_U*>(this)->ComputeOptimalPolicyUsingVI();
            int num_states = NumStates();
            default_action_.resize(num_states);

            for (int s = 0; s < num_states; s++) {
                default_action_[s] = policy_[s].action;
            }
        } else if (type == "SP") {
            // Follow the shortest path from the robot to the opponent
            default_action_.resize(NumStates());
            for (int s = 0; s < NumStates(); s++) {
                default_action_[s] = 0;
                if (rob_[s] == opp_[s]) {
                    default_action_[s] = TagAction();
                } else {
                    double cur_dist = floor_.Distance(rob_[s], opp_[s]);
                    for (int a = 0; a < 4; a++) {
                        int next = NextRobPosition(rob_[s], opp_[s], a);
                        double dist = floor_.Distance(next, opp_[s]);

                        if (dist < cur_dist) {
                            default_action_[s] = a;
                            break;
                        }
                    }
                }
            }
        } else {
            cerr << "Unsupported default action type " << type << endl;
            exit(1);
        }
    }

    int LaserTag_U::GetAction(const State& state) const {
        return default_action_[GetIndex(&state)];
    }

    Coord LaserTag_U::MostLikelyOpponentPosition(
            const vector<State*>& particles) const {
        static vector<double> probs = vector<double>(floor_.NumCells());

        for (int i = 0; i < particles.size(); i++) {
            TagState_U* tagstate = static_cast<TagState_U*>(particles[i]);
            probs[opp_[tagstate->state_id]] += tagstate->weight;
        }

        double maxWeight = 0;
        int opp = -1;
        for (int i = 0; i < probs.size(); i++) {
            if (probs[i] > maxWeight) {
                maxWeight = probs[i];
                opp = i;
            }
            probs[i] = 0.0;
        }

        return floor_.GetCell(opp);
    }

    Coord LaserTag_U::MostLikelyRobPosition(const vector<State*>& particles) const {
        static vector<double> probs = vector<double>(floor_.NumCells());

        double maxWeight = 0;
        int rob = -1;
        for (int i = 0; i < particles.size(); i++) {
            TagState_U* tagstate = static_cast<TagState_U*>(particles[i]);
            int id = rob_[tagstate->state_id];
            probs[id] += tagstate->weight;
//        cout<<"Added ID:"<<id<<", weight:"<<tagstate->weight<<":"<<particles.size()<<endl;
            if (probs[id] > maxWeight) {
                maxWeight = probs[id];
                rob = id;
            }
        }

        for (int i = 0; i < probs.size(); i++) {
            probs[i] = 0.0;
        }
//    cout<<"Getting robot cell:"<<rob<<endl;
        return floor_.GetCell(rob);
    }

    const TagState_U& LaserTag_U::MostLikelyState(
            const vector<State*>& particles) const {
        static vector<double> probs = vector<double>(NumStates());

        double maxWeight = 0;
        int bestId = -1;
        for (int i = 0; i < particles.size(); i++) {
            TagState_U* tagstate = static_cast<TagState_U*>(particles[i]);
            int id = GetIndex(tagstate);
            probs[id] += tagstate->weight;

            if (probs[id] > maxWeight) {
                maxWeight = probs[id];
                bestId = id;
            }
        }

        for (int i = 0; i < particles.size(); i++) {
            TagState_U* tagstate = static_cast<TagState_U*>(particles[i]);
            probs[GetIndex(tagstate)] = 0;
        }

        return *states_[bestId];
    }

    const State* LaserTag_U::GetMMAP(const vector<State*>& particles) const {
        Coord rob = MostLikelyRobPosition(particles);
        Coord opp = MostLikelyOpponentPosition(particles);

        int state_id = RobOppIndicesToStateIndex(floor_.GetIndex(rob),
                                                 floor_.GetIndex(opp));
        return states_[state_id];
    }

    Belief* LaserTag_U::Tau(const Belief* belief, ACT_TYPE action, OBS_TYPE obs) const {
        static vector<double> probs = vector<double>(NumStates());

        const vector<State*>& particles =
                static_cast<const ParticleBelief*>(belief)->particles();

        double sum = 0;
        for (int i = 0; i < particles.size(); i++) {
            TagState_U* state = static_cast<TagState_U*>(particles[i]);
            const vector<State>& distribution = transition_probabilities_[GetIndex(
                    state)][action];
            for (int j = 0; j < distribution.size(); j++) {
                const State& next = distribution[j];
                double p = state->weight * next.weight
                           * ObsProb(obs, *(states_[next.state_id]), action);
                probs[next.state_id] += p;
                sum += p;
            }
        }

        vector<State*> new_particles;
        for (int i = 0; i < NumStates(); i++) {
            if (probs[i] > 0) {
                State* new_particle = Copy(states_[i]);
                new_particle->weight = probs[i] / sum;
                new_particles.push_back(new_particle);
                probs[i] = 0;
            }
        }
        cout<<"C++:Tau:ParticlesSize:"<<particles.size()<<endl;
        return new ParticleBelief(new_particles, this, NULL, false);
    }

    void LaserTag_U::Observe(const Belief* belief, ACT_TYPE action,
                           std::map<OBS_TYPE, double>& obss) const {
        cerr << "Exit: Two many observations!" << endl;
        exit(0);
    }

    double LaserTag_U::StepReward(const Belief* belief, ACT_TYPE action) const {
        const vector<State*>& particles =
                static_cast<const ParticleBelief*>(belief)->particles();
        cout<<"C++:StepReward:ParticleSize:"<<particles.size()<<endl;
        double sum = 0;
        for (int i = 0; i < particles.size(); i++) {
            State* particle = particles[i];
            TagState_U* state = static_cast<TagState_U*>(particle);
            double reward = 0;
            if (action == TagAction()) {
                if (rob_[state->state_id] == opp_[state->state_id]) {
                    reward = TAG_REWARD;
                } else {
                    reward = -TAG_REWARD;
                }
            } else {
                reward = -1;
            }
            sum += state->weight * reward;
        }

        return sum;
    }

    double LaserTag_U::Reward(int s, ACT_TYPE action) const {
        const TagState_U* state = states_[s];
        double reward = 0;
        if (action == TagAction()) {
            if (rob_[state->state_id] == opp_[state->state_id]) {
                reward = TAG_REWARD;
            } else {
                reward = -TAG_REWARD;
            }
        } else {
            reward = -1;
        }
        cout<<"C++:Reward("<<s<<","<<action<<")="<<reward<<endl;
        return reward;
    }

    ostream& operator<<(ostream& os, const LaserTag_U& lasertag) {
        for (int s = 0; s < lasertag.NumStates(); s++) {
            os << "State " << s << " " << lasertag.opp_[s] << " "
               << lasertag.rob_[s] << " " << lasertag.floor_.NumCells() << endl;
            lasertag.PrintState(*lasertag.states_[s], os);

            for (int d = 0; d < lasertag.NBEAMS; d++) {
                os << d;
                for (int i = 0; i < lasertag.reading_distributions_[s][d].size(); i++)
                    os << " " << lasertag.reading_distributions_[s][d][i];
                os << endl;
            }
        }
        return os;
    }

    void LaserTag_U::NoiseSigma(double noise_sigma)
    {
        noise_sigma_ = noise_sigma;
    }
}