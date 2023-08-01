//
// Created by bhuvanesh on 31.07.23.
//

#include "tag_state.h"
#include "tag_belief.h"
#include <despot/solver/pomcp.h>
#include <despot/util/floor.h>
#include <despot/util/coord.h>
#include "tag_shr_policy.h"

using namespace despot;
namespace despot {
    despot::ACT_TYPE TagSHRPolicy::Action(const std::vector<despot::State *> &particles, despot::RandomStreams &streams,
                                          despot::History &history) const {
//        cout<<"Got particles:"<<particles.size()<<endl;
//            if(particles.size()>1){
//                cout<<"Got particles:"<<particles.size()<<endl;
//            }
//        cout<<"ISIDE TAGSHRPolciy Action"<<endl;
// If history is empty then take a random move
        if (history.Size() == 0) {
            return despot::Random::RANDOM.NextInt(tag_model_->NumActions() - 1);
        }

// Compute rob position
        despot::Coord rob;
        if (tag_model_->same_loc_obs_ != floor_.NumCells()) {
//            cout<<"Getting MostLikelyRobotPosition"<<endl;
            rob = tag_model_->MostLikelyRobPosition(particles);
        } else {
            std::cout << "Getting Robot position from History LastObservation" << std::endl;
            rob = floor_.GetCell(history.LastObservation());
        }

// Compute opp position
        despot::Coord opp;
        opp = tag_model_->MostLikelyOpponentPosition(particles);

        double distance = despot::Coord::ManhattanDistance(rob, opp);
//        cout<<"The robot is at distance:"<<distance<<endl;

// If we just saw an opponent then TAG
        if (distance <= 1) {
            return tag_model_->TagAction();
        }
//        cout<<"The distance is "<<distance<<endl;
        std::vector<despot::ACT_TYPE> actions;

// Don't double back and don't go into walls
        for (int d = 0; d < 4; d++) {
            if (!despot::Compass::Opposite(d, history.LastAction())
                && floor_.Inside(rob + despot::Compass::DIRECTIONS[d])) {
                actions.push_back(d);
            }
        }

// Have to double back
        if (actions.size() == 0) {
            for (int d = 0; d < 4; d++) {
                if (floor_.Inside(rob + despot::Compass::DIRECTIONS[d]))
                    actions.push_back(d);
            }
        }

// Rob may be trapped by the obstacles
        if (actions.size() == 0)
            return 0;

        despot::ACT_TYPE action = actions[despot::Random::RANDOM.NextInt(actions.size())];
        return action;
    }
}