//
// Created by bhuvanesh on 01.08.23.
//
#include "tag_blind_beliefpolicy.h"
using namespace despot;

namespace despot {
    TagBlindBeliefPolicy::TagBlindBeliefPolicy(const despot::LaserTag_U *model) :
            BeliefLowerBound(model),
            tag_model_(model) {
        const_cast<despot::LaserTag_U *>(tag_model_)->ComputeBlindAlpha();
    }

    despot::ValuedAction TagBlindBeliefPolicy::Value(const despot::Belief *belief) const {
        double bestValue = despot::Globals::NEG_INFTY;
        int bestAction = -1;
        for (despot::ACT_TYPE action = 0; action < tag_model_->NumActions(); action++) {
            double value = tag_model_->ComputeActionValue(
                    static_cast<const despot::ParticleBelief *>(belief), *tag_model_,
                    action);
            if (value > bestValue) {
                bestValue = value;
                bestAction = action;
            }
        }

        return despot::ValuedAction(bestAction, bestValue);
    }
}