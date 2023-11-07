//
// Created by bhuvanesh on 31.07.23.
//
#include "tag_sp_particleupperbound.h"

namespace despot {
using namespace despot;

    TagSPParticleUpperBound::TagSPParticleUpperBound (const despot::LaserTag_U* model) :
            tag_model_(model) {
        despot::Floor floor = tag_model_->floor_;
        value_.resize(tag_model_->NumStates());
        for (int s = 0; s < tag_model_->NumStates(); s++) {
            int rob = tag_model_->rob_[s], opp = tag_model_->opp_[s];
            int dist = (int) floor.Distance(rob, opp);
            value_[s] = -(1 - despot::Globals::Discount(dist)) / (1 - despot::Globals::Discount())
                        + tag_model_->TAG_REWARD * despot::Globals::Discount(dist);
        }
    }

    double TagSPParticleUpperBound::Value(const despot::State& s) const {
        const despot::TagState_U& state = static_cast<const despot::TagState_U&>(s);

        return value_[state.state_id];
    }
}