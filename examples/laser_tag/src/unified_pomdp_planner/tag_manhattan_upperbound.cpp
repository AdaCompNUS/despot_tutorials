//
// Created by bhuvanesh on 31.07.23.
//
#include "tag_manhattan_upperbound.h"

namespace despot {
    TagManhattanUpperBound::TagManhattanUpperBound(const despot::LaserTag_U *model) :
            tag_model_(model) {
        despot::Floor floor = tag_model_->floor_;
        value_.resize(tag_model_->NumStates());
        for (int s = 0; s < tag_model_->NumStates(); s++) {
            despot::Coord rob = floor.GetCell(tag_model_->rob_[s]), opp = floor.GetCell(
                    tag_model_->opp_[s]);
            int dist = despot::Coord::ManhattanDistance(rob, opp);
            value_[s] = -(1 - despot::Globals::Discount(dist)) / (1 - despot::Globals::Discount())
                        + tag_model_->TAG_REWARD * despot::Globals::Discount(dist);
        }
    }

    double TagManhattanUpperBound::Value(const despot::State &s) const {
        const despot::TagState_U &state = static_cast<const despot::TagState_U &>(s);
        return value_[state.state_id];
    }

    double TagManhattanUpperBound::Value(const despot::Belief *belief) const {
        const std::vector<despot::State *> &particles =
                static_cast<const despot::ParticleBelief *>(belief)->particles();

        double value = 0;
        for (int i = 0; i < particles.size(); i++) {
            despot::State *particle = particles[i];
            const despot::TagState_U *state = static_cast<const despot::TagState_U *>(particle);
            value += state->weight * value_[state->state_id];
        }
        return value;
    }
}