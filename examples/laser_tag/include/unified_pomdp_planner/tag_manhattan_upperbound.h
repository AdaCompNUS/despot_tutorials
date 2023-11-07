//
// Created by bhuvanesh on 31.07.23.
//

#ifndef SRC_TAG_MANHATTAN_UPPERBOUND_H
#define SRC_TAG_MANHATTAN_UPPERBOUND_H

#include "tag_shr_policy.h"
#include "tag_state.h"
#include "tag_belief.h"
#include <despot/solver/pomcp.h>
#include <despot/util/floor.h>
#include <despot/util/coord.h>
#include <cmath>
#include <unified_laser_tag.h>
#include <queue>
namespace despot {
/* ==============================================================================
* TagManhattanUpperBound class
* ==============================================================================*/

    class TagManhattanUpperBound : public despot::ParticleUpperBound, public despot::BeliefUpperBound {
    protected:
        const despot::LaserTag_U *tag_model_;
        std::vector<double> value_;
    public:
        TagManhattanUpperBound(const despot::LaserTag_U *model);

        using ParticleUpperBound::Value;

        double Value(const despot::State &s) const ;

        using BeliefUpperBound::Value;

        double Value(const despot::Belief *belief) const ;
    };
}
#endif //SRC_TAG_MANHATTAN_UPPERBOUND_H
