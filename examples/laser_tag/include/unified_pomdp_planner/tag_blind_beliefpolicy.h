//
// Created by bhuvanesh on 01.08.23.
//

#ifndef SRC_TAG_BLIND_BELIEFPOLICY_H
#define SRC_TAG_BLIND_BELIEFPOLICY_H

#include "tag_sp_particleupperbound.h"
#include "tag_manhattan_upperbound.h"
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
* TagBlindBeliefPolicy class
* ==============================================================================*/

    class TagBlindBeliefPolicy : public despot::BeliefLowerBound {
    private:
        std::vector<std::vector<double> > alpha_vectors_;
        const despot::LaserTag_U *tag_model_;

    public:
        TagBlindBeliefPolicy(const despot::LaserTag_U *model) ;
        despot::ValuedAction Value(const despot::Belief *belief) const;
    };
}
#endif //SRC_TAG_BLIND_BELIEFPOLICY_H
