//
// Created by bhuvanesh on 31.07.23.
//

#ifndef SRC_TAG_SP_PARTICLEUPPERBOUND_H
#define SRC_TAG_SP_PARTICLEUPPERBOUND_H

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
* TagSPParticleUpperBound class
* ==============================================================================*/
    class TagSPParticleUpperBound : public despot::ParticleUpperBound { // Shortest path
    protected:
        const despot::LaserTag_U *tag_model_;
        std::vector<double> value_;
    public:
        TagSPParticleUpperBound(const despot::LaserTag_U *model);

        double Value(const despot::State &s) const;
    };
}
#endif //SRC_TAG_SP_PARTICLEUPPERBOUND_H
