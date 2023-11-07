//
// Created by bhuvanesh on 31.07.23.
//
#include "unified_laser_tag.h"
#ifndef SRC_TAG_SHR_POLICY_H
#define SRC_TAG_SHR_POLICY_H

//#include "tag_state.h"
//#include "tag_belief.h"
//#include <despot/solver/pomcp.h>
//#include <despot/util/floor.h>
//#include <despot/util/coord.h>
//#include <cmath>
//#include <queue>


/* ==============================================================================
* TagSHRPolicy class
* ==============================================================================*/
namespace despot {
    using namespace despot;
    class TagSHRPolicy : public despot::DefaultPolicy { // Smart History-based Rollout
    const LaserTag_U *tag_model_;
    despot::Floor floor_;

    public:
        TagSHRPolicy(const despot::DSPOMDP *model, despot::ParticleLowerBound *bound) :
                DefaultPolicy(model, bound),
                tag_model_(static_cast<const despot::LaserTag_U *>(model)) {
            floor_ = tag_model_->floor();
//        cout<<"ISIDE TAGSHRPolciy"<<endl;
        }

        despot::ACT_TYPE Action(const std::vector<despot::State *> &particles, despot::RandomStreams &streams,
                                despot::History &history) const;

    };
}
#endif //SRC_TAG_SHR_POLICY_H
