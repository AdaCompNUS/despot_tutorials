//
// Created by bhuvanesh on 31.07.23.
//

#ifndef SRC_TAG_BELIEF_H
#define SRC_TAG_BELIEF_H

#include "unified_laser_tag.h"
namespace despot {
/* ==============================================================================
 * TagBelief_U class
 * ==============================================================================*/

    class TagBelief_U : public despot::ParticleBelief {
        const despot::LaserTag_U *tag_model_;
    public:
        TagBelief_U(std::vector<despot::State *> particles, const despot::LaserTag_U *model, Belief *prior =
        NULL);

        void Update(despot::ACT_TYPE action, despot::OBS_TYPE obs);
    };

}

#endif //SRC_TAG_BELIEF_H
