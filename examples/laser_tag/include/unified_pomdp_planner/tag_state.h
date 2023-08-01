//
// Created by bhuvanesh on 31.07.23.
//

#ifndef SRC_TAG_STATE_H
#define SRC_TAG_STATE_H

#include <despot/core/particle_belief.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/util/floor.h>
#include <despot/util/coord.h>
#include <despot/core/mdp.h>
#include <despot/interface/pomdp.h>

namespace despot {
/* ==============================================================================
 * TagState_U class
 * ==============================================================================*/

    class TagState_U : public despot::State {
    public:
        TagState_U();

        TagState_U(int _state_id);

        std::string text() const;
    };
}
#endif //SRC_TAG_STATE_H
