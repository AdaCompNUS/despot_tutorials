//
// Created by bhuvanesh on 31.07.23.
//

#include "tag_belief.h"
#include <despot/solver/pomcp.h>
#include <despot/util/floor.h>
#include <despot/util/coord.h>
#include <cmath>
#include <unified_laser_tag.h>
#include <queue>
#include "tag_state.h"

namespace despot {
/* ==============================================================================
 * TagState_U class
 * ==============================================================================*/

    despot::TagState_U::TagState_U() = default;

    despot::TagState_U::TagState_U(int _state_id) {
        state_id = _state_id;
    }

    std::string despot::TagState_U::text() const {
        // NOTE: This will fail if several Tag instances are running
        if (LaserTag_U::current_ != nullptr) {
            int rob = LaserTag_U::current_->StateIndexToRobIndex(state_id);
            Coord rob_pos = LaserTag_U::current_->floor_.GetCell(rob);
            int opp = LaserTag_U::current_->StateIndexToOppIndex(state_id);
            Coord opp_pos = LaserTag_U::current_->floor_.GetCell(opp);
            return "Rob at " + to_string(rob_pos) + ", Opp at " + to_string(opp_pos);
        } else
            return to_string(state_id);
    }
}