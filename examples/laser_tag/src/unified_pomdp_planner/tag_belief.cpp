//
// Created by bhuvanesh on 31.07.23.
//

#include <tag_belief.h>
using namespace despot;
/* ==============================================================================
 * TagBelief_U class
 * ==============================================================================*/

TagBelief_U::TagBelief_U(std::vector<despot::State*> particles, const despot::LaserTag_U* model,
                         despot::Belief* prior) :
        despot::ParticleBelief(particles, model, prior, false),
        tag_model_(model) {
}

void TagBelief_U::Update(despot::ACT_TYPE action, despot::OBS_TYPE obs) {
    despot::Belief* updated = tag_model_->Tau(this, action, obs);

    for (int i = 0; i < despot::ParticleBelief::particles_.size(); i++)
        tag_model_->Free(despot::ParticleBelief::particles_[i]);
    despot::ParticleBelief::particles_.clear();

    const std::vector<despot::State*>& new_particles =
            static_cast<despot::ParticleBelief*>(updated)->particles();
    for (int i = 0; i < new_particles.size(); i++)
        despot::ParticleBelief::particles_.push_back(tag_model_->Copy(new_particles[i]));

    delete updated;
}