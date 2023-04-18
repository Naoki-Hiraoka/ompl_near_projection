#include <ompl_near_projection/NearConstrainedSpaceInformation.h>

namespace ompl_near_projection{

  bool NearConstrainedValidStateSampler::sampleNearValid(ompl::base::State *state, const ompl::base::State *near, double distance){
    nearSampler_->sampleUniformNearValid(state, near, distance);
    return true;
  }

};
