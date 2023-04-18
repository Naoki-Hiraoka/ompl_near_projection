#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection{
  bool NearGoalSpace::sampleTo(ompl::base::State *state, const ompl::base::State *source) const{
    NearProjectedStateSpacePtr goalSpaceNear = std::static_pointer_cast<NearProjectedStateSpace>(goalSpace_); // goal spaceは、NearProjectedStateSpaceであるという想定

    bool ret = goalSpaceNear->getNearConstraint()->projectNearValid(state, source);
    si_->enforceBounds(state);
    return ret;
  };
};
