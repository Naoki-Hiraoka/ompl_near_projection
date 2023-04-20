#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection{
  bool NearGoalSpace::sampleTo(ompl::base::State *state, const ompl::base::State *source) const{
    NearProjectedStateSpacePtr goalSpaceNear = std::static_pointer_cast<NearProjectedStateSpace>(goalSpace_); // goal spaceは、NearProjectedStateSpaceであるという想定. 本当はGoalSpace::setSpace時にcastして保管しておきたいのだが、GoalSpace::setSpaceがvirtual関数として宣言されていないのでできなかった.
    si_->copyState(state, source);
    bool ret = goalSpaceNear->getNearConstraint()->projectNearValid(state, source);
    si_->enforceBounds(state);
    return ret;
  };
};
