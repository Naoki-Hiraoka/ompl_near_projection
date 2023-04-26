#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection{
  double NearGoalSpace::distanceGoal(const ompl::base::State *st) const {
    NearProjectedStateSpacePtr goalSpaceNear = std::static_pointer_cast<NearProjectedStateSpace>(goalSpace_); // goal spaceは、NearProjectedStateSpaceであるという想定. 本当はGoalSpace::setSpace時にcastして保管しておきたいのだが、GoalSpace::setSpaceがvirtual関数として宣言されていないのでできなかった.

    return goalSpaceNear->getNearConstraint()->distance(st);
  }

  bool NearGoalSpace::sampleTo(ompl::base::State *state, const ompl::base::State *source, double* distance) const{
    NearProjectedStateSpacePtr goalSpaceNear = std::static_pointer_cast<NearProjectedStateSpace>(goalSpace_); // goal spaceは、NearProjectedStateSpaceであるという想定. 本当はGoalSpace::setSpace時にcastして保管しておきたいのだが、GoalSpace::setSpaceがvirtual関数として宣言されていないのでできなかった.
    si_->copyState(state, source);
    bool ret = goalSpaceNear->getNearConstraint()->projectNearValid(state, source, distance);
    si_->enforceBounds(state);
    return ret;
  };
};
