#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection{
  bool NearGoalSpace::sampleTo(ompl::base::State *state, const ompl::base::State *source) const{
    ompl::base::ConstrainedStateSpacePtr goal_s = std::dynamic_pointer_cast<ompl::base::ConstrainedStateSpace>(goalSpace_);
    if(goal_s != nullptr){
      si_->copyState(state, source);
      bool ret = goal_s->getConstraint()->project(state);
      si_->enforceBounds(state);
      return ret;
    }else{
      sampleGoal(state);
      return true;
    }
  };
};
