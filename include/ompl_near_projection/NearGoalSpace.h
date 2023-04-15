#ifndef OMPL_NEAR_PROJECTION_NEARGOALSPACE_H
#define OMPL_NEAR_PROJECTION_NEARGOALSPACE_H

#include <ompl/base/goals/GoalSpace.h>
#include <ompl_near_projection/NearProjectedStateSpace.h>

namespace ompl_near_projection{
  class NearGoalSpace : public ompl::base::GoalSpace {
  public:
    NearGoalSpace(const ompl::base::SpaceInformationPtr &si) : GoalSpace(si) {}

    // sourceをGoalSpaceのconstraintへprojectionしたものをstateとして返す.
    // projection失敗してもそのままprojection後の値をstateとして返す. 返り値はfalseになる. stateがStateSpaceを満たすためには、sourceがStateSpaceを満たしていて、GoalSpaceのprojectionが「StateSpaceの制約を満たす範囲内で可能な限りGoalSpaceへprojetionする」という仕様である必要がある.
    bool sampleTo(ompl::base::State *state, const ompl::base::State *source) const;
  };
};

#endif
