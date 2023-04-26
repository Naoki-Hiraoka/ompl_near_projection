#ifndef OMPL_NEAR_PROJECTION_NEARGOALSPACE_H
#define OMPL_NEAR_PROJECTION_NEARGOALSPACE_H

#include <ompl/base/goals/GoalSpace.h>
#include <ompl_near_projection/NearProjectedStateSpace.h>

namespace ompl_near_projection{
  class NearGoalSpace : public ompl::base::GoalSpace {
  public:
    // goal spaceは、NearProjectedStateSpaceである必要がある
    NearGoalSpace(const ompl::base::SpaceInformationPtr &si) :
      GoalSpace(si)
    {
    }

    double distanceGoal(const ompl::base::State *st) const override;

    // goal spaceのconstraintは、「まずambientSpaceを満たす. 残りの自由度でGoalSpaceを満たす」というものである必要がある.
    // sourceはambientSpaceを満たしている必要がある.
    // sourceから、ambientSpaceを満たす範囲で可能な限りGoalSpaceに近づくようにprojectionし、結果をstateとして返す.
    // goalSpaceに到達しなくても、途中のstateをstateとして返す. 返り値はfalseになる.
    // 返り値のstateは、sourceからstateへの、ambientSpaceを満たすmotionが必ず存在するという保証がある
    // distanceには、返り値のstateのgoalまでの距離が入る
    virtual bool sampleTo(ompl::base::State *state, const ompl::base::State *source, double* distance = nullptr) const;

  };
};

#endif
