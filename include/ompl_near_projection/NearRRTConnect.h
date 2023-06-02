#ifndef OMPL_NEAR_PROJECTION_NEARRRTCONNECT_H
#define OMPL_NEAR_PROJECTION_NEARRRTCONNECT_H

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <mutex>

namespace ompl_near_projection {
  namespace geometric {
    class NearRRTConnect : public ompl::geometric::RRTConnect {
    public:
      NearRRTConnect(const ompl::base::SpaceInformationPtr &si, bool addIntermediateStates = false) :
        RRTConnect(si, addIntermediateStates)
      {}

      ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

    protected:
      /** \brief Grow a tree towards a random state */
      // この関数はthread safe.
      ompl::geometric::RRTConnect::GrowState growTreeNear(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion, std::mutex& treeLock);

      std::mutex tStartLock_;
      std::mutex tGoalLock_;
    };
  }
};

#endif
