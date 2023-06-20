#ifndef OMPL_NEAR_PROJECTION_RRT_H_
#define OMPL_NEAR_PROJECTION_RRT_H_

#include <ompl/geometric/planners/rrt/RRT.h>

namespace ompl_near_projection {
    namespace geometric {
      class NearRRT : public ompl::geometric::RRT
      {
      public:
        NearRRT(const ompl::base::SpaceInformationPtr &si, bool addIntermediateStates = false) :
          RRT(si, addIntermediateStates) {
        }

        ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;


      };
    }
}

#endif
