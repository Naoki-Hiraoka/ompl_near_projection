#ifndef OMPL_NEAR_PROJECTION_PNEARKRRTCONNECT_H
#define OMPL_NEAR_PROJECTION_PNEARKRRTCONNECT_H

#include <ompl_near_projection/NearRRTConnect.h>
#include <thread>

namespace ompl_near_projection {
  namespace geometric {
    class pNearRRTConnect : public NearRRTConnect {
    public:
      pNearRRTConnect(const ompl::base::SpaceInformationPtr &si, bool addIntermediateStates = false) :
        NearRRTConnect(si, addIntermediateStates)
      {}

      ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

      void setThreadCount(unsigned int nthreads) {
        assert(nthreads > 0);
        threadCount_ = nthreads;
      }

      unsigned int getThreadCount() const {
        return threadCount_;
      }

      void setSleepUs(unsigned int us) {
        sleepUs_ = us;
      }
      unsigned int getSleepUs() {
        return sleepUs_;
      }

    protected:
      struct SolutionInfo
      {
        Motion *startSolution;
        Motion *goalSolution;
        Motion *approxsol;
        double approxdif;
        std::mutex lock;
      };

      void threadSolve(unsigned int tid, const ompl::base::PlannerTerminationCondition &ptc, SolutionInfo *sol);

      std::mutex discLock_;
      unsigned int threadCount_ = 2;
      unsigned int sleepUs_ = 10 * 1000;

      std::mutex startTreeLock_;
    };
  }
};

#endif
