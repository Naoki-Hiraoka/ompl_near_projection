#include <ompl_near_projection/pNearKPIECE1.h>
#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection {
  namespace geometric {
    void pNearKPIECE1::threadSolve(unsigned int tid, const ompl::base::PlannerTerminationCondition &ptc,
                                   SolutionInfo *sol) {
      ompl::base::Goal *goal = pdef_->getGoal().get();
      auto *goal_s_near = dynamic_cast<NearGoalSpace *>(goal); // ここがKPIECE1と異なる.
      if (!goal_s_near) {
        OMPL_ERROR("%s: Goal is not NearGoalSpace!", getName().c_str());
        return;
      }

      ompl::geometric::Discretization<Motion>::Coord xcoord(projectionEvaluator_->getDimension());

      std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがKPIECE1と異なる
      if (!sampler_near) {
        OMPL_ERROR("%s: Sampler is not <NearProjectedStateSampler!", getName().c_str());
        return;
      }

      ompl::base::State *xstate = si_->allocState();

      while (sol->solution == nullptr && !ptc)
        {
          discLock_.lock();
          /*
            addMotion時:
                cell->data->score = (1.0 + log((double)(iteration_))) / (1.0 + dist);
            updateCell時:
                cell->data->importance = cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections)
            ここでは、iteration = 1, score = 1で固定で運用している
           */
          //disc_.countIteration();

          /* Decide on a state to expand from */
          Motion *existing = nullptr;
          ompl::geometric::Discretization<Motion>::Cell *ecell = nullptr;
          disc_.selectMotion(existing, ecell);
          discLock_.unlock();
          assert(existing);

          bool solv = false;

          /* sample random state (with goal biasing) */
          if (rng_.uniform01() < goalBias_) {
            // この部分がKPIECE1と異なる
            solv = goal_s_near->sampleTo(xstate, existing->state); // sampleToの出力へのmotionが存在する前提. checkMotionを省略することで高速化
          } else {
            // この部分がKPIECE1と異なる
            sampler_near->sampleUniformNearValid(xstate, existing->state, maxDistance_); // sampleUniformNearValidの出力へのmotionが存在する前提. checkMotionを省略することで高速化
          }

          /* create a motion */
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, xstate);
          motion->parent = existing;

          projectionEvaluator_->computeCoordinates(motion->state, xcoord);
          discLock_.lock();
          disc_.addMotion(motion, xcoord);  // this will also update the discretization heaps as needed, so no
          // call to updateCell() is needed
          discLock_.unlock();

          if (solv)
            {
              sol->lock.lock();
              sol->approxdif = 0.0;
              sol->solution = motion;
              sol->lock.unlock();
              break;
            }
          discLock_.lock();
          disc_.updateCell(ecell);
          discLock_.unlock();
        }

      si_->freeState(xstate);
    }

    ompl::base::PlannerStatus pNearKPIECE1::solve(const ompl::base::PlannerTerminationCondition &ptc) {
      checkValidity();

      ompl::geometric::Discretization<Motion>::Coord xcoord(projectionEvaluator_->getDimension());

      while (const ompl::base::State *st = pis_.nextStart())
        {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, st);
          projectionEvaluator_->computeCoordinates(motion->state, xcoord);
          disc_.addMotion(motion, xcoord, 1.0);
        }

      if (disc_.getMotionCount() == 0)
        {
          OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
          return ompl::base::PlannerStatus::INVALID_START;
        }

      if (!sampler_)
        sampler_ = si_->allocStateSampler();

      OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                  disc_.getMotionCount());

      SolutionInfo sol;
      sol.solution = nullptr;
      sol.approxsol = nullptr;
      sol.approxdif = std::numeric_limits<double>::infinity();

      std::vector<std::thread *> th(threadCount_);
      for (unsigned int i = 0; i < threadCount_; ++i)
        th[i] = new std::thread([this, i, &ptc, &sol]
                                {
                                  return threadSolve(i, ptc, &sol);
                                });
      for (unsigned int i = 0; i < threadCount_; ++i)
        {
          th[i]->join();
          delete th[i];
        }

      bool solved = false;
      bool approximate = false;
      if (sol.solution != nullptr)
        {
          lastGoalMotion_ = sol.solution;

          /* construct the solution path */
          std::vector<Motion *> mpath;
          while (sol.solution != nullptr)
            {
              mpath.push_back(sol.solution);
              sol.solution = sol.solution->parent;
            }

          /* set the solution path */
          auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
          for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

          pdef_->addSolutionPath(path, approximate, sol.approxdif, getName());
          solved = true;
        }

      OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(),
                  disc_.getMotionCount(), disc_.getCellCount(), disc_.getGrid().countInternal(),
                  disc_.getGrid().countExternal());

      return {solved, approximate};
    }
  };
};