#include <ompl_near_projection/pNearEST.h>
#include <ompl_near_projection/NearGoalSpace.h>
#include <ompl_near_projection/NearConstrainedSpaceInformation.h>

namespace ompl_near_projection {
  namespace geometric {
    void pNearEST::threadSolve(unsigned int tid, const ompl::base::PlannerTerminationCondition &ptc,
                                   SolutionInfo *sol) {
      ompl::base::Goal *goal = pdef_->getGoal().get();
      auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);
      auto *goal_s_near = dynamic_cast<NearGoalSpace *>(goal_s); // ここがESTと異なる.

      std::vector<Motion *> neighbors;

      std::shared_ptr<NearConstrainedValidStateSampler> sampler_near = std::dynamic_pointer_cast<NearConstrainedValidStateSampler>(sampler_); // ここがESTと異なる

      while(sol->solution == nullptr && !ptc) {
        if(motions_.size() < tid + 1) usleep(sleepUs_); // これがないと、start state近くの点を過剰にsamplingしてしまってtree全体が歪む
        else break;
      }

      ompl::base::State *xstate = si_->allocState();
      auto *xmotion = new Motion();

      while (sol->solution == nullptr && !ptc)
        {
          // Select a state to expand from
          motionLock_.lock();
          Motion *existing = pdf_.sample(rng_.uniform01());
          motionLock_.unlock();
          assert(existing);

          bool keep = false;

          // Sample random state in the neighborhood (with goal biasing)
          if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            {
              if (goal_s_near != nullptr) {
                // この部分がESTと異なる
                goal_s_near->sampleTo(xstate, existing->state);
                keep = true; // sampleToの出力へのmotionが存在する前提. checkMotionを省略することで高速化
              } else {
                goal_s->sampleGoal(xstate);
                keep = si_->checkMotion(existing->state, xstate);
              }

              // Compute neighborhood of candidate motion
              xmotion->state = xstate;
              motionLock_.lock();
              nn_->nearestR(xmotion, nbrhoodRadius_, neighbors);
              motionLock_.unlock();
            }
          else
            {
              if(sampler_near) {
                // この部分がKPIECE1と異なる
                sampler_near->sampleNearValid(xstate, existing->state, maxDistance_);
                keep = true; // sampleNearValidの出力へのmotionが存在する前提. checkMotionを省略することで高速化
              }else{
                // Sample a state in the neighborhood
                if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
                  continue;
                keep = si_->checkMotion(existing->state, xstate);
              }

              // Compute neighborhood of candidate state
              xmotion->state = xstate;
              motionLock_.lock();
              nn_->nearestR(xmotion, nbrhoodRadius_, neighbors);
              motionLock_.unlock();

              // reject state with probability proportional to neighborhood density
              if (!neighbors.empty() )
                {
                  double p = 1.0 - (1.0 / neighbors.size());
                  if (rng_.uniform01() < p)
                    continue;
                }
            }

          // Is motion good?
          if (keep)
            {
              // create a motion
              auto *motion = new Motion(si_);
              si_->copyState(motion->state, xstate);
              motion->parent = existing;

              // add it to everything
              motionLock_.lock();
              addMotion(motion, neighbors);
              motionLock_.unlock();

              // done?
              double dist = 0.0;
              bool solved = goal->isSatisfied(motion->state, &dist);
              if (solved)
                {
                  sol->approxdif = dist;
                  sol->solution = motion;
                  break;
                }
              if (dist < sol->approxdif)
                {
                  sol->approxdif = dist;
                  sol->approxsol = motion;
                }
            }
        }

      si_->freeState(xstate);
      delete xmotion;
    }

    ompl::base::PlannerStatus pNearEST::solve(const ompl::base::PlannerTerminationCondition &ptc) {
      checkValidity();

      std::vector<Motion *> neighbors;

      while (const ompl::base::State *st = pis_.nextStart())
        {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, st);

          nn_->nearestR(motion, nbrhoodRadius_, neighbors);
          addMotion(motion, neighbors);
        }

      if (motions_.empty())
        {
          OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
          return ompl::base::PlannerStatus::INVALID_START;
        }

      if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

      OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), motions_.size());

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
      if (sol.solution != nullptr) {
        {
          lastGoalMotion_ = sol.solution;

          // construct the solution path
          std::vector<Motion *> mpath;
          while (sol.solution != nullptr)
            {
              mpath.push_back(sol.solution);
              sol.solution = sol.solution->parent;
            }

          // set the solution path
          auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
          for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
          pdef_->addSolutionPath(path, false, sol.approxdif, getName());
          solved = true;
        }
      }

      OMPL_INFORM("%s: Created %u states", getName().c_str(), motions_.size());

      return {solved, false};
    }
  };
};
