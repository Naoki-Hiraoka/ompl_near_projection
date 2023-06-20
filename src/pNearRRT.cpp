#include <ompl_near_projection/pNearRRT.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection {
  namespace geometric {

    void pNearRRT::threadSolve(unsigned int tid, const ompl::base::PlannerTerminationCondition &ptc,
                               SolutionInfo *sol) {
      ompl::base::Goal *goal = pdef_->getGoal().get();
      auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);
      auto *goal_s_near = dynamic_cast<NearGoalSpace *>(goal_s); // ここがRRTと異なる.
      NearProjectedStateSpacePtr spaceNear = std::dynamic_pointer_cast<NearProjectedStateSpace>(si_->getStateSpace());
      if(!spaceNear){
        OMPL_ERROR("%s: Sapace is not NearProjectedStateSpace!", getName().c_str());
        return;
      }

      std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがRRTと異なる
      if (!sampler_near) {
        OMPL_ERROR("%s: Sampler is not NearProjectedStateSampler!", getName().c_str());
        return;
      }

      while(sol->solution == nullptr && !ptc) {
        if(nn_->size() < tid + 1) usleep(sleepUs_); // これがないと、start state近くの点を過剰にsamplingしてしまってtree全体が歪む
        else break;
      }

      auto *rmotion = new Motion(si_);
      ompl::base::State *rstate = rmotion->state;
      ompl::base::State *xstate = si_->allocState();

      while (sol->solution == nullptr && !ptc)
        {
          sampler_near->sampleUniformRaw(rstate);

          /* find closest state in the tree */
          motionLock_.lock();
          Motion *nmotion = nn_->nearest(rmotion);
          motionLock_.unlock();
          ompl::base::State *dstate = rstate;

          /* find state to add */
          double d = si_->distance(nmotion->state, rstate);
          if (d > maxDistance_)
            {
              spaceNear->interpolateRaw(nmotion->state, rstate, maxDistance_ / d, xstate);
              dstate = xstate;
            }

          // constraintを満たす範囲で、nmotion->stateを出発点として、可能な限りdstateに近づくように移動して、結果をdstateに入れて返す
          spaceNear->getNearConstraint()->projectNearValid(dstate, nmotion->state);

          if (true)
            {
              if (addIntermediateStates_)
                {
                  std::vector<ompl::base::State *> states;
                  const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                  if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                  for (std::size_t i = 1; i < states.size(); ++i)
                    {
                      auto *motion = new Motion;
                      motion->state = states[i];
                      motion->parent = nmotion;
                      motionLock_.lock();
                      nn_->add(motion);
                      motionLock_.unlock();

                      nmotion = motion;
                    }
                }
              else
                {
                  auto *motion = new Motion(si_);
                  si_->copyState(motion->state, dstate);
                  motion->parent = nmotion;
                  motionLock_.lock();
                  nn_->add(motion);
                  motionLock_.unlock();

                  nmotion = motion;
                }

              if(rng_.uniform01() < goalBias_){
                bool sat = goal_s_near->sampleTo(xstate, nmotion->state); // sampleToの出力へのmotionが存在する前提. checkMotionを省略することで高速化
                /* create a motion */
                auto *motion2 = new Motion(si_);
                si_->copyState(motion2->state, xstate);
                motion2->parent = nmotion;
                motionLock_.lock();
                nn_->add(motion2);
                motionLock_.unlock();

                nmotion = motion2;
              }

              double dist = 0.0;
              bool sat = goal->isSatisfied(nmotion->state, &dist);
              if (sat)
                {
                  sol->approxdif = dist;
                  sol->solution = nmotion;
                  break;
                }
              if (dist < sol->approxdif)
                {
                  sol->approxdif = dist;
                  sol->approxsol = nmotion;
                }
            }
        }

      si_->freeState(xstate);
      if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
      delete rmotion;

    }

    ompl::base::PlannerStatus pNearRRT::solve(const ompl::base::PlannerTerminationCondition &ptc)
    {
      checkValidity();

      while (const ompl::base::State *st = pis_.nextStart())
        {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, st);
          nn_->add(motion);
        }

      if (nn_->size() == 0)
        {
          OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
          return ompl::base::PlannerStatus::INVALID_START;
        }

      if (!sampler_)
        sampler_ = si_->allocStateSampler();

      OMPL_INFORM("p%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

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
          pdef_->addSolutionPath(path, false, sol.approxdif, getName());
          solved = true;
        }

      OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

      return {solved, false};
    }


  }
}
