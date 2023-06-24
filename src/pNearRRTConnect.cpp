#include <ompl_near_projection/pNearRRTConnect.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection {
  namespace geometric {
    ompl::base::PlannerStatus pNearRRTConnect::solve(const ompl::base::PlannerTerminationCondition &ptc) {
      checkValidity();
      auto *goal = dynamic_cast<ompl::base::GoalSampleableRegion *>(pdef_->getGoal().get());

      if (goal == nullptr)
        {
          OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
          return ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
        }

      while (const ompl::base::State *st = pis_.nextStart())
        {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, st);
          motion->root = motion->state;
          tStart_->add(motion);
        }

      if (tStart_->size() == 0)
        {
          OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
          return ompl::base::PlannerStatus::INVALID_START;
        }

      if (!goal->couldSample())
        {
          OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
          return ompl::base::PlannerStatus::INVALID_GOAL;
        }

      if (!sampler_)
        sampler_ = si_->allocStateSampler();
      std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがRRTConnectと異なる
      if (!sampler_near) {
        OMPL_ERROR("%s: Sampler is not NearProjectedStateSampler!", getName().c_str());
        return ompl::base::PlannerStatus::CRASH;
      }

      OMPL_INFORM("p%s: Starting planning with %d states already in datastructure", getName().c_str(),
                  (int)(tStart_->size() + tGoal_->size()));

      SolutionInfo sol;
      sol.startSolution = nullptr;
      sol.goalSolution = nullptr;
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

      OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                  tStart_->size(), tGoal_->size());


      bool solved = false;
      if (sol.startSolution != nullptr) {
        /* construct the solution path */
        Motion *solution = sol.startSolution;
        std::vector<Motion *> mpath1;
        while (solution != nullptr)
          {
            mpath1.push_back(solution);
            solution = solution->parent;
          }

        solution = sol.goalSolution;
        std::vector<Motion *> mpath2;
        while (solution != nullptr)
          {
            mpath2.push_back(solution);
            solution = solution->parent;
          }

        auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
        path->getStates().reserve(mpath1.size() + mpath2.size());
        for (int i = mpath1.size() - 1; i >= 0; --i)
          path->append(mpath1[i]->state);
        for (auto &i : mpath2)
          path->append(i->state);

        pdef_->addSolutionPath(path, false, 0.0, getName());

        return ompl::base::PlannerStatus::EXACT_SOLUTION;
      }

      if (sol.approxsol)
        {
          /* construct the solution path */
          std::vector<Motion *> mpath;
          while (sol.approxsol != nullptr)
            {
              mpath.push_back(sol.approxsol);
              sol.approxsol = sol.approxsol->parent;
            }

          auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
          for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
          pdef_->addSolutionPath(path, true, sol.approxdif, getName());
          return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
        }

      return ompl::base::PlannerStatus::TIMEOUT;
    }

    void pNearRRTConnect::threadSolve(unsigned int tid, const ompl::base::PlannerTerminationCondition &ptc, SolutionInfo *sol){
      auto *goal = dynamic_cast<ompl::base::GoalSampleableRegion *>(pdef_->getGoal().get());
      std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがRRTConnectと異なる

      TreeGrowingInfo tgi;
      tgi.xstate = si_->allocState();

      auto *rmotion = new Motion(si_);
      ompl::base::State *rstate = rmotion->state;

      // while(sol->startSolution == nullptr && !ptc) {
      //   if(tStart_->size() < tid + 1) usleep(sleepUs_); // これがないと、start state近くの点を過剰にsamplingしてしまってtree全体が歪む? RRTの場合関係ない
      //   else break;
      // }

      while (sol->startSolution == nullptr && !ptc)
        {
          startTreeLock_.lock();
          TreeData &tree = startTree_ ? tStart_ : tGoal_;
          std::mutex &treeLock = startTree_ ? tStartLock_ : tGoalLock_;
          tgi.start = startTree_;
          startTree_ = !startTree_;
          TreeData &otherTree = startTree_ ? tStart_ : tGoal_;
          std::mutex &otherTreeLock = startTree_ ? tStartLock_ : tGoalLock_;
          bool currentStartTree = startTree_;
          startTreeLock_.unlock();

          pisLock_.lock();
          if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
            {
              const ompl::base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
              if (st != nullptr)
                {
                  auto *motion = new Motion(si_);
                  si_->copyState(motion->state, st);
                  motion->root = motion->state;
                  tGoalLock_.lock();
                  tGoal_->add(motion);
                  tGoalLock_.unlock();
                }

              if (tGoal_->size() == 0)
                {
                  OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                  break;
                }
            }
          pisLock_.unlock();

          /* sample random state */
          sampler_near->sampleUniformRaw(rstate);

          GrowState gs = growTreeNear(tree, tgi, rmotion, treeLock);

          if (gs != TRAPPED)
            {
              /* remember which motion was just added */
              Motion *addedMotion = tgi.xmotion;

              /* attempt to connect trees */

              si_->copyState(rstate, tgi.xstate);

              tgi.start = currentStartTree;

              /* if initial progress cannot be done from the otherTree, restore tgi.start */
              GrowState gsc = growTreeNear(otherTree, tgi, rmotion, otherTreeLock);
              if (gsc == TRAPPED)
                tgi.start = !tgi.start;

              while (gsc == ADVANCED)
                gsc = growTreeNear(otherTree, tgi, rmotion, otherTreeLock);

              /* update distance between trees */
              otherTreeLock.lock();
              const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
              otherTreeLock.unlock();
              if (newDist < distanceBetweenTrees_)
                {
                  distanceBetweenTrees_ = newDist;
                  // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
                }

              Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
              Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

              /* if we connected the trees in a valid way (start and goal pair is valid)*/
              if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
                {
                  sol->lock.lock();
                  if(!sol->startSolution){
                    connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);
                    sol->startSolution = startMotion;
                    sol->goalSolution = goalMotion;
                  }
                  sol->lock.unlock();

                  break;
                }
              else
                {
                  // We didn't reach the goal, but if we were extending the start
                  // tree, then we can mark/improve the approximate path so far.
                  if (tgi.start)
                    {
                      // We were working from the startTree.
                      double dist = 0.0;
                      goal->isSatisfied(tgi.xmotion->state, &dist);
                      sol->lock.lock();
                      if (dist < sol->approxdif)
                        {
                          sol->approxdif = dist;
                          sol->approxsol = tgi.xmotion;
                        }
                      sol->lock.unlock();
                    }
                }
            }
        }

      si_->freeState(tgi.xstate);
      si_->freeState(rstate);
      delete rmotion;


    }


  }
};
