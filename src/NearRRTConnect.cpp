#include <ompl_near_projection/NearRRTConnect.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection {
  namespace geometric {
    ompl::base::PlannerStatus NearRRTConnect::solve(const ompl::base::PlannerTerminationCondition &ptc) {
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
      std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがKPIECE1と異なる
      if (!sampler_near) {
        OMPL_ERROR("%s: Sampler is not NearProjectedStateSampler!", getName().c_str());
        return ompl::base::PlannerStatus::CRASH;
      }

      OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                  (int)(tStart_->size() + tGoal_->size()));

      TreeGrowingInfo tgi;
      tgi.xstate = si_->allocState();

      Motion *approxsol = nullptr;
      double approxdif = std::numeric_limits<double>::infinity();
      auto *rmotion = new Motion(si_);
      ompl::base::State *rstate = rmotion->state;
      bool solved = false;

      while (!ptc)
        {
          TreeData &tree = startTree_ ? tStart_ : tGoal_;
          std::mutex &treeLock = startTree_ ? tStartLock_ : tGoalLock_;
          tgi.start = startTree_;
          startTree_ = !startTree_;
          TreeData &otherTree = startTree_ ? tStart_ : tGoal_;
          std::mutex &otherTreeLock = startTree_ ? tStartLock_ : tGoalLock_;

          if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
            {
              const ompl::base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
              if (st != nullptr)
                {
                  auto *motion = new Motion(si_);
                  si_->copyState(motion->state, st);
                  motion->root = motion->state;
                  tGoal_->add(motion);
                }

              if (tGoal_->size() == 0)
                {
                  OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                  break;
                }
            }

          /* sample random state */
          sampler_near->sampleUniformRaw(rstate);

          GrowState gs = growTreeNear(tree, tgi, rmotion, treeLock);

          if (gs != TRAPPED)
            {
              /* remember which motion was just added */
              Motion *addedMotion = tgi.xmotion;

              /* attempt to connect trees */

              si_->copyState(rstate, tgi.xstate);

              tgi.start = startTree_;

              /* if initial progress cannot be done from the otherTree, restore tgi.start */
              GrowState gsc = growTreeNear(otherTree, tgi, rmotion, otherTreeLock);
              if (gsc == TRAPPED)
                tgi.start = !tgi.start;

              while (gsc == ADVANCED)
                gsc = growTreeNear(otherTree, tgi, rmotion, otherTreeLock);

              // /* update distance between trees */
              // const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
              // if (newDist < distanceBetweenTrees_)
              //   {
              //     distanceBetweenTrees_ = newDist;
              //     // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
              //   }

              Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
              Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

              /* if we connected the trees in a valid way (start and goal pair is valid)*/
              if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
                {
                  connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                  /* construct the solution path */
                  Motion *solution = startMotion;
                  std::vector<Motion *> mpath1;
                  while (solution != nullptr)
                    {
                      mpath1.push_back(solution);
                      solution = solution->parent;
                    }

                  solution = goalMotion;
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
                  solved = true;
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
                      if (dist < approxdif)
                        {
                          approxdif = dist;
                          approxsol = tgi.xmotion;
                        }
                    }
                }
            }
        }

      si_->freeState(tgi.xstate);
      si_->freeState(rstate);
      delete rmotion;

      OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                  tStart_->size(), tGoal_->size());

      if (approxsol && !solved)
        {
          /* construct the solution path */
          std::vector<Motion *> mpath;
          while (approxsol != nullptr)
            {
              mpath.push_back(approxsol);
              approxsol = approxsol->parent;
            }

          auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
          for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
          pdef_->addSolutionPath(path, true, approxdif, getName());
          return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
        }

      return solved ? ompl::base::PlannerStatus::EXACT_SOLUTION : ompl::base::PlannerStatus::TIMEOUT;
    }

    ompl::geometric::RRTConnect::GrowState NearRRTConnect::growTreeNear(TreeData &tree, TreeGrowingInfo &tgi,
                                                                        Motion *rmotion, std::mutex& treeLock) {

      NearProjectedStateSpacePtr spaceNear = std::dynamic_pointer_cast<NearProjectedStateSpace>(si_->getStateSpace());
      if(!spaceNear){
        OMPL_ERROR("%s: Sapace is not NearProjectedStateSpace!", getName().c_str());
        return TRAPPED;
      }

      /* find closest state in the tree */
      treeLock.lock();
      Motion *nmotion = tree->nearest(rmotion);
      treeLock.unlock();

      /* assume we can reach the state we go towards */
      bool reach = false;

      /* find state to add */
      ompl::base::State *dstate = tgi.xstate;
      // si_->copyState(dstate, rmotion->state);
      // double d = si_->distance(nmotion->state, dstate);
      // if (d > maxDistance_)
      //   {
      //     spaceNear->interpolateRaw(nmotion->state, rmotion->state, maxDistance_ / d, dstate);
      //   }
      spaceNear->elementWiseDistanceLimit(nmotion->state, rmotion->state, maxDistance_, dstate); // 全体のノルムでやると、SE3の並進成分が大きすぎて、SE3の回転成分がほとんど動かなくなってしまうので

      // constraintを満たす範囲で、nmotion->stateを出発点として、可能な限りdstateに近づくように移動して、結果をdstateに入れて返す
      spaceNear->getNearConstraint()->projectNearValid(dstate, nmotion->state);

      if (si_->distance(dstate, rmotion->state) <= spaceNear->getDelta()){
        reach = true;
      }

      if(!reach && si_->distance(dstate, nmotion->state) <= spaceNear->getDelta()){
        return TRAPPED;
      }

      if (addIntermediateStates_)
        {
          const ompl::base::State *astate = tgi.start ? nmotion->state : dstate;
          const ompl::base::State *bstate = tgi.start ? dstate : nmotion->state;

          std::vector<ompl::base::State *> states;
          const unsigned int count = si_->getStateSpace()->validSegmentCount(astate, bstate);

          if (si_->getMotionStates(astate, bstate, states, count, true, true))
            si_->freeState(states[0]);

          for (std::size_t i = 1; i < states.size(); ++i)
            {
              auto *motion = new Motion;
              motion->state = states[i];
              motion->parent = nmotion;
              motion->root = nmotion->root;
              treeLock.lock();
              tree->add(motion);
              treeLock.unlock();

              nmotion = motion;
            }

          tgi.xmotion = nmotion;
        }
      else
        {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, dstate);
          motion->parent = nmotion;
          motion->root = nmotion->root;
          treeLock.lock();
          tree->add(motion);
          treeLock.unlock();

          tgi.xmotion = motion;
        }

      return reach ? REACHED : ADVANCED;
    }

  }
};
