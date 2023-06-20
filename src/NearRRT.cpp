#include <ompl_near_projection/NearRRT.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection {
    namespace geometric {

      ompl::base::PlannerStatus NearRRT::solve(const ompl::base::PlannerTerminationCondition &ptc)
      {
        checkValidity();
        ompl::base::Goal *goal = pdef_->getGoal().get();
        auto *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion *>(goal);
        auto *goal_s_near = dynamic_cast<NearGoalSpace *>(goal_s); // ここがRRTと異なる.
        NearProjectedStateSpacePtr spaceNear = std::dynamic_pointer_cast<NearProjectedStateSpace>(si_->getStateSpace());
        if(!spaceNear){
          OMPL_ERROR("%s: Sapace is not NearProjectedStateSpace!", getName().c_str());
          return ompl::base::PlannerStatus::CRASH;
        }

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
        std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがRRTと異なる
        if (!sampler_near) {
          OMPL_ERROR("%s: Sampler is not NearProjectedStateSampler!", getName().c_str());
          return ompl::base::PlannerStatus::CRASH;
        }

        OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

        Motion *solution = nullptr;
        Motion *approxsol = nullptr;
        double approxdif = std::numeric_limits<double>::infinity();
        auto *rmotion = new Motion(si_);
        ompl::base::State *rstate = rmotion->state;
        ompl::base::State *xstate = si_->allocState();

        while (!ptc)
          {
            sampler_near->sampleUniformRaw(rstate);

            /* find closest state in the tree */
            Motion *nmotion = nn_->nearest(rmotion);
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
                        nn_->add(motion);

                        nmotion = motion;
                      }
                  }
                else
                  {
                    auto *motion = new Motion(si_);
                    si_->copyState(motion->state, dstate);
                    motion->parent = nmotion;
                    nn_->add(motion);

                    nmotion = motion;
                  }

                if(rng_.uniform01() < goalBias_){
                  bool sat = goal_s_near->sampleTo(xstate, nmotion->state); // sampleToの出力へのmotionが存在する前提. checkMotionを省略することで高速化
                  /* create a motion */
                  auto *motion2 = new Motion(si_);
                  si_->copyState(motion2->state, xstate);
                  motion2->parent = nmotion;
                  nn_->add(motion2);

                  nmotion = motion2;
                }

                double dist = 0.0;
                bool sat = goal->isSatisfied(nmotion->state, &dist);
                if (sat)
                  {
                    approxdif = dist;
                    solution = nmotion;
                    break;
                  }
                if (dist < approxdif)
                  {
                    approxdif = dist;
                    approxsol = nmotion;
                  }
              }
          }

        bool solved = false;
        if (solution != nullptr)
          {
            lastGoalMotion_ = solution;

            /* construct the solution path */
            std::vector<Motion *> mpath;
            while (solution != nullptr)
              {
                mpath.push_back(solution);
                solution = solution->parent;
              }

            /* set the solution path */
            auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
            for (int i = mpath.size() - 1; i >= 0; --i)
              path->append(mpath[i]->state);
            pdef_->addSolutionPath(path, false, approxdif, getName());
            solved = true;
          }

        si_->freeState(xstate);
        if (rmotion->state != nullptr)
          si_->freeState(rmotion->state);
        delete rmotion;

        OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

        return {solved, false};
      }


    }
}
