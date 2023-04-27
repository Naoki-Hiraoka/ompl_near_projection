/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ompl_near_projection/NearKPIECE1.h>
#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection {
  namespace geometric {
    ompl::base::PlannerStatus NearKPIECE1::solve(const ompl::base::PlannerTerminationCondition &ptc) {
      checkValidity();
      ompl::base::Goal *goal = pdef_->getGoal().get();
      auto *goal_s_near = dynamic_cast<NearGoalSpace *>(goal); // ここがKPIECE1と異なる.
      if (!goal_s_near) {
        OMPL_ERROR("%s: Goal is not NearGoalSpace!", getName().c_str());
        return ompl::base::PlannerStatus::CRASH;
      }

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
      std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがKPIECE1と異なる
      if (!sampler_near) {
        OMPL_ERROR("%s: Sampler is not <NearProjectedStateSampler!", getName().c_str());
        return ompl::base::PlannerStatus::CRASH;
      }

      OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                  disc_.getMotionCount());

      Motion *solution = nullptr;
      Motion *approxsol = nullptr;
      double approxdif = std::numeric_limits<double>::infinity();
      ompl::base::State *xstate = si_->allocState();

      while (!ptc)
        {
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
          assert(existing);

          sampler_near->sampleUniformNearValid(xstate, existing->state, maxDistance_); // sampleUniformNearValidの出力へのmotionが存在する前提. checkMotionを省略することで高速化
          double dist = goal_s_near->distanceGoal(xstate);
          /* create a motion */
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, xstate);
          motion->parent = existing;
          projectionEvaluator_->computeCoordinates(motion->state, xcoord);
          disc_.addMotion(motion, xcoord, dist);  // this will also update the discretization heaps as needed, so no call to updateCell() is needed

          existing = motion;
          bool solv = goal_s_near->sampleTo(xstate, existing->state, &dist); // sampleToの出力へのmotionが存在する前提. checkMotionを省略することで高速化
          /* create a motion */
          auto *motion2 = new Motion(si_);
          si_->copyState(motion2->state, xstate);
          motion2->parent = existing;
          projectionEvaluator_->computeCoordinates(motion2->state, xcoord);
          disc_.addMotion(motion2, xcoord, dist);  // this will also update the discretization heaps as needed, so no call to updateCell() is needed
          if (solv)
            {
              approxdif = dist;
              solution = motion2;
              break;
            }
          disc_.updateCell(ecell);
        }

      bool solved = false;
      bool approximate = false;
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
          pdef_->addSolutionPath(path, approximate, approxdif, getName());
          solved = true;
        }

      si_->freeState(xstate);

      OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(),
                  disc_.getMotionCount(), disc_.getCellCount(), disc_.getGrid().countInternal(),
                  disc_.getGrid().countExternal());

      return {solved, approximate};
    }
  }
};
