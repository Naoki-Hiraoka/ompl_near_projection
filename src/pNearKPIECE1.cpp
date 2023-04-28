#include <ompl_near_projection/pNearKPIECE1.h>
#include <ompl_near_projection/NearGoalSpace.h>
#include <ompl_near_projection/NearProblemDefinition.h>

namespace ompl_near_projection {
  namespace geometric {
    void pNearKPIECE1::threadSolve(unsigned int tid, const ompl::base::PlannerTerminationCondition &ptc,
                                   SolutionInfo *sol) {
      NearProblemDefinitionPtr pdef_near_ = std::dynamic_pointer_cast<NearProblemDefinition>(pdef_);
      if(!pdef_near_){
        OMPL_ERROR("%s: pdef is not NearProblemDefinition!", getName().c_str());
        return;
      }

      if(pdef_near_->getGoals().size()==0){
        NearGoalSpacePtr goal = std::dynamic_pointer_cast<NearGoalSpace>(pdef_near_->getGoal());
        if (!goal) {
          OMPL_ERROR("%s: Goal is not NearGoalSpace!", getName().c_str());
          return;
        }
        pdef_near_->setGoals(std::vector<NearGoalSpacePtr>{goal});
      }
      const std::vector<NearGoalSpacePtr>& goals = pdef_near_->getGoals();

      ompl::geometric::Discretization<Motion>::Coord xcoord(projectionEvaluator_->getDimension());

      std::shared_ptr<NearProjectedStateSampler> sampler_near = std::dynamic_pointer_cast<NearProjectedStateSampler>(sampler_); // ここがKPIECE1と異なる
      if (!sampler_near) {
        OMPL_ERROR("%s: Sampler is not <NearProjectedStateSampler!", getName().c_str());
        return;
      }

      ompl::base::State *xstate = si_->allocState();

      while(sol->solution == nullptr && !ptc) {
        if(disc2_.getMotionCount() < tid + 1) usleep(sleepUs_); // これがないと、start state近くの点を過剰にsamplingしてしまってtree全体が歪む
        else break;
      }

      while (sol->solution == nullptr && !ptc)
        {
          /*
            addMotion時:
                cell->data->score = (1.0 + log((double)(iteration_))) / (1.0 + dist);
            updateCell時:
                cell->data->importance = cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections)
            ここでは、iteration = 1, score = 1で固定で運用している
           */
          discLock_.lock();
          //disc_.countIteration();

          /* Decide on a state to expand from */
          Motion *existing = nullptr;
          NearDiscretization<Motion>::Cell *ecell = nullptr;
          disc2_.selectMotion(existing, ecell);
          disc2_.updateCell(ecell); // ここで、selectした回数によって各cellのimpotanceを更新する. この直後に他のthreadがselectMotionしたときにはすでに反映されているように
          discLock_.unlock();
          assert(existing);

          sampler_near->sampleUniformNearValid(xstate, existing->state, maxDistance_); // sampleUniformNearValidの出力へのmotionが存在する前提. checkMotionを省略することで高速化
          double dist = (goals.size()==1) ? goals[0]->distanceGoal(xstate) : 0.0;
          /* create a motion */
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, xstate);
          motion->parent = existing;
          projectionEvaluator_->computeCoordinates(motion->state, xcoord);
          discLock_.lock();
          disc2_.addMotion(motion, xcoord, dist);  // this will also update the discretization heaps as needed, so no call to updateCell() is needed
          discLock_.unlock();

          existing = motion;

          bool has_unsolved = false;
          for(int i=0;i<goals.size();i++){
            if(sol->solution != nullptr || ptc){
              has_unsolved = true;
              break;
            }
            if(pdef_near_->getSolutionPathForAGoal(i) != nullptr) continue;
            else has_unsolved = true;

            bool solv = goals[i]->sampleTo(xstate, existing->state, &dist); // sampleToの出力へのmotionが存在する前提. checkMotionを省略することで高速化
          /* create a motion */
            auto *motion2 = new Motion(si_);
            si_->copyState(motion2->state, xstate);
            motion2->parent = existing;
            projectionEvaluator_->computeCoordinates(motion2->state, xcoord);
            discLock_.lock();
            disc2_.addMotion(motion2, xcoord, dist);  // this will also update the discretization heaps as needed, so no call to updateCell() is needed
            discLock_.unlock();
            if (solv)
              {
                /* construct the solution path */
                std::vector<Motion *> mpath;
                while (motion2 != nullptr)
                  {
                    mpath.push_back(motion2);
                    motion2 = motion2->parent;
                  }
                /* set the solution path */
                auto path(std::make_shared<ompl::geometric::PathGeometric>(si_));
                for (int i = mpath.size() - 1; i >= 0; --i)
                  path->append(mpath[i]->state);
                sol->lock.lock();
                pdef_near_->addSolutionPathForAGoal(i, path, false, dist, getName());
                sol->lock.unlock();
                break;
              }
          }
          if(!has_unsolved) sol->solution = motion;
        }

      si_->freeState(xstate);
    }

    ompl::base::PlannerStatus pNearKPIECE1::solve(const ompl::base::PlannerTerminationCondition &ptc) {
      checkValidity();

      NearProblemDefinitionPtr pdef_near_ = std::dynamic_pointer_cast<NearProblemDefinition>(pdef_);
      if(!pdef_near_){
        OMPL_ERROR("%s: pdef is not NearProblemDefinition!", getName().c_str());
        return ompl::base::PlannerStatus::CRASH;
      }

      if(pdef_near_->getGoals().size()==0){
        NearGoalSpacePtr goal = std::dynamic_pointer_cast<NearGoalSpace>(pdef_near_->getGoal());
        if (!goal) {
          OMPL_ERROR("%s: Goal is not NearGoalSpace!", getName().c_str());
          return ompl::base::PlannerStatus::CRASH;
        }
        pdef_near_->setGoals(std::vector<NearGoalSpacePtr>{goal});
      }

      NearDiscretization<Motion>::Coord xcoord(projectionEvaluator_->getDimension());

      while (const ompl::base::State *st = pis_.nextStart())
        {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, st);
          projectionEvaluator_->computeCoordinates(motion->state, xcoord);
          disc2_.addMotion(motion, xcoord, 1.0);
        }

      if (disc2_.getMotionCount() == 0)
        {
          OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
          return ompl::base::PlannerStatus::INVALID_START;
        }

      if (!sampler_)
        sampler_ = si_->allocStateSampler();

      OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                  disc2_.getMotionCount());

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
        solved = true;
      }

      OMPL_INFORM("%s: Created %u states in %u cells (%u internal + %u external)", getName().c_str(),
                  disc2_.getMotionCount(), disc2_.getCellCount(), disc2_.getGrid().countInternal(),
                  disc2_.getGrid().countExternal());

      return {solved, false};
    }
  };
};
