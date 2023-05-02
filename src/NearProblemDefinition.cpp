#include <ompl_near_projection/NearProblemDefinition.h>

namespace ompl_near_projection {

  void NearProblemDefinition::setGoals(const std::vector<NearGoalSpacePtr> &goals) {
    if(goals.size()>0) goal_ = goals[0];
    goals_ = goals;
    solutionsForEachGoal_.resize(goals.size());
  }
  const std::vector<NearGoalSpacePtr> &NearProblemDefinition::getGoals() const {
    return goals_;
  }

  const std::vector<ompl::base::PathPtr>& NearProblemDefinition::getSolutionPathForEachGoal() const{
    return solutionsForEachGoal_;
  }

  ompl::base::PathPtr NearProblemDefinition::getSolutionPathForAGoal(unsigned int goalid) const {
    return solutionsForEachGoal_[goalid];
  }

  void NearProblemDefinition::addSolutionPathForAGoal(unsigned int goalid, const ompl::base::PathPtr &path, bool approximate, double difference,
                                                      const std::string &plannerName) {
    ompl::base::PlannerSolution sol(path);
    if (approximate)
      sol.setApproximate(difference);
    sol.setPlannerName(plannerName);
    addSolutionPathForAGoal(goalid, sol);
  }

  void NearProblemDefinition::addSolutionPathForAGoal(unsigned int goalid, const ompl::base::PlannerSolution &sol) {
    solutionsForEachGoal_[goalid] = sol.path_;
    addSolutionPath(sol);
  }

  void NearProblemDefinition::clearSolutionPathsForAllGoals() {
    for(int i=0;i<solutionsForEachGoal_.size();i++) solutionsForEachGoal_[i] = nullptr;
    clearSolutionPaths();
  }

};

