#ifndef OMPL_NEAR_PROJECTION_NEARPROBLEMDEFINITION_H
#define OMPL_NEAR_PROJECTION_NEARPROBLEMDEFINITION_H

#include <ompl/base/ProblemDefinition.h>
#include <ompl_near_projection/NearGoalSpace.h>

namespace ompl_near_projection {
  OMPL_CLASS_FORWARD(NearProblemDefinition); // *Ptrを定義

  class NearProblemDefinition : public ompl::base::ProblemDefinition {
  public:
    NearProblemDefinition(ompl::base::SpaceInformationPtr si):
      ProblemDefinition(si)
    {}

    void setGoals(const std::vector<NearGoalSpacePtr> &goals);
    const std::vector<NearGoalSpacePtr> &getGoals() const;

    // goals_と同じサイズのvectorを返す. 各要素がgoals_の各要素に対応している. pathが存在しない場合はnullptrを返す.
    const std::vector<ompl::base::PathPtr>& getSolutionPathForEachGoal() const;
    ompl::base::PathPtr getSolutionPathForAGoal(unsigned int goalid) const;

    void addSolutionPathForAGoal(unsigned int goalid, const ompl::base::PathPtr &path, bool approximate = false, double difference = -1.0,
                                 const std::string &plannerName = "Unknown");

    // 内部でaddSolutionPathも呼ぶ
    void addSolutionPathForAGoal(unsigned int goalid, const ompl::base::PlannerSolution &sol);

    // 内部でclearSolutionPathsも呼ぶ
    void clearSolutionPathsForAllGoals();

    void setFindAllGoals(bool findAllGoals) { findAllGoals_ = findAllGoals; }
    bool getFindAllGoals() { return findAllGoals_; }
  protected:
    std::vector<NearGoalSpacePtr> goals_;
    std::vector<ompl::base::PathPtr> solutionsForEachGoal_;
    bool findAllGoals_ = false;
    bool addSolutionPathForAGoalCalled_ = false;
  };

};

#endif
