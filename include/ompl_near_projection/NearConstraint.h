#ifndef OMPL_NEAR_PROJECTION_NEARCONSTRAINT_H
#define OMPL_NEAR_PROJECTION_NEARCONSTRAINT_H

#include <ompl/base/Constraint.h>

namespace ompl_near_projection{
  OMPL_CLASS_FORWARD(NearConstraint); // NearConstraintPtrを定義. (shared_ptr)

  class NearConstraint : public ompl::base::Constraint {
  public:
    NearConstraint(const unsigned int ambientDim, const unsigned int coDim,
                   double tolerance = ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE)
      : Constraint(ambientDim, coDim, tolerance)
    {
    }

    // nearから、constraintを満たす範囲で可能な限りstateに近づくようにprojectionし、結果をstateに入れて返す.
    // nearがconstraintを満たしている場合、返り値のstateも必ずconstraintを満たす. また、constaintを満たしつつnearと返り値のstateを結ぶmotionが必ず存在する.
    // constraintが「まずconstraint1を満たす. 残りの自由度でconstraint2を満たす」というもので、nearがconstraint1を満たしている場合、返り値のstateも必ずconstraint1を満たす. また、constraint1を満たしつつnearと返り値のstateを結ぶmotionが必ず存在する
    // projectionに失敗しても、stateはprojection後の値が入る. 返り値はfalseになる.
    virtual bool projectNearValid(ompl::base::State *state, const ompl::base::State *near) const = 0;

    const ompl::base::StateSpace* getStateSpace() const { return stateSpace_; }
    void setStateSpace(ompl::base::StateSpace* stateSpace) { stateSpace_ = stateSpace; }
  protected:
    ompl::base::StateSpace* stateSpace_;
  };

};

#endif
