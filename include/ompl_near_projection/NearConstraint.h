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
    // nearはconstraintを満たしている想定. nearの情報を利用することで、projectの成功率を上げる.
    // 例えば、nearを初期値として、constraint下でstateからの距離を可能な限り小さくする最適化計算を行う
    // projection失敗しても、stateはprojection後の値が入る. 返り値はfalseになる.
    // 名前がprojectだと、親クラスのproject(State * state)を消してしまうみたいなので、prejectNearという名前にした
    virtual bool projectNear(ompl::base::State *state, const ompl::base::State *near) const;

  };

};

#endif
