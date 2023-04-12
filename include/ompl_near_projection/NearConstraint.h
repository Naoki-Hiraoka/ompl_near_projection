#ifndef OMPL_NEAR_PROJECTION_NEARCONSTRAINT_H
#define OMPL_NEAR_PROJECTION_NEARCONSTRAINT_H

#include <ompl/base/Constraint.h>

namespace ompl_near_constraint{
  OMPL_CLASS_FORWARD(NearConstraint); // NearConstraintPtrを定義. (shared_ptr)

  class NearConstraint : public ompl::base::Constraint {
  public:
    //virtual bool project(ompl::base::State *state) const override;
    // nearはconstraintを満たしている想定. nearの情報を利用することで、projectの成功率を上げる.
    // 例えば、nearを初期値として、constraint下でstateからの距離を可能な限り小さくする最適化計算を行う
    // 名前がprojectだと、親クラスのproject(State * state)を消してしまうみたいなので、prejectNearという名前にした
    virtual bool projectNear(ompl::base::State *state, const ompl::base::State *near) const;
  };

};

#endif
