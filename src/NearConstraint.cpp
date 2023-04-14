#include <ompl_near_projection/NearConstraint.h>

namespace ompl_near_projection{
  bool NearConstraint::projectNear(ompl::base::State *state, const ompl::base::State *near) const{
    // 継承先のクラスで実装せよ
    return project(state);
  }
};
