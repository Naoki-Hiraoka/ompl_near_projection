#ifndef OMPL_NEAR_PROJECTION_NEARPROJECTEDSTATESPACE_H
#define OMPL_NEAR_PROJECTION_NEARPROJECTEDSTATESPACE_H

#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl_near_projection/NearConstraint.h>

namespace ompl_near_projection{
  OMPL_CLASS_FORWARD(NearProjectedStateSpace); // NearProjectedStateSpacePtrを定義 (shared_ptr)

  class NearProjectedStateSampler : public ompl::base::ProjectedStateSampler {
  public:
    NearProjectedStateSampler(const NearProjectedStateSpace *space, ompl::base::StateSamplerPtr sampler);

    // constraintを無視して、全状態空間の中からsampleする.
    void sampleUniformRaw(ompl::base::State *state);

    // nearはconstraintを満たしている想定.
    // 返り値のstateは、nearからstateへの、constraintを満たすmotionが必ず存在するという保証がある
    void sampleUniformNearValid(ompl::base::State *state, const ompl::base::State *near, double distance);

    // nearはconstraintを満たしている想定.
    // 返り値のstateは、nearからstateへの、constraintを満たすmotionが必ず存在するという保証がある
    // intermediateStatesは、nearからstateへ至る中間のstateが入る. nearとstateは入らない.
    void sampleGaussianValid(ompl::base::State *state, const ompl::base::State *mean, double stdDev);
  protected:
    const NearConstraintPtr nearConstraint_;
  };


  class NearProjectedStateSpace : public ompl::base::ProjectedStateSpace {
  public:
    class StateType : public ompl::base::ProjectedStateSpace::StateType {
    public:
      StateType(const ompl::base::ConstrainedStateSpace *space)
        : ProjectedStateSpace::StateType(space), space_(space) {
      }
      ~StateType() {
        for(int i=0;i<intermediateStates.size();i++) space_->freeState(intermediateStates[i]);
      }

      mutable std::vector<ompl::base::State*> intermediateStates; // このstateに至るまでの中間のstate. projectionの途中経過を格納したもので、constraintを厳密に満たしているとは限らない. 始点は含むが終点は含まない. これがあるなら、motionが必ず存在する.
    protected:
      const ompl::base::ConstrainedStateSpace *space_;
    };

    NearProjectedStateSpace(const ompl::base::StateSpacePtr &ambientSpace, const NearConstraintPtr &constraint) :
      ProjectedStateSpace(ambientSpace, constraint),
      nearConstraint_(constraint)
    {
      nearConstraint_->setStateSpace(this);
    }

    ompl::base::State *allocState() const override;
    void copyState(ompl::base::State *destination, const ompl::base::State *source) const override;

    const NearConstraintPtr getNearConstraint() const
    {
      return nearConstraint_;
    }

    ompl::base::StateSamplerPtr allocDefaultStateSampler() const override
    {
      return std::make_shared<NearProjectedStateSampler>(this, space_->allocDefaultStateSampler());
    }

    ompl::base::StateSamplerPtr allocStateSampler() const override
    {
      return std::make_shared<NearProjectedStateSampler>(this, space_->allocStateSampler());
    }

    // constraint->projectNear(state, near)を使う
    bool discreteGeodesic(const ompl::base::State *from, const ompl::base::State *to, bool interpolate = false,
                          std::vector<ompl::base::State *> *geodesic = nullptr) const override;

    void interpolateRaw(const ompl::base::State *from, const ompl::base::State *to, const double t,
                        ompl::base::State *state) const;

  protected:
    NearConstraintPtr nearConstraint_;
  };
};

#endif
