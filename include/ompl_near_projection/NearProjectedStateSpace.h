#ifndef OMPL_NEAR_PROJECTION_NEARPROJECTEDSTATESPACE_H
#define OMPL_NEAR_PROJECTION_NEARPROJECTEDSTATESPACE_H

#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl_near_projection/NearConstraint.h>

namespace ompl_near_projection{
  OMPL_CLASS_FORWARD(NearProjectedStateSpace); // NearProjectedStateSpacePtrを定義 (shared_ptr)

  class NearProjectedStateSampler : public ompl::base::ProjectedStateSampler {
  public:
    NearProjectedStateSampler(const NearProjectedStateSpace *space, ompl::base::StateSamplerPtr sampler);

    // constraint->projectNear(state, near)を使う
    // nearはconstraintを満たしている想定
    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    // constraint->projectNear(state, near)を使う
    // nearはconstraintを満たしている想定
    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
  protected:
    const NearConstraintPtr nearConstraint_;
  };


  class NearProjectedStateSpace : public ompl::base::ProjectedStateSpace {
  public:
    NearProjectedStateSpace(const ompl::base::StateSpacePtr &ambientSpace, const NearConstraintPtr &constraint) :
      ProjectedStateSpace(ambientSpace, constraint),
      nearConstraint_(constraint)
    {
    }

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

  protected:
    NearConstraintPtr nearConstraint_;
  };
};

#endif
