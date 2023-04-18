#ifndef OMPL_NEAR_PROJECTION_NEARCONSTRAINEDSPACEINFORMATION_H
#define OMPL_NEAR_PROJECTION_NEARCONSTRAINEDSPACEINFORMATION_H

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl_near_projection/NearProjectedStateSpace.h>

namespace ompl_near_projection{
  OMPL_CLASS_FORWARD(NearConstrainedSpaceInformation); // *Ptrを定義. (shared_ptr)

  // statespaceは、NearProjectedStateSpaceでなければならない
  class NearConstrainedValidStateSampler : public ompl::base::ConstrainedValidStateSampler {
  public:
    NearConstrainedValidStateSampler(const ompl::base::SpaceInformation *si)
      : ConstrainedValidStateSampler(si),
        nearSampler_(std::dynamic_pointer_cast<NearProjectedStateSampler>(si->getStateSpace()->allocStateSampler()))
    {
      assert(nearSampler_);
    }

    // nearはconstraintを満たしている想定.
    // 返り値のstateは、nearからstateへの、constraintを満たすmotionが必ず存在するという保証がある
    virtual bool sampleNearValid(ompl::base::State *state, const ompl::base::State *near, double distance);

  protected:
    std::shared_ptr<NearProjectedStateSampler> nearSampler_;
  };

  class NearConstrainedSpaceInformation : public ompl::base::ConstrainedSpaceInformation {
  public:
    NearConstrainedSpaceInformation(ompl::base::StateSpacePtr space) : ConstrainedSpaceInformation(std::move(space)) {
      setValidStateSamplerAllocator([](const ompl::base::SpaceInformation *si) -> std::shared_ptr<ompl::base::ValidStateSampler> {
          return std::make_shared<NearConstrainedValidStateSampler>(si);
        });
    }
  };


};

#endif
