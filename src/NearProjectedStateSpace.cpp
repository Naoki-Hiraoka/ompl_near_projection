#include <ompl_near_projection/NearProjectedStateSpace.h>

namespace ompl_near_projection{

  NearProjectedStateSampler::NearProjectedStateSampler(const NearProjectedStateSpace *space, ompl::base::StateSamplerPtr sampler) :
    ProjectedStateSampler(space, sampler),
    nearConstraint_(space->getNearConstraint())
  {
  }

  void NearProjectedStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) {
    WrapperStateSampler::sampleUniformNear(state, near, distance);
    if(nearConstraint_) nearConstraint_->projectNear(state, near);
    else constraint_->project(state);
    space_->enforceBounds(state);
  }

  void NearProjectedStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
    WrapperStateSampler::sampleGaussian(state, mean, stdDev);
    if(nearConstraint_) nearConstraint_->projectNear(state, mean);
    else constraint_->project(state);
    space_->enforceBounds(state);
  }

  bool NearProjectedStateSpace::discreteGeodesic(const ompl::base::State *from, const ompl::base::State *to, bool interpolate,
                                                 std::vector<ompl::base::State *> *geodesic) const
  {
    // Save a copy of the from state.
    if (geodesic != nullptr)
      {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
      }

    const double tolerance = delta_;

    // No need to traverse the manifold if we are already there.
    double dist, step, total = 0;
    if ((dist = distance(from, to)) <= tolerance)
      return true;

    const double max = dist * lambda_;

    auto previous = cloneState(from);
    auto scratch = allocState();

    auto &&svc = si_->getStateValidityChecker();

    do
      {
        WrapperStateSpace::interpolate(previous, to, delta_ / dist, scratch);

        // Project new state onto constraint manifold
        if (!nearConstraint_->projectNear(scratch, previous)  // not on manifold // この行がProjectedStateSpace::discreteGeodesicと異なる
            || !(interpolate || svc->isValid(scratch))      // not valid
            || (step = distance(previous, scratch)) > lambda_ * delta_)  // deviated
          break;

        // Check if we have wandered too far
        total += step;
        if (total > max)
          break;

        // Check if we are no closer than before
        // epsilonの閾値を設けたほうがいいかも TODO
        const double newDist = distance(scratch, to);
        if (newDist >= dist)
          break;

        dist = newDist;
        copyState(previous, scratch);

        // Store the new state
        if (geodesic != nullptr)
          geodesic->push_back(cloneState(scratch));

      } while (dist >= tolerance);

    freeState(scratch);
    freeState(previous);

    return dist <= tolerance;
  }

};
