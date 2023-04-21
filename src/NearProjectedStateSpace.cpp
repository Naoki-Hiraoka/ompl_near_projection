#include <ompl_near_projection/NearProjectedStateSpace.h>

namespace ompl_near_projection{

  NearProjectedStateSampler::NearProjectedStateSampler(const NearProjectedStateSpace *space, ompl::base::StateSamplerPtr sampler) :
    ProjectedStateSampler(space, sampler),
    nearConstraint_(space->getNearConstraint())
  {
  }

  void NearProjectedStateSampler::sampleUniformNearValid(ompl::base::State *state, const ompl::base::State *near, double distance) {
    WrapperStateSampler::sampleUniformNear(state, near, distance);
    nearConstraint_->projectNearValid(state, near);
    space_->enforceBounds(state);
  }

  void NearProjectedStateSampler::sampleGaussianValid(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
    WrapperStateSampler::sampleGaussian(state, mean, stdDev);
    nearConstraint_->projectNearValid(state, mean);
    space_->enforceBounds(state);
  }

  ompl::base::State *NearProjectedStateSpace::allocState() const {
    return new StateType(this);
  }

  void NearProjectedStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const {
    WrapperStateSpace::copyState(destination, source);
    for(int i=0;i<destination->as<StateType>()->intermediateStates.size();i++){
      freeState(destination->as<StateType>()->intermediateStates[i]);
    }
    destination->as<StateType>()->intermediateStates.resize(source->as<StateType>()->intermediateStates.size());
    for(int i=0;i<source->as<StateType>()->intermediateStates.size();i++){
      destination->as<StateType>()->intermediateStates[i] = cloneState(source->as<StateType>()->intermediateStates[i]);
    }
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

    const std::vector<ompl::base::State*>& intermediateStates = static_cast<const NearProjectedStateSpace::StateType*>(to)->intermediateStates;
    std::vector<ompl::base::State *> path;
    if(geodesic && intermediateStates.size() > 0){ // geodesicが与えられている場合: interpolate(). 与えられていない場合: simplify(), check(). 特にsimplify時にintermidiateStatesを使ってはいけない
      for(int i=0;i<intermediateStates.size()+1;i++){
        if(i==intermediateStates.size()) {
          copyState(scratch, to);
        }else{
          copyState(scratch, intermediateStates[i]);
          if(distance(previous, scratch) < delta_) continue;
        }
        nearConstraint_->projectNearValid(scratch, previous);
        if ((step = distance(previous, scratch)) > lambda_ * delta_)  // deviated
          break;
        total += step;
        if (total > max)
          break;
        const double newDist = distance(scratch, to);
        if (newDist >= dist)
          break;
        dist = newDist;
        copyState(previous, scratch);
        geodesic->push_back(cloneState(scratch));
      }
    }else{
      do
        {
          WrapperStateSpace::interpolate(previous, to, std::min(1.0, delta_ / dist), scratch);

          // Project new state onto constraint manifold
          // ここがProjectedStateSpace::discreteGeodesicと異なる
          nearConstraint_->projectNearValid(scratch, previous); // 返り値のscratchはambientSpace及びconstraintを必ず満たしている. そのため、scratchのisValidチェックは省略可能で高速化. 逆に、projectの誤差によってscratchが僅かにvalidでない場合があるので、むしろscratchのvalidチェックはしてはいけない. (特にConstrainedSpaceInformation->interpolate()時に、問題になる)
          if ((step = distance(previous, scratch)) > lambda_ * delta_)  // deviated
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
          else
            path.push_back(cloneState(scratch));

        } while (dist >= tolerance);
    }

    freeState(scratch);
    freeState(previous);

    if(dist <= tolerance){
      if (geodesic != nullptr) {
        for(int i=0;i<to->as<NearProjectedStateSpace::StateType>()->intermediateStates.size();i++) freeState(to->as<NearProjectedStateSpace::StateType>()->intermediateStates[i]);
        to->as<NearProjectedStateSpace::StateType>()->intermediateStates.clear();
      }else{
        for(int i=0;i<to->as<NearProjectedStateSpace::StateType>()->intermediateStates.size();i++) freeState(to->as<NearProjectedStateSpace::StateType>()->intermediateStates[i]);
        to->as<NearProjectedStateSpace::StateType>()->intermediateStates = path;
      }
    }

    return dist <= tolerance;
  }

};
