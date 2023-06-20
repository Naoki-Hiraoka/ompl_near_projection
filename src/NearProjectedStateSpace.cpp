#include <ompl_near_projection/NearProjectedStateSpace.h>

namespace ompl_near_projection{

  NearProjectedStateSampler::NearProjectedStateSampler(const NearProjectedStateSpace *space, ompl::base::StateSamplerPtr sampler) :
    ProjectedStateSampler(space, sampler),
    nearConstraint_(space->getNearConstraint())
  {
  }

  void NearProjectedStateSampler::sampleUniformRaw(ompl::base::State *state)
  {
    WrapperStateSampler::sampleUniform(state);
    space_->enforceBounds(state);
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

    std::vector<ompl::base::State*>& intermediateStates = static_cast<const NearProjectedStateSpace::StateType*>(to)->intermediateStates;
    std::vector<ompl::base::State*>& intermediateStatesInv = static_cast<const NearProjectedStateSpace::StateType*>(from)->intermediateStates;
    std::vector<ompl::base::State *> path{cloneState(from)};

    if(intermediateStates.size() > 0 && distance(intermediateStates[0], from) < delta_) {
      if(geodesic == nullptr){
        freeState(scratch);
        freeState(previous);
        return true;
      }else{
        for(int i=1;i<intermediateStates.size();i++){ // 始点はふくまない
          copyState(scratch, intermediateStates[i]); // たまにIkの誤差でisValidではないことがあるが、やむなし
          if(distance(previous, scratch) < delta_) continue;
          NearProjectedStateSpace::StateType* tmp_state = static_cast<NearProjectedStateSpace::StateType*>(cloneState(scratch));
          tmp_state->intermediateStates.resize(1);
          tmp_state->intermediateStates[0] = cloneState(previous);
          geodesic->push_back(tmp_state);
          copyState(previous, scratch);
        }
        freeState(scratch);
        freeState(previous);
        for(int i=0;i+1<intermediateStates.size();i++) freeState(intermediateStates[i]);
        intermediateStates[0] = intermediateStates.back();
        intermediateStates.resize(1);
        return true;
      }
    }else if(intermediateStatesInv.size() > 0 && distance(intermediateStatesInv[0], to) < delta_) {
      if(geodesic == nullptr){
        freeState(scratch);
        freeState(previous);
        return true;
      }else{
        for(int i=intermediateStatesInv.size()-1;i>=1;i--){ // 始点はふくまない
          copyState(scratch, intermediateStatesInv[i]); // たまにIkの誤差でisValidではないことがあるが、やむなし
          if(distance(previous, scratch) < delta_) continue;
          NearProjectedStateSpace::StateType* tmp_state = static_cast<NearProjectedStateSpace::StateType*>(cloneState(scratch));
          tmp_state->intermediateStates.resize(1);
          tmp_state->intermediateStates[0] = cloneState(previous);
          geodesic->push_back(tmp_state);
          copyState(previous, scratch);
        }
        freeState(scratch);
        freeState(previous);
        for(int i=0;i+1<intermediateStatesInv.size();i++) freeState(intermediateStatesInv[i]);
        intermediateStatesInv[0] = intermediateStatesInv.back();
        intermediateStatesInv.resize(1);
        return true;
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
          if(step < delta_ / lambda_) // stacked
            break;

          // Check if we have wandered too far
          total += step;
          if (total > max)
            break;

          // Check if we are no closer than before
          const double newDist = distance(scratch, to);
          if (newDist >= dist)
            break;

          dist = newDist;

          // Store the new state
          if (geodesic != nullptr){
            NearProjectedStateSpace::StateType* tmp_state = static_cast<NearProjectedStateSpace::StateType*>(cloneState(scratch));
            tmp_state->intermediateStates.resize(1);
            tmp_state->intermediateStates[0] = cloneState(previous);
            geodesic->push_back(tmp_state);
          }else{
            path.push_back(cloneState(scratch));
          }
          copyState(previous, scratch);

        } while (dist >= tolerance);

      if(dist <= tolerance){
        for(int i=0;i<intermediateStates.size();i++) freeState(intermediateStates[i]);
        if (geodesic != nullptr) {
          intermediateStates.resize(1);
          intermediateStates[0] = cloneState(from);
        }else{
          intermediateStates = path;
        }
        return true;
      }else{
        for(int i=0;i<path.size();i++) freeState(path[i]);
        return false;
      }
    }
  }

  void NearProjectedStateSpace::interpolateRaw(const ompl::base::State *from, const ompl::base::State *to, const double t,
                                               ompl::base::State *state) const{
    WrapperStateSpace::interpolate(from, to, t, state);
  }


};
