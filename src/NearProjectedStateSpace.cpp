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
      for(int j=0;j<destination->as<StateType>()->intermediateStates[i].size();j++){
        freeState(destination->as<StateType>()->intermediateStates[i][j]);
      }
    }
    destination->as<StateType>()->intermediateStates.resize(source->as<StateType>()->intermediateStates.size());
    for(int i=0;i<source->as<StateType>()->intermediateStates.size();i++){
      destination->as<StateType>()->intermediateStates[i].resize(source->as<StateType>()->intermediateStates[i].size());
      for(int j=0;j<source->as<StateType>()->intermediateStates[i].size();j++){
        destination->as<StateType>()->intermediateStates[i][j] = cloneState(source->as<StateType>()->intermediateStates[i][j]);
      }
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

    NearProjectedStateSpace::StateType* previous = static_cast<NearProjectedStateSpace::StateType*>(cloneState(from));
    for(int j=0;j<previous->intermediateStates.size();j++) for(int k=0;k<previous->intermediateStates[j].size();k++) freeState(previous->intermediateStates[j][k]);
    previous->intermediateStates.clear();
    NearProjectedStateSpace::StateType*  scratch = static_cast<NearProjectedStateSpace::StateType*>(allocState());

    auto &&svc = si_->getStateValidityChecker();

    std::vector<ompl::base::State *> path{cloneState(from)};

    std::vector<std::vector<ompl::base::State*> >& intermediateStates = static_cast<const NearProjectedStateSpace::StateType*>(to)->intermediateStates;
    int idx = -1;
    for(int i=0;i<intermediateStates.size();i++){
      if(intermediateStates[i].size() > 0 && distance(intermediateStates[i][0], from) < delta_){
        idx = i;
        break;
      }
    }
    if(idx >= 0) {
      if(geodesic == nullptr){
        freeState(scratch);
        freeState(previous);
        return true;
      }else{
        for(int i=1;i<intermediateStates[idx].size();i++){ // 始点はふくまない
          copyState(scratch, intermediateStates[idx][i]); // たまにIkの誤差でisValidではないことがあるが、やむなし
          for(int j=0;j<scratch->intermediateStates.size();j++) for(int k=0;k<scratch->intermediateStates[j].size();k++) freeState(scratch->intermediateStates[j][k]);
          scratch->intermediateStates.clear();
          if(distance(previous, scratch) < delta_) continue;
          NearProjectedStateSpace::StateType* tmp_state = static_cast<NearProjectedStateSpace::StateType*>(cloneState(scratch));
          tmp_state->intermediateStates.push_back(std::vector<ompl::base::State*>{cloneState(previous)});
          geodesic->push_back(tmp_state);
          copyState(previous, scratch);
        }
        freeState(scratch);
        freeState(previous);
        if(intermediateStates[idx].size() > 1){
          intermediateStates.push_back(std::vector<ompl::base::State*>{cloneState(intermediateStates[idx].back())});
        }
        return true;
      }
    }

    std::vector<std::vector<ompl::base::State*> >& intermediateStatesInv = static_cast<const NearProjectedStateSpace::StateType*>(from)->intermediateStates;
    for(int i=0;i<intermediateStatesInv.size();i++){
      if(intermediateStatesInv[i].size() > 0 && distance(intermediateStatesInv[i][0], to) < delta_){
        idx = i;
        break;
      }
    }
    if(idx >= 0) {
      if(geodesic == nullptr){
        freeState(scratch);
        freeState(previous);
        return true;
      }else{
        for(int i=intermediateStatesInv[idx].size()-1;i>=1;i--){ // 始点はふくまない
          copyState(scratch, intermediateStatesInv[idx][i]); // たまにIkの誤差でisValidではないことがあるが、やむなし
          for(int j=0;j<scratch->intermediateStates.size();j++) for(int k=0;k<scratch->intermediateStates[j].size();k++) freeState(scratch->intermediateStates[j][k]);
          scratch->intermediateStates.clear();
          if(distance(previous, scratch) < delta_) continue;
          NearProjectedStateSpace::StateType* tmp_state = static_cast<NearProjectedStateSpace::StateType*>(cloneState(scratch));
          tmp_state->intermediateStates.push_back(std::vector<ompl::base::State*>{cloneState(previous)});
          geodesic->push_back(tmp_state);
          copyState(previous, scratch);
        }
        freeState(scratch);
        freeState(previous);
        if(intermediateStatesInv[idx].size() > 1){
          intermediateStatesInv.push_back(std::vector<ompl::base::State*>{cloneState(intermediateStatesInv[idx].back())});
        }
        return true;
      }
    }

    {
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
          NearProjectedStateSpace::StateType* tmp_state = static_cast<NearProjectedStateSpace::StateType*>(cloneState(scratch));
          for(int j=0;j<tmp_state->intermediateStates.size();j++) for(int k=0;k<tmp_state->intermediateStates[j].size();k++) freeState(tmp_state->intermediateStates[j][k]);
          tmp_state->intermediateStates.clear();

          path.push_back(tmp_state);
          if (geodesic != nullptr){
            geodesic->push_back(cloneState(tmp_state));
          }
          copyState(previous, scratch);

        } while (dist >= tolerance);

      if(dist <= tolerance){
        intermediateStates.push_back(path);
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
