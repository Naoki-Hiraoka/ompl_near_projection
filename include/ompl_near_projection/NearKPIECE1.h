/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef OMPL_NEAR_PROJECTION_NEARKPIECE1_H
#define OMPL_NEAR_PROJECTION_NEARKPIECE1_H

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl_near_projection/NearDiscretization.h>

namespace ompl_near_projection {
  namespace geometric {
    class NearKPIECE1 : public ompl::geometric::KPIECE1 {
    public:
      NearKPIECE1(const ompl::base::SpaceInformationPtr &si) :
        KPIECE1(si),
        disc2_([this](Motion *m) { freeMotion(m); })
      {}

      ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

      void setBorderFraction(double bp)
      {
        disc2_.setBorderFraction(bp);
      }
      double getBorderFraction() const
      {
        return disc2_.getBorderFraction();
      }
      void setup() override{
        KPIECE1::setup();
        disc2_.setDimension(projectionEvaluator_->getDimension());
      }
      void clear() override{
        KPIECE1::clear();
        disc2_.clear();
      }
      void getPlannerData(ompl::base::PlannerData &data) const override{
        Planner::getPlannerData(data);
        disc2_.getPlannerData(data, 0, true, lastGoalMotion_);
      }

    protected:
      NearDiscretization<Motion> disc2_; // デフォルトのものだと、複数スレッド並列で実行した場合に、複数スレッドが近くのcellにmotionを生成した場合に、それらのcellを過剰に忌避してしまう問題があった.
    };
  }
};

#endif
