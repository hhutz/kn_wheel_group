/* -*- C++ -*- *****************************************************************
 * Copyright (c) 2013 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 *
 * Licensed under the NASA Open Source Agreement, Version 1.3 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   https://opensource.org/licenses/NASA-1.3
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************
 *
 * Project: RoverSw
 * Module: knSimMotor
 * Author: Hans Utz
 *
 *****************************************************************************/

#ifndef kn_SimWheelGroup_h
#define kn_SimWheelGroup_h

#include "knSimMotor_Export.h"
#include "SimMotorGroup.h"

#include "knMotorImpl/WheelGroupImpl.h"

namespace kn
{
  //! Abstract motor group interface.
  /**
   * Syncronized control of a set of wheels with drive and steer motors.
   * The primary customer for this interface is the implementation of the Locomotor IDL interface.
   */
  class knSimMotor_Export SimWheelGroup : public virtual WheelGroupImpl,
                                          public SimMotorGroupBase
  {
  public:
    SimWheelGroup(SimMotorGroup::MotorVector const& motors,
                  WheelGroupParameters const * params = NULL,
                  WheelParametersVector const& wheelParams = WheelParametersVector());
    virtual ~SimWheelGroup() throw();

    //--------------------------------------------------
    // Motor Base interface
    virtual int stop(StopMode mode);

    //--------------------------------------------------
    // Motor Group interface

    using WheelGroup::cmdProfiles;
    using WheelGroup::cmdTrajectories;

    //--------------------------------------------------
    // Wheel Group interface

    virtual WheelGroupSample wheelGroupState(bool waitNextSample, ACE_Time_Value const * timeout);
    virtual int futureWheelGroupState(kn::WheelGroupFuture& state, const ACE_Time_Value& targetTime);

    virtual int cmdProfiles(std::vector<MotorProfile> const& profiles,
                            double curvature, double crabAngle = 0);
    virtual int cmdTrajectories(int ref,
                                double frequency,
                                TrajectoryVector const& mPostions,
                                TrajectoryVector const& mSpeeds,
                                DoubleVector const& curvatures,
                                DoubleVector const& curvatureRates,
                                DoubleVector const& crabAngles,
                                DoubleVector const& crabRates,
                                DoubleVector const& speeds);

    // Simulation update
    virtual void executeTimeSlice(double frequency);

  protected:
    void trajectoryCtrl(double slice);
    void pushTrajectoryPoints(unsigned int num);
    void startTrajectory(double frequency);

    DoubleVector m_curvaturesQueue;
    DoubleVector m_curvatureRatesQueue;
    DoubleVector m_speedsQueue;
  };
}
#endif // kn_SimWheelGroup_h
