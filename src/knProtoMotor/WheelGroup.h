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
 * Module: knProtoMotor
 * Author: Vinh To
 *
 *****************************************************************************/

#ifndef kn_protoMotor_WheelGroup_h
#define kn_protoMotor_WheelGroup_h

#include "knProtoMotor_Export.h"

#include "knMotorImpl/WheelGroupImpl.h"

#include "ace/Recursive_Thread_Mutex.h"
#include "ace/Condition_Recursive_Thread_Mutex.h"

namespace kn
{
  namespace protoMotor
  {
    class MotorInterface;

    //! Wheel group interface implementation for ProtoMotor motor control.
    /**
     * There is not really anything to add to what the base implementation provides.
     */
    class knProtoMotor_Export WheelGroup : public kn::WheelGroupImpl
    {
    public:
      WheelGroup(MotorInterface * interface,
                 MotorVector const& motors = MotorVector(),
                 WheelGroupParameters const * params = NULL,
                 WheelParametersVector const& wheelParams = WheelParametersVector());
      virtual ~WheelGroup() throw();

      //--------------------------------------------------
      // Motor interface

      virtual int cmdStatus(bool waitForCompletion = false, ACE_Time_Value const * timeout = NULL);
      virtual int calibrate(CalibrationMode mode);
      virtual int tryRecover();
      virtual int stop(StopMode mode);

      //--------------------------------------------------
      // Motor Group interface

      using WheelGroupImpl::cmdProfiles;
      using WheelGroupImpl::cmdTrajectories;

      virtual MotorGroupSample motorGroupState(bool waitNextSample = false, ACE_Time_Value const * timeout = NULL);
      virtual int futureMotorGroupState(MotorGroupFuture& state, const ACE_Time_Value& targetTime);

      //--------------------------------------------------
      // Wheel Group interface

      virtual WheelGroupSample wheelGroupState(bool waitNextSample = false, ACE_Time_Value const * timeout = NULL);
      virtual int futureWheelGroupState(kn::WheelGroupFuture& state, const ACE_Time_Value& targetTime = ACE_Time_Value::zero);
      virtual int cmdProfiles(std::vector<MotorProfile> const& profiles,
                              double curvature, double crabAngle = 0.);
      virtual int cmdTrajectories(int ref,
                                  double frequency,
                                  TrajectoryVector const& mPostions,
                                  TrajectoryVector const& mSpeeds,
                                  DoubleVector const& curvatures,
                                  DoubleVector const& curvatureRates,
                                  DoubleVector const& crabAngles,
                                  DoubleVector const& crabRates,
                                  DoubleVector const& speeds);

      MotorVector& motors() throw()
      {
        return m_motors;
      }

    protected:
      friend class MotorInterface;

      MotorInterface * m_interface;
    };
  }
}
#endif // kn_protoMotor_WheelGroup_h
