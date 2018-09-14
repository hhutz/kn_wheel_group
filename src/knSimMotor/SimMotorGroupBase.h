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

#ifndef kn_SimMotorGroupBase_h
#define kn_SimMotorGroupBase_h

#include "knSimMotor_Export.h"

#include "knMotorImpl/MotorGroupImpl.h"

#include "ace/Recursive_Thread_Mutex.h"
#include "ace/Condition_Recursive_Thread_Mutex.h"

namespace kn
{
  class SimMotor;

  //! Abstract motor group interface.
  /**
   * Syncronized control of a set of wheels with drive and steer motors.
   * The primary customer for this interface is the implementation of the Locomotor IDL interface.
   */
  class knSimMotor_Export SimMotorGroupBase : public virtual MotorGroupImpl
  {
  public:
    typedef std::vector<SimMotor *> MotorVector;

    SimMotorGroupBase(MotorVector const& motors,
		      MotorGroupParameters const * params);
    virtual ~SimMotorGroupBase() throw();

    // Motor base interface

    virtual int cmdStatus(bool waitForCompletion, ACE_Time_Value const * timeout);
    virtual int calibrate(CalibrationMode mode);
    virtual int tryRecover();

    // Motor group interface
    virtual MotorGroupSample motorGroupState(bool waitNextSample, ACE_Time_Value const * timeout);
    virtual int futureMotorGroupState(MotorGroupFuture& state, const ACE_Time_Value& targetTime);

    // Simulation update
    virtual void executeTimeSlice(double frequency);
    virtual void setTime(ACE_Time_Value const& now);

  protected:
    void startTrajectories(double frequency);

    ACE_Recursive_Thread_Mutex m_mutex;
    ACE_Condition_Recursive_Thread_Mutex m_condition;

    MotorVector m_motors;
    ACE_Time_Value m_now;
  };
}
#endif // kn_SimMotorGroupBase_h
