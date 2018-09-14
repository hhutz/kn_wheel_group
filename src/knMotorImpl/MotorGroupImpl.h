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
 * Module: knMotorImpl
 * Author: Hans Utz
 *
 *****************************************************************************/

#ifndef kn_MotorGroupImpl_h
#define kn_MotorGroupImpl_h

#include "knMotorImpl_Export.h"

#include "knMotor/MotorGroup.h"
#include "knMotor/MotorGroupSample.h"

#include <ace/Synch_Traits.h>

namespace kn
{
  class MotorHelper;

  //! Abstract motor group interface.
  /**
   * Syncronized control of a set of wheels with drive and steer motors.
   * The primary customer for this interface is the implementation of the Locomotor IDL interface.
   */
  class knMotorImpl_Export MotorGroupImpl : public virtual MotorGroup
  {
  public:
    typedef std::vector<MotorHelper *> MotorVector;
    
    //! Ctor
    /** The motor-group parameters can be passed on construction,
     * or explicitly set later. Parameters are considered constant though.
     * The class does not take ownership of the parameters.
     */
    MotorGroupImpl(MotorVector const& motors = MotorVector(),
                   MotorGroupParameters const * params = NULL);
    virtual ~MotorGroupImpl() throw();
    
    // Motor base interface

    virtual unsigned int numMotors() const throw() {
      return m_params->motors.size();
    }
    virtual MotorGroupParameters const * params() const throw() {
      return m_params;
    }
    virtual void setParams(MotorGroupParameters const * params) throw() {
      m_params = params;
    }

    //virtual int cmdStatus(bool waitForCompletion = false, ACE_Time_Value * timeout = NULL);

    // Motor interface

    //virtual MotorGroupSample motorGroupState(bool waitNextSample = false, ACE_Time_Value * timeout = NULL);
    //virtual int futureMotorGroupState(MotorGroupFuture& state, ACE_Time_Value const& targetTime);

    virtual void collectMotorStatus(ACE_Condition_Recursive_Thread_Mutex& condition,
				    ACE_Time_Value const& timestamp = ACE_Time_Value::zero);

  protected:
    int calibrateSkel(ACE_Condition_Recursive_Thread_Mutex& condition, CalibrationMode mode);
    int cmdStatusSkel(ACE_Condition_Recursive_Thread_Mutex& condition,
		      bool waitForCompletion, ACE_Time_Value const * timeout);
    MotorGroupSample motorGroupStateSkel(ACE_Condition_Recursive_Thread_Mutex& condition,
					 bool waitNextSample = false, ACE_Time_Value const * timeout = NULL);

    int collectFutureMotorGroupState(MotorGroupFuture& state, ACE_Time_Value const& targetTime);

    void setStateStop(StopMode mode, ACE_Time_Value const& now) throw();
    int setStateCmdProfiles(MotorProfileVector const& profile, ACE_Time_Value const& now);
    int setStateCmdTrajectories(int ref,
				double frequency,
				TrajectoryVector const& postions,
				TrajectoryVector const& speeds);

    //! Pointer to the motor-group parameters.
    /** Note, that MotorGroup does not take ownership. */
    MotorGroupParameters const * m_params;
    MotorVector m_motors;

    MotorGroupSample m_motorGroupSample;
    MotorGroupSample * m_sample;
  };
}
#endif // kn_MotorGroupImpl_h
