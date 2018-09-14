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

#include "SimMotorGroupBase.h"
#include "SimMotor.h"

#include "knMotor/MotorGroupSample.h"

#include <cmath>

using namespace std;

namespace kn
{
  SimMotorGroupBase::SimMotorGroupBase(MotorVector const& motors,
                               MotorGroupParameters const * params) :
    MotorGroupImpl(MotorGroupImpl::MotorVector(motors.begin(), motors.end()),
                   params),
    m_mutex(),
    m_condition(m_mutex),
    m_motors(motors),
    m_now(ACE_Time_Value::zero)
  {
  }

  SimMotorGroupBase::~SimMotorGroupBase() throw()
  {
  }

  int 
  SimMotorGroupBase::cmdStatus(bool waitForCompletion, ACE_Time_Value const * timeout)
  {
    return cmdStatusSkel(m_condition, waitForCompletion, timeout);
  }

  int
  SimMotorGroupBase::calibrate(CalibrationMode mode)
  {
    return calibrateSkel(m_condition, mode);
  }

  int
  SimMotorGroupBase::tryRecover()
  {
    // we assume all errors to be recoverable
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      (*first)->sample().status = KNMOTOR_OKAY;
    }

    m_sample->status = KNMOTOR_OKAY;

    return m_sample->status;
  }

  MotorGroupSample 
  SimMotorGroupBase::motorGroupState(bool waitNextSample, ACE_Time_Value const * timeout)
  {
    return motorGroupStateSkel(m_condition, waitNextSample, timeout);
  }

  int 
  SimMotorGroupBase::futureMotorGroupState(MotorGroupFuture& state, const ACE_Time_Value& targetTime)
  {
    // to maximize the time that this index is valid, 
    // we base this on the latest telemetry update
    ACE_Time_Value timeout = targetTime;
    if (timeout == ACE_Time_Value::zero)
      timeout = ACE_OS::gettimeofday() + ACE_Time_Value(0, 100000);
    
    MotorGroupSample sample = motorGroupState(true, &targetTime);
    if (isError(KNMOTOR_ERR_TIMEOUT, sample.status))
      return KNMOTOR_ERR_TIMEOUT;

    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
    return collectFutureMotorGroupState(state, targetTime);
  }

  // Simulation update
  void
  SimMotorGroupBase::executeTimeSlice(double frequency)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      (*first)->executeTimeSlice(frequency);
    }
    
 
    collectMotorStatus(m_condition, m_now);
  }

  void
  SimMotorGroupBase::setTime(ACE_Time_Value const& now)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
    m_now = now;

    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      (*first)->setTime(now);
    }
  }

  void
  SimMotorGroupBase::startTrajectories(double frequency)
  {
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      (*first)->startTrajectory(frequency);
    }
  }
}
