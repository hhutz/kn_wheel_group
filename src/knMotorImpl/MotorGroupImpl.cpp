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

#include "MotorGroupImpl.h"
#include "MotorHelper.h"

#include "knMotor/MotorGroupSample.h"
#include "knMotor/MotorFuture.h"

#include "knMotorShare/MotorGroupFuture.h"

#include "miro/Log.h"

#include "ace/Guard_T.h"
#include "ace/Recursive_Thread_Mutex.h"
#include "ace/Condition_Recursive_Thread_Mutex.h"

#include <cmath>
#include <cassert>

using namespace std;

namespace kn
{
  MotorGroupImpl::MotorGroupImpl(MotorVector const& motors,
                                 MotorGroupParameters const * params) :
    m_params(params),
    m_motors(motors),
    m_sample(&m_motorGroupSample)
  {
    m_sample->status = KNMOTOR_IDLE;
    m_sample->motors.resize(motors.size());
  }

  MotorGroupImpl::~MotorGroupImpl() throw()
  {
  }

  int
  MotorGroupImpl::calibrateSkel(ACE_Condition_Recursive_Thread_Mutex& condition, CalibrationMode mode)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(condition.mutex());
      
    if (mode != CALIBRATE_WAIT) {
      if (m_sample->status == KNMOTOR_BUSY_CALIBRATING) {
	return  m_sample->status;
      }
    }
    else if (m_sample->status != KNMOTOR_OKAY &&
	     !isError(m_sample->status, KNMOTOR_ERR_CALIBRATION)) {
      return m_sample->status;
    }
    
    // check if all motors support async calibration
    if (mode == CALIBRATE_ASYNC) {
      bool async = false;
      MotorVector::const_iterator first, last = m_motors.end();
      for (first = m_motors.begin(); first != last; ++first) {
	async |= (*first)->calibrateAsync();
      }

      if (async == false) {
	return KNMOTOR_ERR_NOT_SUPPORTED;
      }
    }
    
    m_sample->status = KNMOTOR_BUSY_CALIBRATING;
  
    int rc = KNMOTOR_OKAY;
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      MotorHelper * motor = *first; 
      motor->sample().status = KNMOTOR_BUSY_CALIBRATING;
      
      condition.mutex().release();
      int rc = motor->calibrateSkel(mode);
      condition.mutex().acquire();
      
      if (mode == CALIBRATE_SYNC ||
	  (mode == CALIBRATE_ASYNC && rc != KNMOTOR_OKAY) ) {
	motor->sample().status = rc;
      }

      if (rc != KNMOTOR_OKAY)
	break;
    }
    
    // sync sample status with return code
    collectMotorStatus(condition, m_sample->timestamp);
    return rc;
  }

  int
  MotorGroupImpl::cmdStatusSkel(ACE_Condition_Recursive_Thread_Mutex& condition,
				bool waitForCompletion, ACE_Time_Value const * timeout)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(condition.mutex());

    if (waitForCompletion) {
      while (m_sample->status > 0) {
        if (condition.wait(timeout) == -1 &&
	    errno == ETIME)
          return KNMOTOR_ERR_TIMEOUT;
      }
    }

    return m_sample->status;
  }

  MotorGroupSample
  MotorGroupImpl::motorGroupStateSkel(ACE_Condition_Recursive_Thread_Mutex& condition,
				      bool waitNextSample, ACE_Time_Value const * timeout)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(condition.mutex());

    if (waitNextSample) {
      ACE_Time_Value currentSample = m_sample->timestamp;
      while (m_sample->timestamp == currentSample) {
        if (condition.wait(timeout) == -1 &&
            errno == ETIME) {
	  MotorGroupSample sample = *m_sample;
	  sample.status |= KNMOTOR_ERR_TIMEOUT;
          return sample;
        }
      }
    }

    return *m_sample;
  }

  int 
  MotorGroupImpl::collectFutureMotorGroupState(MotorGroupFuture& state, ACE_Time_Value const& targetTime)
  {
    int maxRef = -2;
    state.targetTime = ACE_Time_Value::zero;
    state.motorPositions.reserve(numMotors());
    state.motorPositions.clear();
    state.motorSpeeds.reserve(numMotors());
    state.motorSpeeds.clear();
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      MotorFuture future;
      int ref = (*first)->collectFutureMotorState(future, targetTime);

      if (ref < -2)
	return ref;

      maxRef = max(ref, maxRef);
      state.targetTime = max(future.targetTime, state.targetTime);
      state.motorPositions.push_back(future.position);
      state.motorSpeeds.push_back(future.speed);
    }
    
    return maxRef;
  }

  void
  MotorGroupImpl::setStateStop(StopMode mode, ACE_Time_Value const& now) throw()
  {
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      (*first)->setStateStop(mode, now);
    }

    m_sample->timestamp = now;
    m_sample->motorStatus.clear();

    m_sample->status = KNMOTOR_OKAY;
    for (first = m_motors.begin(); first != last; ++first) {
      MotorSample const& sample = (*first)->sample();
      m_sample->motorStatus.push_back(sample.status);
 
      if (m_sample->status >= 0) {
        if (sample.status != 0) {
          m_sample->status = sample.status;
        }
      }
      else {
        if (sample.status < 0) {
          m_sample->status |= sample.status;
        }
      }
    }
  }

  int
  MotorGroupImpl::setStateCmdProfiles(MotorProfileVector const& profiles,
				      ACE_Time_Value const& now)
  {
    if (false) {
      cout << "motor profiles: " << endl;
      MotorProfileVector::const_iterator first, last = profiles.end();
      for (first = profiles.begin(); first != last; ++first) {
        cout << *first << endl;
      }
    }

    if (m_sample->status != KNMOTOR_IDLE)
      return m_sample->status;

    if (profiles.size() != m_motors.size())
      return KNMOTOR_ERR_NUM_MOTORS;
    
    MotorProfileVector::const_iterator iter = profiles.begin();
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first, ++iter) {
      (*first)->setStateCmdProfile(*iter, now);
    }
    
    m_sample->status = KNMOTOR_BUSY;
    m_sample->timestamp = now;

    if (m_sample->status < 0) {
      cout << "state: " << endl
           << *m_sample << endl;
    }
    return KNMOTOR_OKAY;
  }

  int
  MotorGroupImpl::setStateCmdTrajectories(int ref,
                                          double frequency,
                                          TrajectoryVector const& positions,
                                          TrajectoryVector const& speeds)
  {
    if (positions.empty() && speeds.empty())
      return KNMOTOR_ERR_TRAJ_CMD;

    if ((!positions.empty() && positions.size() != numMotors()) ||
        (!speeds.empty() && speeds.size() != numMotors()))
      return KNMOTOR_ERR_NUM_MOTORS;

    {
      unsigned int s = (!positions.empty())? positions.front().size() : speeds.front().size();
      if (!positions.empty()) {
        TrajectoryVector::const_iterator first, last = positions.end();
        for (first = positions.begin(); first != last; ++first) {
          if (first->size() != s)
            return KNMOTOR_ERR_TRAJ_CMD;
        }
      }
      
      if (!speeds.empty()) {
        TrajectoryVector::const_iterator first, last = speeds.end();
        for (first = speeds.begin(); first != last; ++first) {
          if (first->size() != s)
            return KNMOTOR_ERR_TRAJ_CMD;
        }
      }
    }

    if (m_sample->status != KNMOTOR_IDLE &&
          m_sample->status != KNMOTOR_BUSY_INTERRUPTABLE)
      return m_sample->status;

    // we can only append if there is trajectory executed
    // we can only set a new trajectory, if idle
    if (ref == -1 && m_sample->status != KNMOTOR_IDLE) {
      MIRO_LOG(LL_ERROR, "setStateCmdTrajectory() - failed as Motor status not idle");
      return KNMOTOR_ERR_TIMEOUT;
    }
    if (ref >= 0 && m_sample->status != KNMOTOR_BUSY_INTERRUPTABLE) {
      MIRO_LOG(LL_ERROR, "setStateCmdTrajectory() - failed as Motor status status not busy-interruptable");
      return KNMOTOR_BUSY;
    }

    TrajectoryVector dummy(1);
    TrajectoryVector::const_iterator p(dummy.begin());
    TrajectoryVector::const_iterator s(dummy.begin());
    if(!positions.empty())
      p = positions.begin();
    if(!speeds.empty())
      s = speeds.begin();
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      int rc = (*first)->setStateCmdTrajectory(ref, frequency, *p, *s);
      if (rc != KNMOTOR_OKAY) {
        return rc;
      }

      if (p != dummy.begin())
        ++p;
      if (s != dummy.begin())
        ++s;
    }

    m_sample->status = KNMOTOR_BUSY_INTERRUPTABLE;

     return KNMOTOR_OKAY;
  }

  void
  MotorGroupImpl::collectMotorStatus(ACE_Condition_Recursive_Thread_Mutex& condition,
                                     ACE_Time_Value const& timestamp)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(condition.mutex());

    m_sample->timestamp = timestamp;
    m_sample->motors.clear();
    m_sample->motorStatus.clear();
    m_sample->currents.clear();
    m_sample->temperatures.clear();

    bool active = m_sample->status > 0;

    m_sample->status = KNMOTOR_OKAY;
    MotorVector::const_iterator first, last = m_motors.end();
    for (first = m_motors.begin(); first != last; ++first) {
      MotorSample const& sample = (*first)->sample();
      m_sample->motors.push_back(sample.state);
      m_sample->motorStatus.push_back(sample.status);
 
      if (m_sample->status >= 0) {
        if (sample.status != 0) {
          m_sample->status = sample.status;
        }
      }
      else {
        if (sample.status < 0) {
          m_sample->status |= sample.status;
        }
      }
      m_sample->currents.push_back(sample.current);
      m_sample->temperatures.push_back(sample.temperature);

      if (sample.targetTime <  m_sample->targetTime)
        m_sample->targetTime = sample.targetTime;
      if (sample.timestamp <  m_sample->timestamp) {
        //    assert (sample.timestamp != ACE_Time_Value::zero);
        m_sample->timestamp = sample.timestamp;
      }
    }

    bool nonNanCurrents = false;
    bool nonNanTemperatures = false;

    for (unsigned int i = 0; i < m_sample->motors.size(); ++i) {
      if (m_sample->currents[i] == m_sample->currents[i]) {
        nonNanCurrents = true;
      }
      if (m_sample->temperatures[i] == m_sample->temperatures[i]) {
        nonNanTemperatures = true;
      }
    }

    if (isError(KNMOTOR_OKAY, m_sample->status))
      m_sample->motorStatus.clear();
    if (!nonNanCurrents)
      m_sample->currents.clear();
    if (!nonNanTemperatures)
      m_sample->temperatures.clear();

    condition.broadcast();

    if (active && m_sample->status < 0) {
      cout << "group: completed with error: " << m_sample->status << endl;
      for (first = m_motors.begin(); first != last; ++first) {
	MotorSample sample = (*first)->sample();
	cout << sample.status << " ";
      }
      cout << endl;
    }
  }
}

