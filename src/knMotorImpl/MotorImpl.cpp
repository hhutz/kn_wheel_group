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

#include "MotorImpl.h"

#include "knMotor/MotorFuture.h"

#include "miro/Log.h"

#include <iterator>
#include <limits>

#include <cmath>

namespace kn
{
  using namespace std;

  MotorImpl::MotorImpl(ACE_Recursive_Thread_Mutex& mutex,
		       ACE_Condition_Recursive_Thread_Mutex& condition,
		       MotorParameters const * params) :
    MotorHelper(params),
    m_mutex(mutex),
    m_condition(condition)
  {
  }
  
  MotorImpl::~MotorImpl() throw()
  {}

  MotorParameters const * 
  MotorImpl::params() const throw() 
  {
    return m_params;
  }

  void 
  MotorImpl::setParams(MotorParameters const * params) throw()
  {
    m_params = params;
  }

  int
  MotorImpl::calibrate(CalibrationMode mode)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
   
    if (mode != CALIBRATE_WAIT) {
      if (m_sample.status == KNMOTOR_BUSY_CALIBRATING) {
	return  m_sample.status;
      }
    }
    else if (m_sample.status != KNMOTOR_OKAY &&
	     !isError(m_sample.status, KNMOTOR_ERR_CALIBRATION)) {
      return m_sample.status;
    }

    // check if motor supports async calibration
    if (mode == CALIBRATE_ASYNC &&
	!this->calibrateAsync()) {
      return KNMOTOR_ERR_NOT_SUPPORTED;
    }
    
    m_sample.status = KNMOTOR_BUSY_CALIBRATING;
  
    m_condition.mutex().release();
    int rc = this->calibrateSkel(mode);
    m_condition.mutex().acquire();
      
    if (mode == CALIBRATE_SYNC ||
	(mode == CALIBRATE_ASYNC && rc != KNMOTOR_OKAY) ) {
      m_sample.status = rc;
    }
    
    // sync sample status with return code
    m_condition.broadcast();
    return rc;
  }

  int 
  MotorImpl::cmdStatus(bool waitForCompletion, ACE_Time_Value const * timeout)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    if (waitForCompletion) {
      while (m_sample.status > 0) {
        if (m_condition.wait(timeout) == -1 &&
            errno == ETIME)
          return KNMOTOR_ERR_TIMEOUT;
      }
    }

    return m_sample.status;
  }
    
  MotorSample 
  MotorImpl::motorState(bool waitNextSample, ACE_Time_Value const * timeout)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    if (waitNextSample) {
      ACE_Time_Value currentSample = m_sample.timestamp;
      while (m_sample.timestamp == currentSample) {
        if (m_condition.wait(timeout) == -1 &&
            errno == ETIME) {
	  MotorSample sample = m_sample;
	  sample.status |= KNMOTOR_ERR_TIMEOUT;
          return sample;
        }
      }
    }

    return m_sample;
  }

  void 
  MotorImpl::integrateData(MotorSample const& sample) throw()
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    MotorHelper::integrateData(sample);
    m_condition.broadcast();
  }

  int 
  MotorImpl::futureMotorState(MotorFuture& state, ACE_Time_Value const& targetTime)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    return collectFutureMotorState(state, targetTime);
  }
}
