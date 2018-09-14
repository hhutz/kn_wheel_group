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

#include "MotorHelper.h"

#include "knMotor/MotorParameters.h"
#include "knMotor/MotorFuture.h"

#include "miro/Log.h"

#include <iterator>
#include <limits>
#include <iostream>
#include <cmath>

namespace 
{
  double const NANd = std::numeric_limits<double>::quiet_NaN();
}

namespace kn
{
  using namespace std;
  typedef MotorBase::DoubleVector DoubleVector;

  MotorHelper::MotorHelper(MotorParameters const * params) :
    m_params(params),
    m_sample(KNMOTOR_IDLE,
             ACE_Time_Value::zero,
             ACE_Time_Value::zero,
             MotorState(MotorProfile(MotorProfile::CTRL_OFF,
                                     MotorProfile::POS_REL,
                                     0.,
                                     0.,
                                     0.),
                        0, 
                        0),
             NANd,
             NANd),
    m_indexOffset(-1),
    m_frequency(0.)
  {
  }
  
  MotorHelper::~MotorHelper() throw()
  {}

  int
  MotorHelper::calibrateSkel(MotorBase::CalibrationMode mode)
  {
    int rc = KNMOTOR_OKAY;
    switch(mode)
    {
    case MotorBase::CALIBRATE_SYNC:
      rc = calibrateHook();
      break;
    case MotorBase::CALIBRATE_ASYNC:
      switch (activate()) {
      case -1:
	rc = KNMOTOR_ERR_OTHER;
	break;
      case 1:
	assert(false); // this should actually never happen...
	rc = KNMOTOR_BUSY_CALIBRATING; 
	break;
      default:
	rc = KNMOTOR_OKAY;
	break;
      }
      break;
    case MotorBase::CALIBRATE_WAIT:
      rc = (wait() == -1)? KNMOTOR_ERR_OTHER : m_sample.status;
      break;
    }
    return rc;
  }

  int
  MotorHelper::svc()
  {
    calibrateHook();
    return 0; // ignored anyways...
  }

  bool 
  MotorHelper::calibrateAsync() const throw()
  {
    return false; 
  }

  int 
  MotorHelper::calibrateHook()
  {
    return KNMOTOR_OKAY;
  }


  void 
  MotorHelper::integrateData(MotorSample const& sample) throw()
  {
    if (m_sample.status == KNMOTOR_BUSY ||
        m_sample.status == KNMOTOR_BUSY_INTERRUPTABLE) {
      onCommandCompletion();
    }
    m_sample = sample;
  }

  int 
  MotorHelper::collectFutureMotorState(MotorFuture& state, ACE_Time_Value const& targetTime)
  {
    // if we are idle or in error status
    if (m_sample.status != KNMOTOR_BUSY_INTERRUPTABLE &&
        m_sample.status != KNMOTOR_BUSY) {
      
      state.targetTime = m_sample.timestamp;
      state.position = m_sample.state.position;
      state.speed = m_sample.state.speed;

      return -1;
    }

    // if we're in trajectory mode but the motors have not moved yet,
    // reset to initial conditions
    if (m_sample.status == KNMOTOR_BUSY_INTERRUPTABLE &&
	m_indexOffset == 0) {
      state.targetTime = m_sample.timestamp;
      state.position = m_sample.state.position;
      state.speed = m_sample.state.speed;
      return -1;
    } 

    // if the next available slot was asked for or
    // if we are profile mode
    if (targetTime == ACE_Time_Value::zero ||
        m_sample.status == KNMOTOR_BUSY) {
      state.targetTime = m_sample.targetTime;
      state.position = m_sample.state.cmd.position;
      state.speed = m_sample.state.cmd.speed;

      return m_indexOffset - 1;
    }
    
    // if asked for a time not accessible
    if (targetTime < m_sample.targetTime) {
      cout << "MotorHelper::collectFutureMotorState asked for a time not accessible: targetTime sampleTime"
	   << targetTime << " " << m_sample.targetTime << endl;
      return KNMOTOR_ERR_TIMEOUT;
    }
      
    // calculate ref
    ACE_Time_Value deltaT = targetTime - m_sample.targetTime;
    double dT = deltaT.sec() + (deltaT.usec() / 1000000.);

    int ref = (int)ceil(dT * m_frequency);
    int samplesMax = max(m_positions.size(), m_speeds.size());

    // return end of buffers, if timestamp is in the future
    if (ref > samplesMax) {
      ACE_Time_Value t(1);
      t *= 1/m_frequency;
      t *= samplesMax;

      state.targetTime = m_sample.targetTime + t;
      state.position = (!m_positions.empty())? NANd : m_positions.front();
      state.speed = (!m_speeds.empty())? NANd : m_speeds.front();

      cout << "motor out of samples: " << endl;
      return -2;
    }

    // return the referenced slot
    state.targetTime = ACE_Time_Value(ref);
    state.targetTime *= 1./m_frequency;
    state.targetTime += targetTime;
    
    state.position = (m_positions.empty())? NANd : m_positions[m_positions.size() - 1 - ref];
    state.speed = (m_speeds.empty())? NANd : m_speeds[m_speeds.size() - 1 - ref];

    return ref + m_indexOffset;
  }

  void 
  MotorHelper::onCommandCompletion() throw()
  {
    m_positions.clear();
    m_speeds.clear();

    m_indexOffset = -1;
    m_frequency = 0.;
  }

  void
  MotorHelper::setStateStop(MotorBase::StopMode mode, ACE_Time_Value const& now) throw()
  {
    if (m_sample.status > KNMOTOR_IDLE) 
    {
      onCommandCompletion();

      MotorState& state = m_sample.state;
      
      m_sample.status = KNMOTOR_BUSY;
      m_sample.timestamp = now;

      state.cmd.posMode = MotorProfile::POS_REL;
      state.cmd.position = 0.;
      state.cmd.speed = 0;
      if (isnan(state.cmd.acc))
        state.cmd.acc = m_params->limits.maxAcceleration;
      
      switch (mode) {
      case MotorBase::STOP_EMERGENCY:
        state.cmd.ctrlMode = MotorProfile::CTRL_HOLD;
        state.cmd.acc = m_params->limits.maxAcceleration;
        //        m_sample.status = KNMOTOR_ERR_SW_ESTOP;
        break;
      case MotorBase::STOP_OFF:
        state.cmd.ctrlMode = MotorProfile::CTRL_HOLD;
        state.cmd.posMode = MotorProfile::POS_NA;
        state.cmd.position = NANd;
        state.cmd.speed = NANd;
        state.cmd.acc =  NANd;
        break;
      case MotorBase::STOP_ABRUPTLY:
        state.cmd.ctrlMode = MotorProfile::CTRL_HOLD;
        state.cmd.acc = m_params->limits.maxAcceleration;
        break;
      case MotorBase::STOP_SMOOTHLY:
        state.cmd.ctrlMode = MotorProfile::CTRL_HOLD;
        if (state.cmd.acc < 0.00001)
          state.cmd.acc = m_params->limits.maxAcceleration;
        break;
      }
      
      if (fabs(state.cmd.acc) < 0.00001 ||
          isnan(state.speed) ||
          isnan(state.cmd.acc))  {
        
        m_sample.targetTime = ACE_Time_Value::zero;
      }
      else {
        m_sample.targetTime.set(fabs(state.speed / state.cmd.acc));
        m_sample.targetTime += now;
      }
    }
    else if (false && mode == MotorBase::STOP_EMERGENCY) {
      if (m_sample.status >= KNMOTOR_IDLE) {
        m_sample.status = KNMOTOR_IDLE;
      }
      m_sample.status |= KNMOTOR_ERR_SW_ESTOP;     
      m_sample.timestamp = now;
    }
  }

  int
  MotorHelper::setStateCmdProfile(MotorProfile const& profile, 
                                  ACE_Time_Value const& now)
    throw()
  {
    if (m_sample.status != KNMOTOR_IDLE)
      return m_sample.status;

    MotorState& state = m_sample.state;

    state.cmd = profile;

    if (state.cmd.ctrlMode == MotorProfile::CTRL_POSITION) {
      state.cmd.speed = m_params->limits.maxVelocity;
      state.cmd.acc = m_params->limits.maxAcceleration;
    }

    // convert into absolute position command for book-keeping
    if (state.cmd.posMode == MotorProfile::POS_REL) {
      state.cmd.position += state.position;
      state.cmd.posMode = MotorProfile::POS_ABS;
    }

    // calculate time to target-position
    double distance = fabs(state.cmd.position - state.position);   // distance to target position
    double accTime = fabs(state.cmd.speed / state.cmd.acc);        // time to acc to target speed
    double accDist = .5 * fabs(state.cmd.acc) * accTime * accTime; // distance covered in acc
    double fullSpeedDistance = distance - 2. * accDist;            // distance travelled at full speed

    double t = 0.;
    if (fullSpeedDistance > 0. && fabs(state.cmd.speed) > 0.00001 ) {
      double fullSpeedTime = fullSpeedDistance / state.cmd.speed;  // time travelling at full speed
      t = 2 * accTime + fullSpeedTime;
    }
    else if (distance > 0.) {
      t = 2 * sqrt(2. * fabs((0.5 * distance) / state.cmd.speed)); // d = .5 * a * t*t
    }

    m_sample.timestamp = now;
    m_sample.targetTime.set(t);
    m_sample.targetTime += now;
    m_sample.status = KNMOTOR_BUSY;

    // set ref value for motorFuture
    m_indexOffset = -2;
    m_frequency = 0.;

    // redundant, just in case
    m_positions.clear();
    m_speeds.clear();

    return KNMOTOR_OKAY;
  }

  int 
  MotorHelper::setStateCmdTrajectory(int ref,
                                     double frequency,
                                     MotorBase::DoubleVector const& positions,
                                     MotorBase::DoubleVector const& speeds)
  {
    // only allowed if either idle, or in trajectory mode
    if (m_sample.status != KNMOTOR_IDLE &&
        m_sample.status != KNMOTOR_BUSY_INTERRUPTABLE)
      return m_sample.status;

    // we can only append if there is trajectory executed
    // we can only set a new trajectory, if idle
    // we can only append at an index that is not yet consumed
    if (ref == -1 && m_sample.status != KNMOTOR_IDLE) {
      MIRO_LOG(LL_ERROR, "setStateCmdTrajectory() - failed as Motor status not idle");
      return KNMOTOR_ERR_TIMEOUT;
    }
    if (ref >= 0 && m_sample.status != KNMOTOR_BUSY_INTERRUPTABLE) {
      MIRO_LOG(LL_ERROR, "setStateCmdTrajectory() - failed as Motor status status not busy-interruptable");
      return KNMOTOR_BUSY;
    }
    if ((ref + 1) < m_indexOffset && m_sample.status == KNMOTOR_BUSY_INTERRUPTABLE) {
      MIRO_LOG_OSTR(LL_ERROR, "setStateCmdTrajectory() - failed as ref + 1 (" << ref + 1 << ") < indexOffset(" << m_indexOffset << ")");
      return KNMOTOR_ERR_TIMEOUT;
    }

    // we don't change frequency on the fly
    if (ref != -1 &&
        frequency != m_frequency) {
      return KNMOTOR_ERR_TRAJ_CMD;
    }

    m_frequency = frequency;


    // cerr << "initial motor state" <<  m_sample << endl;


    // we are inserting behind ref
    ++ref;

    if (m_indexOffset == -1) {
      m_indexOffset = 0;
    }

    // chop trajectories
    size_t newSize = ref - m_indexOffset; 
    size_t maxSize = max(m_positions.size(), m_speeds.size());
    size_t steps = maxSize - newSize;
    
    if (!m_positions.empty()) {
      DoubleVector::iterator iter = m_positions.begin();
      advance(iter, steps);
      m_positions.erase(m_positions.begin(), iter);
    }
      
    if (!m_speeds.empty()) {
      DoubleVector::iterator iter = m_speeds.begin();
      advance(iter, steps);
      m_speeds.erase(m_speeds.begin(), iter);
    }

    // append trajectories
    m_positions.insert(m_positions.begin(), positions.rbegin(), positions.rend());
    m_speeds.insert(m_speeds.begin(), speeds.rbegin(), speeds.rend());

    assert(m_positions.empty() || m_speeds.empty() || m_positions.size() == m_speeds.size());

    m_sample.status = KNMOTOR_BUSY_INTERRUPTABLE;

    return 0;
  }

  void
  MotorHelper::consumeTrajectoryPoints(unsigned int num, 
                                       ACE_Time_Value const& now, ACE_Time_Value const& targetTime)
    throw()
  {
    ACE_Time_Value tT = targetTime;
    unsigned int newSize = 0;
    unsigned int maxSize = max(m_positions.size(), m_speeds.size());

    if (maxSize >= num) {
      newSize = maxSize - num;
    }
    else {
      // reduce lookahead if not enough points there
      ACE_Time_Value step;
      step.set(1./m_frequency);
      tT -= (num - maxSize) * step;

      num = maxSize;
    }

    if (num == 0)
      return;

    m_sample.status = KNMOTOR_BUSY_INTERRUPTABLE;
    m_sample.timestamp = now;
    m_sample.targetTime = tT;

    if (m_sample.targetTime == ACE_Time_Value::zero) {
      m_sample.targetTime.set(1.);
      m_sample.targetTime *= 1./m_frequency;
      m_sample.targetTime *= num;
      m_sample.targetTime += now;
    }

    m_sample.state.cmd.ctrlMode = MotorProfile::CTRL_TRAJECTORY;
    m_sample.state.cmd.posMode = MotorProfile::POS_ABS;


    consumeSamples(newSize, m_positions, m_sample.state.cmd.position);
    consumeSamples(newSize, m_speeds, m_sample.state.cmd.speed);

    m_indexOffset += num;
  }

  void
  MotorHelper::consumeSamples(unsigned int newSize, DoubleVector& samples, double& target) throw()
  {
    if (!samples.empty()) {
      DoubleVector::iterator iter = samples.begin();
      advance(iter, newSize);
      target = *iter;
      samples.erase(iter, samples.end());
    }
  }


}
