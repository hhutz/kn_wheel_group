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

#include "SimMotor.h"
#include "miro/TimeHelper.h"

#include <iterator>
#include <limits>
#include <sstream>
#include <cmath>

#include <ace/OS_NS_unistd.h>

namespace
{
  const double NANd = std::numeric_limits<double>::quiet_NaN();
}

namespace kn
{
  using namespace std;

  unsigned int const SimMotor::TARGET_LOOKAHEAD = 5;

  SimMotor::SimMotor(MotorParameters const * params) :
    MotorImpl(m_mutex, m_condition, params),
    m_mutex(),
    m_condition(m_mutex),
    m_now(ACE_Time_Value::zero)
  {
  }

  SimMotor::~SimMotor() throw()
  {}

  int SimMotor::tryRecover()
  {
    // we don't do error recovery, yet
    m_sample.status = KNMOTOR_OKAY;
    return m_sample.status;
  }

  void
  SimMotor::onCommandCompletion() throw()
  {
    MotorHelper::onCommandCompletion();
    m_positionQueue.clear();
    m_speedQueue.clear();
  }

  int
  SimMotor::stop(StopMode mode)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    if (m_sample.status <= KNMOTOR_IDLE)
      return m_sample.status;

    setStateStop(mode, m_now);
    m_condition.broadcast();

    return KNMOTOR_OKAY;
  }

  int
  SimMotor::cmdProfile(MotorProfile const& profile)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    int rc = setStateCmdProfile(profile, m_now);
    if (rc == KNMOTOR_OKAY)
      m_condition.broadcast();
    return rc;
  }

  int
  SimMotor::cmdTrajectory(int ref,
                          double frequency,
                          DoubleVector const& positions,
                          DoubleVector const& speeds)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    int status = m_sample.status;
    int rc = setStateCmdTrajectory(ref,
                                   frequency,
                                   positions,
                                   speeds);
    if (rc == KNMOTOR_OKAY) {
      if (status == KNMOTOR_IDLE) {
        // if we just started
        startTrajectory(frequency);
      }
      m_condition.broadcast();
    }
    
    return rc;
  }

  // Simulation update
  void
  SimMotor::executeTimeSlice(double frequency)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    double slice = 1./frequency;

    m_sample.timestamp = m_now;

    if (m_sample.status == KNMOTOR_BUSY ||
        m_sample.status == KNMOTOR_BUSY_INTERRUPTABLE) {

      switch (m_sample.state.cmd.ctrlMode) {
      case kn::MotorProfile::CTRL_POSITION:
      case kn::MotorProfile::CTRL_HOLD:
        velocityCtrl(slice);
        break;
      case kn::MotorProfile::CTRL_TRAPECOID:
      case kn::MotorProfile::CTRL_OFF:
        trapezoidCtrl(slice);
        break;
      case kn::MotorProfile::CTRL_TRAJECTORY:
        trajectoryCtrl(slice);
        break;
      }

      if (m_sample.status == KNMOTOR_IDLE) {
        onCommandCompletion();
      }
    }
    else {
      m_sample.targetTime = m_now;
    }

    m_condition.broadcast();
  }

  void
  SimMotor::setTime(ACE_Time_Value const& now)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
    m_now = now;
  }

  void
  SimMotor::velocityCtrl(double slice)
  {
    MotorState& state = m_sample.state;

    // breaking distance

    // v = a*t
    // s = .5 a*t*t

    // speed
    double targetSpeed = 0;

    // acc
    double acc = state.cmd.acc;

    // decide if we need to go below max-acc
    // to reach target speed
    if (fabs(state.speed - targetSpeed) < (acc*slice))
      acc = fabs(state.speed - targetSpeed) / slice;

    if (state.speed > targetSpeed)
      acc *= -1.;

    double accStep = state.speed * slice + .5 * acc * slice * slice;
 
    // update position
    state.position += accStep;
    state.speed += acc * slice;

    if (false && fabs(state.speed) > M_PI) {
      cerr << "runaway motor: " << m_sample << endl;
      abort();
      return;
    }
    // check for target reached
    if (fabs(state.speed) > 0.00001) {
      return;
    }

    // check for target reached
    state.speed = 0.;
    state.cmd.ctrlMode = MotorProfile::CTRL_POSITION;
    state.cmd.posMode = MotorProfile::POS_REL;
    state.cmd.position = 0;
    
    m_sample.status = KNMOTOR_IDLE;
  }

  void
  SimMotor::trapezoidCtrl(double slice)
  {
    MotorState& state = m_sample.state;

    // breaking distance

    // v = a*t
    // s = .5 a*t*t
    double t = state.speed / state.cmd.acc;

    double breakingDistance = .5 * state.cmd.acc * t * t;

    // distance
    double deltaT = (state.cmd.posMode == MotorProfile::POS_REL)?
      state.cmd.position :
      state.cmd.position - state.position;

    // speed
    double targetSpeed = fabs(state.cmd.speed);
    if (deltaT < 0.)
      targetSpeed = -targetSpeed;


    // constant speed step
    double step = state.speed * slice;

    // decide if we need to get to speed or slow down
    // slow down if target is within stopping distance
    // add the step delta, as that's our sampling resolution
    if ((fabs(deltaT) - breakingDistance) < fabs(step)) {
      targetSpeed = 0.;
    }
    // keep speed if we are on the breaking trajecotry
    else if ((fabs(deltaT) - breakingDistance) <= fabs(2. * step)) {
      targetSpeed = state.speed;
    }

    // acc
    double acc = state.cmd.acc;

    // if target-speed is zero, try to optimize for
    // position
    if (fabs(targetSpeed) < 0.00000001) {
      acc = .5 * state.speed * state.speed / fabs(deltaT);
    }

    // decide if we need to go below max-acc
    // to reach target speed
    if (fabs(state.speed - targetSpeed) < (acc*slice))
      acc = fabs(state.speed - targetSpeed) / slice;
    if (state.speed > targetSpeed)
      acc *= -1.;

    double accStep = state.speed * slice + .5 * acc * slice * slice;


    // check for target reached
    if (fabs(deltaT) <= fabs(step) &&
        fabs(state.speed) <= fabs(state.cmd.acc * slice)) {

      state.position += deltaT;
      state.speed = 0.;
      state.cmd.posMode = MotorProfile::POS_REL;
      state.cmd.position = 0;

      if (state.cmd.ctrlMode == MotorProfile::CTRL_TRAPECOID) {
        state.cmd.ctrlMode = MotorProfile::CTRL_HOLD;
      }

      m_sample.status = KNMOTOR_IDLE;

      return;
    }
 
    // update position
    state.position += accStep;
    state.speed += acc * slice;
    if (state.cmd.posMode == MotorProfile::POS_REL)
      state.cmd.position -= accStep;

  }

  void
  SimMotor::trajectoryCtrl(double slice)
  {
    ACE_Time_Value targetTime;
    targetTime.set(slice * SimMotor::TARGET_LOOKAHEAD);
    targetTime += m_now;

    pushTrajectoryPoints(1);

    // remove points from buffer and adjust target set points
    consumeTrajectoryPoints(1, m_now, targetTime);

    // TODO: reduce queues by targetTime - m_now
    // set position & speed

    MotorState& state = m_sample.state;

//     cerr << "m_p:" << m_positions.size() << "\t  " << "m_s:" << m_speeds.size() << "\t    "
//          << "pQ:" << m_positionQueue.size() << "\t  " << "qsQ:" << m_speedQueue.size() << endl;

    // check target reached
    if (m_positionQueue.empty() && m_speedQueue.empty()) {

      // breaking can't be interrupted 
      m_sample.status = KNMOTOR_BUSY;
      state.cmd.ctrlMode = MotorProfile::CTRL_HOLD;
      state.cmd.posMode = MotorProfile:: POS_REL;
      state.cmd.position = 0.;
      state.cmd.speed = 0.;
      state.cmd.acc = m_params->limits.maxAcceleration;

      if (m_sample.state.speed == 0.) {
        m_sample.status = KNMOTOR_IDLE;
        state.cmd.ctrlMode = MotorProfile::CTRL_POSITION;
      }

      return;
    }
  }

  void
  SimMotor::pushTrajectoryPoints(unsigned int num)
  {
//     // consume trajectory samples
//     if (m_now < m_sample.targetTime) {
//       ACE_Time_Value delta = m_sample.targetTime - m_now;
//       double d = (double)delta.sec() + (double) delta.usec() / 1000000.;
//       unsigned int remainingQueueSize = (int) floor(d * m_frequency);

//       cout << "remaining queue size: " << remainingQueueSize << endl;

//       m_positionQueue.resize(min(m_positionQueue.size(), remainingQueueSize));
//       m_speedQueue.resize(min(m_speedQueue.size(), remainingQueueSize));
//     }
//     // end of trajectory or buffer underrun
//     else {
//       m_positionQueue.clear();
//       m_speedQueue.clear();
//     }
    
    // copy the points to the "execute" buffers
    pushSamplesToQueue(m_positions, m_positionQueue, num);
    pushSamplesToQueue(m_speeds, m_speedQueue, num);

    MotorState& state = m_sample.state;
    if (!m_speedQueue.empty()) {
      state.cmd.speed  = m_speedQueue.front();
      state.speed  = m_speedQueue.back();
    }

    if ((m_speedQueue.empty() || isnan(m_speedQueue.front())) && 
	!m_positionQueue.empty()) {
      state.cmd.speed = m_positionQueue.front();
      if (state.cmd.posMode == MotorProfile::POS_ABS) {
	if (m_positionQueue.size() > 1) {
	  state.cmd.speed -= m_positionQueue[1];
	}
	else {
	  state.cmd.speed -= state.cmd.position;
	}
      }
      state.cmd.speed *= m_frequency;
    }

    if ((m_speedQueue.empty() || isnan(m_speedQueue.back())) &&
	!m_positionQueue.empty()) {
      state.speed = m_positionQueue.back();
      if (state.cmd.posMode == MotorProfile::POS_ABS) 
        state.speed -= state.position;
      state.speed *= m_frequency;
    }

    if (!m_positionQueue.empty()) {
      state.cmd.position = m_positionQueue.front();
      state.position = m_positionQueue.back();
      m_positionQueue.pop_back();
    }

    if (!m_speedQueue.empty()) {
      m_speedQueue.pop_back();
    }

    state.cmd.acc = NANd;
  }

  void
  SimMotor::startTrajectory(double frequency)
  {
    ACE_Time_Value targetTime(1);
    targetTime *= 1./frequency * (double)TARGET_LOOKAHEAD;
    targetTime += m_now;
    
    pushTrajectoryPoints(TARGET_LOOKAHEAD);
    consumeTrajectoryPoints(TARGET_LOOKAHEAD, m_now, targetTime);
    
//     cerr << "m_p:" << m_positions.size() << "\t  " << "m_s:" << m_speeds.size() << "\t    "
//          << "pQ:" << m_positionQueue.size() << "\t  " << "qsQ:" << m_speedQueue.size() << endl;
  }

  void
  SimMotor::pushSamplesToQueue(DoubleVector& feed, DoubleVector& queue, unsigned int num)
  {
    if (!feed.empty()) {
      unsigned int less = min((unsigned int)feed.size(), num);
      DoubleVector::iterator first = feed.begin();
      advance(first, feed.size() - less);
      queue.insert(queue.begin(), first, feed.end());
    }
  }

  bool 
  SimMotor::calibrateAsync() const throw() 
  {
    return true; 
  }

  int 
  SimMotor::calibrateHook()
  {
    stringstream s;
    s << "SimMotor: " << (void *)this << ": calibrating (1sec)" << endl;
    cerr << s.str();
    ACE_OS::sleep(ACE_Time_Value(1));
    stringstream t;
    t << "SimMotor: " << (void *)this << ": calibration done" << endl;
    cerr << t.str();

    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
    m_sample.status = KNMOTOR_IDLE;
    return  m_sample.status;
  }
}
