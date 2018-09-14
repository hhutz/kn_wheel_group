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

#include "SimWheelGroup.h"
#include "SimMotor.h"
#include <limits>

namespace
{
  const double NANd = std::numeric_limits<double>::quiet_NaN();
}

namespace kn
{
  using namespace std;

  SimWheelGroup::SimWheelGroup(SimMotorGroup::MotorVector const& motors,
                               WheelGroupParameters const * params,
                               WheelParametersVector const& wheelParams) :
    MotorGroupImpl(MotorGroupImpl::MotorVector(motors.begin(), motors.end()),
                   params),
    WheelGroupImpl(MotorGroupImpl::MotorVector(motors.begin(), motors.end()),
                   params, wheelParams),
    SimMotorGroupBase(motors, params)
  {
  }

  SimWheelGroup::~SimWheelGroup() throw()
  {
  }

  int
  SimWheelGroup::stop(StopMode mode)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    setStateStop(mode, m_now);
    m_condition.broadcast();

    return KNMOTOR_OKAY;
  }

  WheelGroupSample
  SimWheelGroup::wheelGroupState(bool waitNextSample, ACE_Time_Value const * timeout)
  {
    return wheelGroupStateSkel(m_condition, waitNextSample, timeout);
  }

  int
  SimWheelGroup::futureWheelGroupState(kn::WheelGroupFuture& state, const ACE_Time_Value& targetTime)
  {
    ACE_Time_Value timeout = targetTime;

    if (timeout == ACE_Time_Value::zero)
      // @FIXME: this is dependend on the servicing frequency
      timeout = ACE_OS::gettimeofday() + ACE_Time_Value(0, 500000);

    WheelGroupSample sample = wheelGroupState(true, &timeout);

    if (isError(KNMOTOR_ERR_TIMEOUT, sample.status))
      return KNMOTOR_ERR_TIMEOUT;

    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
    return collectFutureWheelGroupState(state, targetTime);
  }

  int
  SimWheelGroup::cmdProfiles(std::vector<MotorProfile> const& profiles,
                             double curvature, double crabAngle)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    int rc = setStateCmdProfiles(profiles,
                                 curvature, crabAngle,
                                 m_now);

    if (rc == KNMOTOR_OKAY) {
      m_condition.broadcast();
    }

    return rc;
  }

  int
  SimWheelGroup::cmdTrajectories(int ref,
                                 double frequency,
                                 TrajectoryVector const& mPositions,
                                 TrajectoryVector const& mSpeeds,
                                 DoubleVector const& curvatures,
                                 DoubleVector const& curvatureRates,
                                 DoubleVector const& crabAngles,
                                 DoubleVector const& crabRates,
                                 DoubleVector const& speeds)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    int status = m_wheelGroupSample.status;
    int rc = setStateCmdTrajectories(ref,
                                     frequency,
                                     mPositions,
                                     mSpeeds,
                                     curvatures,
                                     curvatureRates,
                                     crabAngles,
                                     crabRates,
                                     speeds);

    if (rc == KNMOTOR_OKAY) {
      if (status == KNMOTOR_IDLE) {
        // if we just started
        startTrajectory(frequency);
        startTrajectories(frequency);
      }

      m_condition.broadcast();
    }

    return rc;
  }

  void
  SimWheelGroup::executeTimeSlice(double frequency)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    double const slice = 1. / frequency;

    m_wheelGroupSample.timestamp = m_now;

    if (m_wheelGroupSample.status == KNMOTOR_BUSY_INTERRUPTABLE) {
      trajectoryCtrl(slice);
    }

    SimMotorGroupBase::executeTimeSlice(frequency);
  }


  void
  SimWheelGroup::trajectoryCtrl(double slice)
  {
    ACE_Time_Value targetTime;
    targetTime.set(slice * SimMotor::TARGET_LOOKAHEAD);
    targetTime += m_now;

    pushTrajectoryPoints(1);

    // remove points from buffer and adjust target set points
    consumeTrajectoryPoints(1, m_now, targetTime);
  }

  void
  SimWheelGroup::pushTrajectoryPoints(unsigned int num)
  {
    // copy the points to the "execute" buffers
    SimMotor::pushSamplesToQueue(m_curvatures, m_curvaturesQueue, num);
    SimMotor::pushSamplesToQueue(m_curvatureRates, m_curvatureRatesQueue, num);
    SimMotor::pushSamplesToQueue(m_speeds, m_speedsQueue, num);

    // copy the points to the "execute" buffers

    if (!m_curvaturesQueue.empty()) {
      m_wheelGroupSample.curvature  = m_curvaturesQueue.back();
      m_curvaturesQueue.pop_back();
    }

    if (!m_curvatureRatesQueue.empty()) {
      m_wheelGroupSample.curvatureRate  = m_curvatureRatesQueue.back();
      m_curvatureRatesQueue.pop_back();
    }

    if (!m_speedsQueue.empty()) {
      m_wheelGroupSample.speed  = m_speedsQueue.back();
      m_speedsQueue.pop_back();
    }
  }

  void
  SimWheelGroup::startTrajectory(double frequency)
  {
    ACE_Time_Value targetTime(1);
    targetTime *= 1. / frequency * (double)SimMotor::TARGET_LOOKAHEAD;
    targetTime += m_now;

    // crabing not supported in trajectory mode
    //    m_wheelGroupSample.targetCrabAngle = 0.;
    //    m_wheelGroupSample.crabAngle = 0.;

    // set NAN's in case vectors are empty
    if (m_curvatures.empty()) {
      m_wheelGroupSample.curvature = NANd;
      m_wheelGroupSample.targetCurvature = NANd;
    }

    if (m_curvatureRates.empty()) {
      m_wheelGroupSample.targetCurvatureRate = NANd;
      m_wheelGroupSample.curvatureRate = NANd;
    }

    if (m_crabAngles.empty()) {
      m_wheelGroupSample.crabAngle = NANd;
      m_wheelGroupSample.targetCrabAngle = NANd;
    }

    if (m_crabRates.empty()) {
      m_wheelGroupSample.targetCrabRate = NANd;
      m_wheelGroupSample.crabRate = NANd;
    }

    if (m_speeds.empty()) {
      m_wheelGroupSample.targetSpeed = NANd;
      m_wheelGroupSample.speed = NANd;
    }

    pushTrajectoryPoints(SimMotor::TARGET_LOOKAHEAD);
    consumeTrajectoryPoints(SimMotor::TARGET_LOOKAHEAD, m_now, targetTime);
  }
}
