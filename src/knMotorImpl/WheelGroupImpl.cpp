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

#include "WheelGroupImpl.h"
#include "MotorImpl.h"

#include "knMotor/MotorParameters.h"

#include "knMotorShare/WheelGroupFuture.h"

#include "miro/Log.h"

#include <ace/Guard_T.h>
#include <ace/Recursive_Thread_Mutex.h>
#include <ace/Condition_T.h>

#include <limits>

#include <cassert>

namespace
{
  double const NANd = std::numeric_limits<double>::quiet_NaN();
}


namespace kn
{
  using namespace std;

  WheelGroupImpl::WheelGroupImpl(MotorVector const& motors,
                                 WheelGroupParameters const * params,
                                 WheelParametersVector const& wheelParams) :
    MotorGroupImpl(motors, params),
    m_params(params),
    m_wheelParams(wheelParams),
    m_indexOffset(-1),
    m_frequency(0.)
  {
    assert(m_wheelParams.empty() ||
           (params != NULL && numWheels() == m_wheelParams.size()));

    m_wheelGroupSample.status = KNMOTOR_IDLE;
    m_wheelGroupSample.motors.resize(motors.size());
    m_wheelGroupSample.curvature = NANd;
    m_wheelGroupSample.curvatureRate = 0.;
    m_wheelGroupSample.speed = 0.;
    m_wheelGroupSample.crabAngle = 0.;
    m_wheelGroupSample.crabRate = 0.;

    m_sample = &m_wheelGroupSample;

    if (params != NULL && m_wheelParams.empty())
      m_wheelParams.resize(numWheels(), NULL);
  }

  WheelGroupImpl::~WheelGroupImpl() throw()
  {
  }

  void
  WheelGroupImpl::setParams(MotorGroupParameters const * params) throw()
  {
    WheelGroupParameters const * p =
      dynamic_cast<WheelGroupParameters const *>(params);

    m_params = p;
    MotorGroupImpl::m_params = p;
    m_wheelParams.resize(numMotors(), NULL);
  }

  void
  WheelGroupImpl::setWheelParams(unsigned int index, WheelParameters const * params) throw()
  {
    if (index < numWheels()) {
      m_wheelParams[index] = params;
    }
  }

  int
  WheelGroupImpl::cmdProfiles(std::vector<MotorProfile> const& profiles)
  {
    return cmdProfiles(profiles, NANd, 0);
  }

  int
  WheelGroupImpl::cmdTrajectories(int ref,
                                  double frequency,
                                  TrajectoryVector const& positions,
                                  TrajectoryVector const& speeds)
  {
    DoubleVector curvatures(positions.size(), NANd);
    return cmdTrajectories(ref, frequency, positions, speeds,
                           curvatures, DoubleVector(), DoubleVector(),
                           DoubleVector(), DoubleVector());
  }

  WheelGroupSample
  WheelGroupImpl::wheelGroupStateSkel(ACE_Condition_Recursive_Thread_Mutex& condition,
                                      bool waitNextSample, ACE_Time_Value const * timeout)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(condition.mutex());

    if (waitNextSample) {
      ACE_Time_Value currentSample = m_wheelGroupSample.timestamp;

      while (m_wheelGroupSample.timestamp == currentSample) {
        if (condition.wait(timeout) == -1 &&
            errno == ETIME) {
          WheelGroupSample sample = m_wheelGroupSample;
          sample.status |= KNMOTOR_ERR_TIMEOUT;
          return sample;
        }
      }
    }

    return m_wheelGroupSample;
  }

  int
  WheelGroupImpl::collectFutureWheelGroupState(WheelGroupFuture& state, ACE_Time_Value const& targetTime)
  {
    int ref = collectFutureMotorGroupState(state, targetTime);

    if (ref < -2)
      return ref;

    int index = ref - m_indexOffset;

    if (index == -1) {
      state.curvature = m_wheelGroupSample.targetCurvature;
      state.curvatureRate = m_wheelGroupSample.targetCurvatureRate;
      state.crabAngle = m_wheelGroupSample.targetCrabAngle;
      state.crabRate = m_wheelGroupSample.targetCrabRate;
      state.speed = m_wheelGroupSample.targetSpeed;
    }
    else if (ref >= 0) {
      // We don't know if the wheels are aligned, so append alignment path
      int telemetryOffset = 0;

      if (m_params->telemetryIndex) {
        telemetryOffset = index;
      }

      state.curvature = (m_curvatures.empty()) ? NANd : m_curvatures[m_curvatures.size() - telemetryOffset];
      state.curvatureRate = (m_curvatureRates.empty()) ? NANd : m_curvatureRates[m_curvatureRates.size() - telemetryOffset];
      state.crabAngle = (m_crabAngles.empty()) ? NANd : m_crabAngles[m_crabAngles.size() - telemetryOffset];
      state.crabRate = (m_crabRates.empty()) ? NANd : m_crabRates[m_crabRates.size() - telemetryOffset];
      state.speed = (m_speeds.empty()) ? NANd : m_speeds[m_speeds.size() - telemetryOffset];
    }
    else if (ref == -2) {
      // We don't know if the wheels are aligned, so append alignment path
      state.curvature = (m_curvatures.empty()) ? NANd : m_curvatures.front();
      state.curvatureRate = (m_curvatureRates.empty()) ? NANd : m_curvatureRates.front();
      state.crabAngle = (m_crabAngles.empty()) ? NANd : m_crabAngles.front();
      state.crabRate = (m_crabRates.empty()) ? NANd : m_crabRates.front();
      state.speed = (m_speeds.empty()) ? NANd : m_speeds.front();
    }
    else if (ref == -1) {
      state.curvature = m_wheelGroupSample.curvature;
      state.curvatureRate = m_wheelGroupSample.curvatureRate;
      state.crabAngle = m_wheelGroupSample.crabAngle;
      state.crabRate = m_wheelGroupSample.crabRate;
      state.speed = m_wheelGroupSample.speed;
    }

    ATrans2& offset = state.offset;
    offset = ATrans2::Identity();

    if (ref > 0) {
      ATrans2 delta;

      double const timeSlice = 1. / m_frequency;

      DoubleVector::const_iterator curvature, curvatureLast = m_curvatures.end();
      DoubleVector::const_iterator crab = m_crabAngles.begin();
      DoubleVector::const_iterator crabRate = m_crabRates.begin();
      DoubleVector::const_iterator speed = m_speeds.begin();

      for (curvature = m_curvatures.begin(); curvature != curvatureLast;
           ++curvature, ++crab, ++crabRate, ++speed) {
        offset = step(offset, *curvature, *crab, *crabRate, *speed, timeSlice);
      }
    }

    return ref;
  }

  void
  WheelGroupImpl::setStateStop(StopMode mode, ACE_Time_Value const& now)
  {
    MotorGroupImpl::setStateStop(mode, now);

    if (m_wheelGroupSample.status != KNMOTOR_IDLE) {
      // chop trajectory vectors
      m_curvatures.clear();
      m_curvatureRates.clear();
      m_crabAngles.clear();
      m_crabRates.clear();
      m_speeds.clear();
      m_indexOffset = -1;

      // update the vehicle state
      if (fabs(m_wheelGroupSample.curvatureRate) > 0.001 ||
          fabs(m_wheelGroupSample.targetCurvatureRate) > 0.001 ||
          fabs(m_wheelGroupSample.targetCurvature - fabs(m_wheelGroupSample.curvature)) > 0.001) {
        m_wheelGroupSample.curvature = NANd;
        m_wheelGroupSample.targetCurvature = NANd;
        m_wheelGroupSample.curvatureRate = NANd;
      }

      if (fabs(m_wheelGroupSample.crabRate) > 0.001 ||
          fabs(m_wheelGroupSample.targetCrabRate) > 0.001 ||
          fabs(m_wheelGroupSample.targetCrabAngle - fabs(m_wheelGroupSample.crabAngle)) > 0.001) {
        m_wheelGroupSample.crabAngle = 0.;
        m_wheelGroupSample.targetCrabAngle = 0.;
      }

      m_wheelGroupSample.speed = NANd;

      m_wheelGroupSample.targetCurvatureRate = 0.;
      m_wheelGroupSample.targetCrabRate = 0.;
      m_wheelGroupSample.targetSpeed = 0.;
    }
  }

  int
  WheelGroupImpl::setStateCmdProfiles(std::vector<MotorProfile> const& profiles,
                                      double curvature, double crabAngle,
                                      ACE_Time_Value const& now)
  {
    int rc = MotorGroupImpl::setStateCmdProfiles(profiles, now);

    if (rc == KNMOTOR_OKAY) {
      m_wheelGroupSample.curvature = curvature;
      m_wheelGroupSample.curvatureRate = 0.;
      m_wheelGroupSample.speed = NANd;
      m_wheelGroupSample.crabAngle = crabAngle;
      m_wheelGroupSample.crabRate = 0.;

      m_wheelGroupSample.targetCurvature = curvature;
      m_wheelGroupSample.targetCurvatureRate = 0.;
      m_wheelGroupSample.targetSpeed = 0.;
      m_wheelGroupSample.targetCrabAngle = crabAngle;
      m_wheelGroupSample.targetCrabRate = 0.;

      // set ref value for motorFuture
      m_indexOffset = -2;
      m_frequency = 0.;

      // redundant, just in case
      m_curvatures.clear();
      m_curvatureRates.clear();
      m_crabAngles.clear();
      m_crabRates.clear();
      m_speeds.clear();
    }

    return rc;
  }

  int
  WheelGroupImpl::setStateCmdTrajectories(int ref,
                                          double frequency,
                                          TrajectoryVector const& mPositions,
                                          TrajectoryVector const& mSpeeds,
                                          DoubleVector const& curvatures,
                                          DoubleVector const& curvatureRates,
                                          DoubleVector const& crabAngles,
                                          DoubleVector const& crabRates,
                                          DoubleVector const& speeds)
  {
    //    cerr << "initial wheel group state\n" <<  m_wheelGroupSample << endl;

    int rc = MotorGroupImpl::setStateCmdTrajectories(ref, frequency, mPositions, mSpeeds);

    if (rc != KNMOTOR_OKAY)
      return rc;

    unsigned int s = (!mPositions.empty()) ? mPositions.front().size() : mSpeeds.front().size();

    if (!curvatures.empty() && curvatures.size() != s) {
      return KNMOTOR_ERR_TRAJ_CMD;
    }

    if (!curvatureRates.empty() && curvatureRates.size() != s) {
      return KNMOTOR_ERR_TRAJ_CMD;
    }

    if (!crabAngles.empty() && crabAngles.size() != s) {
      return KNMOTOR_ERR_TRAJ_CMD;
    }

    if (!crabRates.empty() && crabRates.size() != s) {
      return KNMOTOR_ERR_TRAJ_CMD;
    }

    if (!speeds.empty() && speeds.size() != s) {
      return KNMOTOR_ERR_TRAJ_CMD;
    }

    // we can only append at an index that is not yet consumed
    if ((ref + 1) < m_indexOffset && m_sample->status == KNMOTOR_BUSY_INTERRUPTABLE) {
      MIRO_LOG_OSTR(LL_ERROR, "setStateCmdTrajectory() - failed as ref + 1 (" << ref + 1 << ") < indexOffset(" << m_indexOffset << ")");
      return KNMOTOR_ERR_TIMEOUT;
    }

    m_frequency = frequency;

    // we are inserting behind ref
    ++ref;

    if (m_indexOffset == -1)
      m_indexOffset = 0;

    // if index-offset is zero, this calculates wrong results
    // they are ignored, but this should be resolved anyways.

    // chop trajectories
    unsigned int newSize = ref - m_indexOffset;
    unsigned int maxSize = max(
                             m_speeds.size(),
                             max(
                               max(m_curvatures.size(), m_curvatureRates.size()),
                               max(m_crabAngles.size(), m_crabRates.size())
                             )
                           );

    unsigned int steps = maxSize - newSize;

    if (!m_curvatures.empty()) {
      DoubleVector::iterator iter = m_curvatures.begin();
      advance(iter, steps);
      m_curvatures.erase(m_curvatures.begin(), iter);
    }

    if (!m_curvatureRates.empty()) {
      DoubleVector::iterator iter = m_curvatureRates.begin();
      advance(iter, steps);
      m_curvatureRates.erase(m_curvatureRates.begin(), iter);
    }

    if (!m_crabAngles.empty()) {
      DoubleVector::iterator iter = m_crabAngles.begin();
      advance(iter, steps);
      m_crabAngles.erase(m_crabAngles.begin(), iter);
    }

    if (!m_crabRates.empty()) {
      DoubleVector::iterator iter = m_crabRates.begin();
      advance(iter, steps);
      m_crabRates.erase(m_crabRates.begin(), iter);
    }

    if (!m_speeds.empty()) {
      DoubleVector::iterator iter = m_speeds.begin();
      advance(iter, steps);
      m_speeds.erase(m_speeds.begin(), iter);
    }

    // append trajectories
    m_curvatures.insert(m_curvatures.begin(), curvatures.rbegin(), curvatures.rend());
    m_curvatureRates.insert(m_curvatureRates.begin(), curvatureRates.rbegin(), curvatureRates.rend());
    m_crabAngles.insert(m_crabAngles.begin(), crabAngles.rbegin(), crabAngles.rend());
    m_crabRates.insert(m_crabRates.begin(), crabRates.rbegin(), crabRates.rend());
    m_speeds.insert(m_speeds.begin(), speeds.rbegin(), speeds.rend());

    return KNMOTOR_OKAY;
  }


  void
  WheelGroupImpl::onCommandCompletion() throw()
  {
    m_curvatures.clear();
    m_curvatureRates.clear();
    m_crabAngles.clear();
    m_crabRates.clear();
    m_speeds.clear();

    // we should end up at the target curvature & crab
    m_wheelGroupSample.curvature = m_wheelGroupSample.targetCurvature;
    m_wheelGroupSample.crabAngle = m_wheelGroupSample.targetCrabAngle ;

    // curvatureRate, crabRate & speed should be 0 by now
    m_wheelGroupSample.curvatureRate = 0;
    m_wheelGroupSample.crabRate = 0;
    m_wheelGroupSample.speed = 0;

    // target vel & rate are also 0
    m_wheelGroupSample.targetSpeed = 0.;
    m_wheelGroupSample.targetCurvatureRate = 0.;
    m_wheelGroupSample.targetCrabRate = 0.;

    m_indexOffset = -1;
    m_frequency = 0.;
  }

  void
  WheelGroupImpl::collectMotorStatus(ACE_Condition_Recursive_Thread_Mutex& condition,
                                     ACE_Time_Value const& timestamp)
  {
    int previousStatus = m_wheelGroupSample.status;

    MotorGroupImpl::collectMotorStatus(condition, timestamp);

    if ((previousStatus == KNMOTOR_BUSY ||
         previousStatus == KNMOTOR_BUSY_INTERRUPTABLE) &&
        m_wheelGroupSample.status <= KNMOTOR_IDLE) {
      onCommandCompletion();
//       cout << "wheel group - move done" << endl
//     << m_wheelGroupSample << endl;
    }

    m_wheelGroupSampleSignal(m_wheelGroupSample);

  }

  void
  WheelGroupImpl::consumeTrajectoryPoints(unsigned int num,
                                          ACE_Time_Value const& now, ACE_Time_Value const& targetTime) throw()
  {
    ACE_Time_Value tT = targetTime;
    unsigned int newSize = 0;
    unsigned int maxSize = max(
                             m_speeds.size(),
                             max(
                               max(m_curvatures.size(), m_curvatureRates.size()),
                               max(m_crabAngles.size(), m_crabRates.size())
                             )
                           );

    if (maxSize == 0) {
      cout << "Skipping consumeTrajectoryPoints.  No points." << endl;
      return;
    }

    if (maxSize >= num) {
      newSize = maxSize - num;
    }
    else {
      // reduce lookahead if not enough points there
      ACE_Time_Value step;
      step.set(1. / m_frequency);
      tT -= (num - maxSize) * step;

      num = maxSize;
    }

    if (num == 0)
      return;

    m_wheelGroupSample.status = KNMOTOR_BUSY_INTERRUPTABLE;
    m_wheelGroupSample.timestamp = now;
    m_wheelGroupSample.targetTime = tT;

    if (m_wheelGroupSample.targetTime == ACE_Time_Value::zero) {
      m_wheelGroupSample.targetTime.set(1.);
      m_wheelGroupSample.targetTime *= 1. / m_frequency;
      m_wheelGroupSample.targetTime *= num;
      m_wheelGroupSample.targetTime += now;
    }

    MotorImpl::consumeSamples(newSize, m_curvatures, m_wheelGroupSample.targetCurvature);
    MotorImpl::consumeSamples(newSize, m_curvatureRates, m_wheelGroupSample.targetCurvatureRate);
    MotorImpl::consumeSamples(newSize, m_crabAngles, m_wheelGroupSample.targetCrabAngle);
    MotorImpl::consumeSamples(newSize, m_crabRates, m_wheelGroupSample.targetCrabRate);
    MotorImpl::consumeSamples(newSize, m_speeds, m_wheelGroupSample.targetSpeed);

    m_indexOffset += num;
  }

  int
  WheelGroupImpl::indexOffset() const throw()
  {
    bool inSync = true;
    MotorVector::const_iterator first, last = m_motors.end();

    for (first = m_motors.begin(); first != last; ++first) {
      if ((*first)->indexOffset() != m_indexOffset) {
        inSync = false;
      }
    }

    if (!inSync) {
      cerr << "index-offset: " << m_indexOffset;

      for (first = m_motors.begin(); first != last; ++first) {
        cerr << " " << (*first)->indexOffset();
      }

      cerr << endl;
    }

    return m_indexOffset;
  }

}
