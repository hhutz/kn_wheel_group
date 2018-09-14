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

#include "WheelGroup.h"
#include "MotorInterface.h"

#include <iostream>
#include <limits>
#include <cmath>

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

namespace kn
{
  using namespace std;

  namespace protoMotor
  {

    WheelGroup::WheelGroup(MotorInterface * interface,
                           MotorVector const& motors,
                           WheelGroupParameters const * params,
                           WheelParametersVector const& wheelParams) :
      kn::MotorGroupImpl(motors, params),
      kn::WheelGroupImpl(motors, params, wheelParams),
      m_interface(interface)
    {
      // as we don't know the initial wheel configuration
      // we need to assume that the stuff is not aligned

      m_wheelGroupSample.curvature = NAN;
      m_wheelGroupSample.curvatureRate = 0.;
      m_wheelGroupSample.speed = 0.;
      m_wheelGroupSample.crabAngle = 0.;
      m_wheelGroupSample.crabRate = 0.;
    }

    WheelGroup::~WheelGroup() throw()
    {
      cout << "WheelGroup::~WheelGroup()" << endl;
      MotorVector::const_iterator first, last = m_motors.end();

      for (first = m_motors.begin(); first != last; ++first) {
        cout << "deleting motor" << endl;
        delete *first;
      }
    }

    int
    WheelGroup::cmdStatus(bool waitForCompletion, ACE_Time_Value const * timeout)
    {
      return cmdStatusSkel(m_interface->m_condition, waitForCompletion, timeout);
    }

    int
    WheelGroup::calibrate(CalibrationMode mode)
    {
      // we don't do calibration, yet
      return (m_sample->status == KNMOTOR_IDLE) ? KNMOTOR_OKAY : KNMOTOR_BUSY;
    }

    int
    WheelGroup::tryRecover()
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_interface->m_mutex);
      return m_interface->tryRecover() ? KNMOTOR_OKAY : KNMOTOR_ERR_HW_COM;
    }

    int
    WheelGroup::stop(StopMode mode)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_interface->m_mutex);

      if (m_wheelGroupSample.status > KNMOTOR_IDLE) {
        setStateStop(mode, ACE_OS::gettimeofday());
        m_interface->m_condition.broadcast();
      }

      return m_interface->stop(mode) ? KNMOTOR_OKAY : KNMOTOR_ERR_HW_COM;
    }

    MotorGroupSample
    WheelGroup::motorGroupState(bool waitNextSample, ACE_Time_Value const * timeout)
    {
      //cout << "WheelGroup::motorGroupState" << endl;
      return motorGroupStateSkel(m_interface->m_condition, waitNextSample, timeout);
    }

    int
    WheelGroup::futureMotorGroupState(MotorGroupFuture& state, const ACE_Time_Value& targetTime)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_interface->m_mutex);

      return collectFutureMotorGroupState(state, targetTime);
    }

    WheelGroupSample
    WheelGroup::wheelGroupState(bool waitNextSample, ACE_Time_Value const * timeout)
    {
      return wheelGroupStateSkel(m_interface->m_condition, waitNextSample, timeout);
    }

    int
    WheelGroup::futureWheelGroupState(kn::WheelGroupFuture& state, ACE_Time_Value const& targetTime)
    {
      // to maximize the time that this index is valid,
      // we base this on the latest telemetry update
      ACE_Time_Value timeout = targetTime;

      if (timeout == ACE_Time_Value::zero)
        timeout = ACE_OS::gettimeofday() + ACE_Time_Value(0, 100000);

      MotorGroupSample sample = motorGroupState(true, &timeout);

      if (isError(KNMOTOR_ERR_TIMEOUT, sample.status)) {
        cout << "WheelGroup::futureWheelGroupState motorGroupState error." << endl;
        return KNMOTOR_ERR_TIMEOUT;
      }

      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_interface->m_mutex);

      return collectFutureWheelGroupState(state, timeout);
    }

    int
    WheelGroup::cmdProfiles(std::vector<MotorProfile> const& profiles,
                            double curvature, double crabAngle)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_interface->m_mutex);

      // make that absolute profiles!
      MotorProfileVector p = profiles;

      for (unsigned i = 0; i < p.size(); ++i) {
        if (p[i].posMode == MotorProfile::POS_REL) {
          p[i].position += m_wheelGroupSample.motors[i].position;
          p[i].posMode = MotorProfile::POS_ABS;
        }
      }

      int rc = setStateCmdProfiles(p,
                                   curvature, crabAngle,
                                   ACE_OS::gettimeofday());

      if (rc != KNMOTOR_OKAY) {
        cerr << "cmdProfiles setState errors: " << getErrorName(rc) << endl;
        cerr << "state:" << endl
             << m_wheelGroupSample << endl;
        return rc;
      }

      m_interface->m_condition.broadcast();

      return  m_interface->profile(p) ? KNMOTOR_OKAY : KNMOTOR_ERR_HW_COM;
    }

    int
    WheelGroup::cmdTrajectories(int ref,
                                double frequency,
                                TrajectoryVector const& mPositions,
                                TrajectoryVector const& mSpeeds,
                                DoubleVector const& curvatures,
                                DoubleVector const& curvatureRates,
                                DoubleVector const& crabAngles,
                                DoubleVector const& crabRates,
                                DoubleVector const& speeds)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_interface->m_mutex);
      int rc = setStateCmdTrajectories(ref,
                                       frequency,
                                       mPositions,
                                       mSpeeds,
                                       curvatures,
                                       curvatureRates,
                                       crabAngles,
                                       crabRates,
                                       speeds);

      if (rc != KNMOTOR_OKAY)
        return rc;

      MotorInterface::TrajectoryStepVector trajStepVector;
      //double period = (1.0/frequency);
      DoubleVector periods;
      int len = mPositions[0].size();

      trajStepVector.resize(len);

      for (int i = 0; i < len; ++i) {
        trajStepVector[i].leftFrontDrive  = mPositions[0].at(i) * RAD2DEG;
        trajStepVector[i].rightFrontDrive = mPositions[1].at(i) * RAD2DEG;
        trajStepVector[i].leftRearDrive   = mPositions[2].at(i) * RAD2DEG;
        trajStepVector[i].rightRearDrive  = mPositions[3].at(i) * RAD2DEG;
        trajStepVector[i].leftFrontSteer  = mPositions[4].at(i) * RAD2DEG;
        trajStepVector[i].rightFrontSteer = mPositions[5].at(i) * RAD2DEG;
        trajStepVector[i].leftRearSteer   = mPositions[6].at(i) * RAD2DEG;
        trajStepVector[i].rightRearSteer  = mPositions[7].at(i) * RAD2DEG;

        periods.push_back(1.0 / frequency);
      }

      m_interface->m_condition.broadcast();


      return m_interface->trajectory(len, periods, trajStepVector, ref + 1) ? KNMOTOR_OKAY : KNMOTOR_ERR_HW_COM;
      //return m_interface->trajectory(len, frequency, trajStepVector, ref)? KNMOTOR_OKAY : KNMOTOR_ERR_HW_COM;
    }

  }
}
