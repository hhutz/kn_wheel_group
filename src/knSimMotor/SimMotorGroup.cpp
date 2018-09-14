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

#include "SimMotorGroup.h"
#include "SimMotor.h"

#include "knMotor/MotorGroupSample.h"

#include <cmath>

namespace kn
{
  using namespace std;

  SimMotorGroup::SimMotorGroup(MotorVector const& motors,
                               MotorGroupParameters const * params) :
    SimMotorGroupBase(motors, params)
  {
  }

  SimMotorGroup::~SimMotorGroup() throw()
  {
  }

  int
  SimMotorGroup::stop(StopMode mode)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    if (m_sample->status < KNMOTOR_IDLE)
      return m_sample->status;

    setStateStop(mode, m_now);
    m_condition.broadcast();

    return KNMOTOR_OKAY;
  }

  int
  SimMotorGroup::cmdProfiles(std::vector<MotorProfile> const& profiles)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    int rc = setStateCmdProfiles(profiles, m_now);

    if (rc == KNMOTOR_OKAY)
      m_condition.broadcast();

    return rc;
  }

  int
  SimMotorGroup::cmdTrajectories(int ref,
                                 double frequency,
                                 TrajectoryVector const& positions,
                                 TrajectoryVector const& speeds)
  {
    ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

    int status = m_sample->status;
    int rc = setStateCmdTrajectories(ref,
                                     frequency,
                                     positions,
                                     speeds);

    if (rc == KNMOTOR_OKAY) {
      if (status == KNMOTOR_IDLE) {
        // if we just started
        startTrajectories(frequency);
      }

      m_condition.broadcast();
    }

    return rc;
  }

}
