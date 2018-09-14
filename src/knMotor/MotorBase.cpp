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
 * Module: knMotor
 * Author: Hans Utz
 *
 *****************************************************************************/

#include "MotorBase.h"

#include <algorithm>

namespace
{
  char const * const errorNames[] = {
    "KNMOTOR_NO_ERROR",
    "KNMOTOR_ERR_UNSPECIFIED",
    "KNMOTOR_ERR_OTHER",
    "KNMOTOR_ERR_NUM_MOTORS",
    "KNMOTOR_ERR_PARAM_RANGE",
    "KNMOTOR_ERR_CALIBRATION",
    "KNMOTOR_ERR_TIMEOUT",
    "KNMOTOR_ERR_TRAJ_CMD",
    "KNMOTOR_ERR_HW_ESTOP",
    "KNMOTOR_ERR_SW_ESTOP",
    "KNMOTOR_ERR_TRAJ_EXEC",
    "KNMOTOR_ERR_POS_ERROR",
    "KNMOTOR_ERR_OVERCURRENT",
    "KNMOTOR_ERR_POWER_FAULT",
    "KNMOTOR_ERR_HW_COMM",
    "KNMOTOR_ERR_NOT_SUPPORTED",
    "KNMOTOR_ERR_UNKNOWN"
  };

  int const NUM_ERRORS = sizeof(errorNames) / sizeof(char const * const);
}

namespace kn
{
  using namespace std;

  char const * MotorBase::getErrorName(int error) throw()
  {
    int index = 0;

    // positive status is no error
    if (error >= 0) {
      index = 0;
    }
    // check for highest set error flag
    else {
      // kill negative sign
      error &= INT_MAX;

      // check bits
      for (index = 1; index < NUM_ERRORS; ++index) {
        if (error == 0)
          break;

        error >>= 1;
      }

      index = min(index, NUM_ERRORS - 1);
    }

    return errorNames[index];
  }

  bool
  MotorBase::isError(int error, int status) throw()
  {
    return
      (status >= 0) ?
      (error == KNMOTOR_OKAY) : ((error & status & INT_MAX) != 0);
  }

  int
  MotorBase::clearError(int error, int status) throw()
  {
    if (status < 0 && error < 0) {
      status &= ~(error & INT_MAX);
    }

    return status;
  }

  MotorBase::~MotorBase() throw()
  {
  }
}
