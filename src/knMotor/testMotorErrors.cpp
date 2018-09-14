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

#include <iostream>

using namespace std;
using namespace kn;

int errors[] = {
  KNMOTOR_OKAY,
  KNMOTOR_ERR_OTHER,
  KNMOTOR_ERR_NUM_MOTORS,
  KNMOTOR_ERR_PARAM_RANGE,
  KNMOTOR_ERR_CALIBRATION,
  KNMOTOR_ERR_TIMEOUT,
  KNMOTOR_ERR_TRAJ_CMD,
  KNMOTOR_ERR_HW_ESTOP,
  KNMOTOR_ERR_TRAJ_EXEC,
  KNMOTOR_ERR_POS_ERROR,
  KNMOTOR_ERR_OVERCURRENT,
  KNMOTOR_ERR_HW_COM,
  KNMOTOR_ERR_NOT_SUPPORTED
};

int numErrors = sizeof(errors) / sizeof(int);

int main(int argc, char * argv[])
{
  unsigned int error;

  cout << "error value (uint hex): " << flush;
  cin >> hex >> error;

  cout << "0x" << hex << error << endl;

  for (int i = 0; i < numErrors; ++i) {
    cout << "is " << MotorBase::getErrorName(errors[i]) << ": " << MotorBase::isError(errors[i], error) << endl;
  }

  cout << "it is: " << MotorBase::getErrorName(error) << endl;

  return 0;
}
