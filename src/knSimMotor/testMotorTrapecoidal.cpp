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

#include <ace/Get_Opt.h>

#include <iostream>
#include <cmath>

using namespace std;
using namespace kn;

double frequency = 30.;
MotorParameters params;

double targetPosition = 10. * 360.;
double maxSpeed = 2. * 360.;
double maxAcc = 360.;

double timeout = 0.;
int stopMode = 0;

bool p = false;
bool s = false;

bool verbose = false;

int parseArgs(int& argc, char* argv[])
{
  int rc = 0;
  int c;
  
  // initialize parameters from command line
  ACE_Get_Opt get_opts (argc, argv, "PSf:p:s:a:t:m:v?");
  
  while ((c = get_opts()) != -1) {
    switch (c) {
    case 'P':
      p = true;
      break;
    case 'S':
      s = true;
      break;
    case 'f':
      frequency = atof(get_opts.optarg);
      break;
    case 'p':
      targetPosition = atof(get_opts.optarg);
      break;
    case 's':
      maxSpeed = atof(get_opts.optarg);
      break;
    case 'a':
      maxAcc = atof(get_opts.optarg);
      break;
    case 't':
      timeout = atof(get_opts.optarg);
      break;
    case 'm':
      stopMode = atoi(get_opts.optarg);
      if (stopMode < 0 ||
          stopMode > 6)
        stopMode = 0;
      break;
    case 'v':
      verbose = true;
      break;
    case '?':
    default:
      cerr << "usage: " << argv[0] << "[-PS] [-p pos] [-s speed] [-a acc] [-f hz] [-t timeout] [-m stopMode] [-v?]" << endl
           << "  -P - print position (default: false)" << endl
           << "  -S - print speed (default: false)" << endl
           << "  -p pos - target position (default: 10 * 360deg)" << endl
           << "  -s speed - max speed (default: 2 * 360deg/s)" << endl
           << "  -a acc - max acceleration (default: 1 * 360deg/(s*s))" << endl
           << "  -f hz - sim frequency (default: 30hz)" << endl
           << "  -t timeout - time before stop command (default: 0s - disabled)" << endl
           << "  -m mode - stopping mode  (default: 0)" << endl
           << "    [0 = emergency, 1 = off, 2 = abruptly, 3 = smoothly"  << endl
           << "  -v verbose mode" << endl
           << "  -? help: emit this text and stop" << endl;
      rc = 1;
    }
  }
  
  if (verbose) {
    cerr << "Test parameters:" << endl
         << "freq: " << frequency << "hz" << endl
         << "pos: " << targetPosition << "deg" << endl
         << "speed: " << maxSpeed << "deg/s"<< endl
         << "acc: " << maxAcc << "deg/(s*s)" << endl
         << "timeout: " << timeout << "s" << endl
         << "mode: " << stopMode << endl;
  }
  
  params.limits.maxVelocity = maxSpeed * M_PI/180.;
  params.limits.maxAcceleration = maxAcc * M_PI/180.;  
  
  return rc;
}

SimMotor::StopMode intToStopMode[] = {
  SimMotor::STOP_EMERGENCY,
  SimMotor::STOP_OFF,
  SimMotor::STOP_ABRUPTLY, 
  SimMotor::STOP_SMOOTHLY
};

int main(int argc, char * argv[])
{
  if (parseArgs(argc, argv) != 0)
    return 1;

  SimMotor motor(&params);

  MotorProfile profile(MotorProfile::CTRL_TRAPECOID,
                       MotorProfile::POS_ABS,
                       targetPosition * M_PI/180.,
                       maxSpeed * M_PI/180.,
                       maxAcc * M_PI/180.);
  motor.cmdProfile(profile);

  int iterations = 0;
  bool stopped = false;

  cout << fixed;
  ACE_Time_Value now = ACE_OS::gettimeofday();
  ACE_Time_Value slice = ACE_Time_Value(1);
  slice *= 1./frequency;
  while (motor.cmdStatus() != KNMOTOR_IDLE) {
    //    ACE_OS::sleep(ACE_Time_Value(0, 100000));

    motor.setTime(now);
    motor.executeTimeSlice(frequency);
    ++iterations;

    if (timeout > 0.00001 &&
        iterations > (timeout * frequency) &&
        !stopped) {
      SimMotor::StopMode m = intToStopMode[stopMode];
      motor.stop(m);
      stopped = true;
    }

    MotorSample sample = motor.motorState();
    if (p || s) {
      cout.precision(5);
      cout.fill(' ');

      if (s) {
        cout.width(12);
        cout << sample.state.speed * 180/M_PI;
      }
      if (p && s)
        cout << "  ";
      if (p) {
        cout.width(12);
        cout << sample.state.position * 180/M_PI;
      }
      cout << endl;
    }

    now += slice;
  }
  
  if (verbose)
    cerr << iterations << endl;
  return 0;
}
