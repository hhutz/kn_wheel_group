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
#include "knMotor/MotorFuture.h"

#include <ace/Get_Opt.h>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <iterator>
#include <functional>
#include <algorithm>

#include <cmath>
#include <cassert>

#include <ace/OS_NS_unistd.h>

using namespace std;
using namespace kn;

double frequency = 30.;
vector<string> positionFiles;
vector<string> speedFiles;
vector<vector<double> > positions;
vector<vector<double> > speeds;
vector<double> timeouts;

MotorParameters params;

double maxSpeed = 2. * 360.;
double maxAcc = 360.;
int stopMode = 0;

bool p = false;
bool s = false;

bool verbose = false;

vector<double> loadTrajectory(std::string const& f) 
{
  fstream istr(f.c_str());
  vector<double> t;
  
  //  cerr << f << endl;
  copy(istream_iterator<double>(istr), istream_iterator<double>(), back_inserter(t));
  transform(t.begin(), t.end(), t.begin(), bind2nd(multiplies<double>(), M_PI/180.));

  return t;
}

int parseArgs(int& argc, char* argv[])
{
  int rc = 0;
  int c;
  
  // initialize parameters from command line
  ACE_Get_Opt get_opts (argc, argv, "PSF:p:s:a:t:m:v?");
  
  while ((c = get_opts()) != -1) {
    switch (c) {
    case 'P':
      p = true;
      break;
    case 'S':
      s = true;
      break;
    case 'F':
      frequency = atof(get_opts.optarg);
      break;
    case 'p':
      positionFiles.push_back(get_opts.optarg);
      break;
    case 's':
      speedFiles.push_back(get_opts.optarg);
      break;
    case 'a':
      maxAcc = atof(get_opts.optarg);
      break;
    case 't':
      timeouts.push_back(atof(get_opts.optarg));
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
           << "  -p position file - file holding position samples (multiple files can be given)" << endl
           << "  -s speed file - file holding speed samples (multiple files can be given)" << endl
           << "  -a acc - max acceleration (default: 1 * 360deg/(s*s))" << endl
           << "  -F hz - sim frequency (default: 30hz)" << endl
           << "  -t timeout - time before stop command (default: 0s - disabled) (multiple stops)" << endl
           << "  -m mode - stopping mode  (default: 0)" << endl
           << "    [0 = emergency, 1 = off, 2 = abruptly, 3 = smoothly," << endl
           << "    [4 = off, breaks, 5 = abruptly, breaks, 6 = smoothly, breaks]" << endl
           << "  -v verbose mode" << endl
           << "  -? help: emit this text and stop" << endl;
      rc = 1;
    }
  }
  
  if (verbose) {
    cerr << "Test parameters:" << endl
         << "freq: " << frequency << "hz" << endl
         << "pos files: " << positionFiles.size() << endl
         << "speed files: " << speedFiles.size() << endl
         << "acc: " << maxAcc << "deg/(s*s)" << endl
         << "timeouts: " << timeouts.size() << endl
         << "mode: " << stopMode << endl;
  }
  
  params.limits.maxVelocity = maxSpeed * M_PI/180.;
  params.limits.maxAcceleration = maxAcc * M_PI/180.;  

  {
    vector<string>::const_iterator first, last = positionFiles.end();
    for (first = positionFiles.begin(); first != last; ++first) {
      positions.push_back(loadTrajectory(*first));
    }
  }

  {
    vector<string>::const_iterator first, last = speedFiles.end();
    for (first = speedFiles.begin(); first != last; ++first) {
      speeds.push_back(loadTrajectory(*first));
    }
  }

  assert (speeds.empty() ||
          speeds.size() == positions.size());
  
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

  int iterations = 0;

  ACE_Time_Value startTime = ACE_OS::gettimeofday();
  vector<vector<double> >::const_iterator first, last = positions.end();
  first = positions.begin();
  vector<vector<double> >::const_iterator speed = speeds.begin();
  vector<double>::const_iterator t = timeouts.begin();
  
  
  cout << fixed;
  ACE_Time_Value slice = ACE_Time_Value(1);
  ACE_Time_Value now = startTime;
  slice *= 1./frequency;
  while (first != last) {
    motor.setTime(now);
    bool stopped = false;
    
    cerr << "pos:" << first->size() << endl;
    cerr << "spe:" << speed->size() << endl;
    // send next command
    motor.cmdTrajectory(-1,
                        frequency,
                        *first++,
                        (speed != speeds.end())? *speed++ : vector<double>());

    cerr << "loop" << endl;
    // execute command
    while (motor.cmdStatus() != KNMOTOR_IDLE) {
      ACE_OS::sleep(ACE_Time_Value(0, 1000)); // slow down execution a bit
      motor.setTime(now);
      motor.executeTimeSlice(frequency);
      ++iterations;
      
      // print trajectory
      MotorSample sample = motor.motorState();

      if (verbose)
        cerr << sample << endl;

      if (p || s) {
        cout.precision(5);
        cout.fill(' ');
	
        if (s) {
          cout.width(12);
          cout << sample.state.speed * 180./M_PI;
        }
        if (p && s)
          cout << "  ";
        if (p) {
          cout.width(12);
          cout << sample.state.position * 180./M_PI;
        }
        cout << endl;
      }

      // iterrupt command
      ACE_Time_Value timeout = ACE_Time_Value::zero;
      if (t != timeouts.end()) {
        timeout.set(*t);
        timeout += startTime;
      }
      
      if (timeout != ACE_Time_Value::zero &&
          (sample.targetTime >= timeout) &&
          !stopped) {

        if (first == last) {
          //          cerr << "trajectories done. stopping" << endl;

          SimMotor::StopMode m = intToStopMode[stopMode];
          motor.stop(m);
          stopped = true;
        }
        else {
          cerr << "set next trajectories" << endl;
          MotorFuture f;
          int ref = motor.futureMotorState(f, ACE_Time_Value::zero);
          if (ref == -2) {
            cerr << "interrupt time too far in the future" << endl;
            return 1;
          }
          int rc = motor.cmdTrajectory(ref,
                                       frequency,
                                       *first++,
                                       (speed != speeds.end())? *speed++ : vector<double>());
          if (rc != 0) {
            cerr << "failed to set trajectory: " << rc << endl
                 << "ref: " << ref << endl;
            return 1;
          }
        }
        ++t;
      }  

      now += slice;  
    }
  }
  
  if (verbose)
    cerr << iterations << endl;
  return 0;
}
