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
 * Module: FSD Locomotion
 * Author: Xavier Bouyssounouse
 *
 *****************************************************************************/

// Tests the TrapezoidProfile and HalfTrapProfile classes with a few
// hand coded trapezoid test values.

#include <iostream>
#include <cmath>
#include <cassert>
#include "TrapezoidProfile.h"
#include "HalfTrapProfile.h"

using namespace std;

int main(int argc, char **argv) 
{
  // Only differences should be very small roundoff errors due to size of double.
  const double EPSILON = 1E-12;

  double maxSpeed = 10.;
  double maxAccel = 2.;
  kn::TrapezoidProfile trapezoid(maxSpeed, maxAccel);
  kn::HalfTrapProfile  halfTrapz(maxSpeed, maxAccel);
  double position, speed;
  double initialPosition, initialSpeed, targetPosition;

  ///////////////////////////////////////////////////////////////////////////////////
  // Trapezoid with no overshoot, positive initial speed, 
  // positive initial position, positive target
  ///////////////////////////////////////////////////////////////////////////////////
  initialPosition = 1.0;
  initialSpeed = 4.0;
  targetPosition = 100.0;
  trapezoid.initializeState(initialPosition, initialSpeed, targetPosition);
  halfTrapz.initializeState(initialPosition, initialSpeed);
  cout << trapezoid << endl;
  cout << halfTrapz << endl;
  assert(fabs(targetPosition - trapezoid.derivedTargetPosition()) < EPSILON);
  assert(fabs(trapezoid.trajectoryTime() - 13.3) < EPSILON);

  // climb test
  trapezoid.computeState(2.0, position, speed);
  assert(fabs(speed - 8.0) < EPSILON);          
  assert(fabs(position - 13.0) < EPSILON);
  halfTrapz.computeState(2.0, position, speed);
  assert(fabs(speed - 8.0) < EPSILON);          
  assert(fabs(position - 13.0) < EPSILON);

  // cruise test
  trapezoid.computeState(5.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 42.0) < EPSILON);
  halfTrapz.computeState(5.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 42.0) < EPSILON);

  // trapezoid descend test
  trapezoid.computeState(10.3, position, speed);
  assert(fabs(speed - 6.0) < EPSILON); 
  assert(fabs(position - 91.0) < EPSILON);
  // half trap  cruise test
  halfTrapz.computeState(10.3, position, speed);
  assert(fabs(speed - 10.0) < EPSILON); 
  assert(fabs(position - 95.0) < EPSILON);
  ///////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////////////////
  // Trapezoid overshoot, positive initial speed, 
  // positive initial position, negative target.
  ///////////////////////////////////////////////////////////////////////////////////
  initialPosition = 12.0;
  initialSpeed = 4.0;
  targetPosition = -100.0;
  trapezoid.initializeState(initialPosition, initialSpeed, targetPosition);
  halfTrapz.initializeState(initialPosition, initialSpeed);
  cout << trapezoid << endl;
  cout << halfTrapz << endl;
  assert(fabs(targetPosition - trapezoid.derivedTargetPosition()) < EPSILON);
  assert(fabs(trapezoid.trajectoryTime() - 18.6) < EPSILON);

  // descend test
  trapezoid.computeState(1.0, position, speed);
  assert(fabs(speed - 2.0) < EPSILON);          
  assert(fabs(position - 15.0) < EPSILON);
  halfTrapz.computeState(1.0, position, speed);
  //half trap ascend test
  assert(fabs(speed - 6.0) < EPSILON);          
  assert(fabs(position - 17.0) < EPSILON);

  // cruise test
  trapezoid.computeState(9.0, position, speed);
  assert(fabs(speed + 10.0) < EPSILON);
  assert(fabs(position + 29) < EPSILON);
  halfTrapz.computeState(9.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 93) < EPSILON);

  // trapezoid climb test
  trapezoid.computeState(15.6, position, speed);
  assert(fabs(speed + 6.0) < EPSILON); 
  assert(fabs(position + 91.0) < EPSILON);
  // half trap cruise test
  halfTrapz.computeState(15.6, position, speed);
  assert(fabs(speed - 10.0) < EPSILON); 
  assert(fabs(position - 159.0) < EPSILON);
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  // Trapezoid overshoot, target speed not reached, positive initial speed, 
  // positive initial position, negative target.
  ///////////////////////////////////////////////////////////////////////////////////
  initialPosition = 12.0;
  initialSpeed = 4.0;
  targetPosition = -16.0;
  trapezoid.initializeState(initialPosition, initialSpeed, targetPosition);
  halfTrapz.initializeState(initialPosition, initialSpeed);
  cout << trapezoid << endl;
  cout << halfTrapz << endl;
  assert(fabs(targetPosition - trapezoid.derivedTargetPosition()) < EPSILON);
  assert(fabs(trapezoid.trajectoryTime() - 10.0) < EPSILON);

  // descend test
  trapezoid.computeState(1.0, position, speed);
  assert(fabs(speed - 2.0) < EPSILON);          
  assert(fabs(position - 15.0) < EPSILON);
  halfTrapz.computeState(1.0, position, speed);
  assert(fabs(speed - 6.0) < EPSILON);          
  assert(fabs(position - 17.0) < EPSILON);

  // descend test (speed changes sign)
  trapezoid.computeState(4.0, position, speed);
  assert(fabs(speed + 4.0) < EPSILON);
  assert(fabs(position - 12.0) < EPSILON);
  halfTrapz.computeState(4.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 43.0) < EPSILON);

  // trapezoid climb test
  trapezoid.computeState(9.0, position, speed);
  assert(fabs(speed + 2.0) < EPSILON); 
  assert(fabs(position + 15.0) < EPSILON);
  // half trap cruise test
  halfTrapz.computeState(9.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON); 
  assert(fabs(position - 93) < EPSILON);
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  // Trapezoid overshoot, target speed not reached, positive initial
  // speed, positive initial position, negative target.  Initial speed
  // over max speed.  Trapezoid should decelerate immediately, and
  // reach negative cruise speed at max speed, before reducing speed
  // to target.
  ///////////////////////////////////////////////////////////////////////////////////
  initialPosition = 12.0;
  initialSpeed = 11.0;
  targetPosition = -82.75;
  trapezoid.initializeState(initialPosition, initialSpeed, targetPosition);
  halfTrapz.initializeState(initialPosition, initialSpeed);
  cout << trapezoid << endl;
  cout << halfTrapz << endl;
  assert(fabs(targetPosition - trapezoid.derivedTargetPosition()) < EPSILON);
  assert(fabs(trapezoid.trajectoryTime() - 23.0) < EPSILON);

  // descend test
  trapezoid.computeState(0.25, position, speed);
  assert(fabs(speed - 10.5) < EPSILON);          
  assert(fabs(position - 14.6875) < EPSILON);
  halfTrapz.computeState(0.25, position, speed);
  assert(fabs(speed - 10.5) < EPSILON);          
  assert(fabs(position - 14.6875) < EPSILON);

  // descend test (speed changes sign)
  trapezoid.computeState(9.0, position, speed);
  assert(fabs(speed + 7.0) < EPSILON);
  assert(fabs(position - 30.0) < EPSILON);
  halfTrapz.computeState(9.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 102.25) < EPSILON);

  // cruise test
  trapezoid.computeState(15.0, position, speed);
  assert(fabs(speed + 10.0) < EPSILON);
  assert(fabs(position + 27.75) < EPSILON);
  halfTrapz.computeState(15.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 162.25) < EPSILON);

  // trapezoid climb test
  trapezoid.computeState(20.0, position, speed);
  assert(fabs(speed + 6.0) < EPSILON); 
  assert(fabs(position + 73.75) < EPSILON);
  // half trap cruise test
  halfTrapz.computeState(20.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON); 
  assert(fabs(position - 212.25) < EPSILON);
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  // Trapezoid with no overshoot, positive initial speed, 
  // positive initial position, positive target.  Initial speed
  // higher than target.
  ///////////////////////////////////////////////////////////////////////////////////
  initialPosition = 1.0;
  initialSpeed = 11.0;
  targetPosition = 100.0;
  trapezoid.initializeState(initialPosition, initialSpeed, targetPosition);
  halfTrapz.initializeState(initialPosition, initialSpeed);
  cout << trapezoid << endl;
  cout << halfTrapz << endl;
  assert(fabs(targetPosition - trapezoid.derivedTargetPosition()) < EPSILON);
  assert(fabs(trapezoid.trajectoryTime() - 12.375) < EPSILON);

  // descend test
  double time = 0.10;
  trapezoid.computeState(time, position, speed);
  assert(fabs(speed - 10.8) < EPSILON);          
  assert(fabs(position - 2.09) < EPSILON);
  halfTrapz.computeState(time, position, speed);
  assert(fabs(speed - 10.8) < EPSILON);          
  assert(fabs(position - 2.09) < EPSILON);

  // cruise test
  trapezoid.computeState(5.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 51.25) < EPSILON);
  halfTrapz.computeState(5.0, position, speed);
  assert(fabs(speed - 10.0) < EPSILON);
  assert(fabs(position - 51.25) < EPSILON);

  // trapezoid descend test
  trapezoid.computeState(10.375, position, speed);
  assert(fabs(speed - 4.0) < EPSILON); 
  assert(fabs(position - 96.0) < EPSILON);
  // half trap cruise test
  halfTrapz.computeState(10.375, position, speed);
  assert(fabs(speed - 10.0) < EPSILON); 
  assert(fabs(position - 105.0) < EPSILON);
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  // Overspeed on negative speed.
  ///////////////////////////////////////////////////////////////////////////////////
  initialPosition = -1.0;
  initialSpeed = -11.0;
  targetPosition = -100.0;
  trapezoid.initializeState(initialPosition, initialSpeed, targetPosition);
  halfTrapz.initializeState(initialPosition, initialSpeed);
  cout << trapezoid << endl;
  cout << halfTrapz << endl;
  assert(fabs(targetPosition - trapezoid.derivedTargetPosition()) < EPSILON);
  assert(fabs(trapezoid.trajectoryTime() - 12.375) < EPSILON);

  // climb test
  trapezoid.computeState(0.10, position, speed);
  assert(fabs(speed + 10.8) < EPSILON);          
  assert(fabs(position + 2.09) < EPSILON);
  halfTrapz.computeState(0.10, position, speed);
  assert(fabs(speed + 10.8) < EPSILON);          
  assert(fabs(position + 2.09) < EPSILON);

  // cruise test
  trapezoid.computeState(5.0, position, speed);
  assert(fabs(speed + 10.0) < EPSILON);
  assert(fabs(position + 51.25) < EPSILON);
  halfTrapz.computeState(5.0, position, speed);
  assert(fabs(speed + 1.0) < EPSILON);
  assert(fabs(position + 31.0) < EPSILON);

  // trapezoid climb test
  trapezoid.computeState(10.375, position, speed);
  assert(fabs(speed + 4.0) < EPSILON); 
  assert(fabs(position + 96.0) < EPSILON);
  // half trap cruise test
  halfTrapz.computeState(10.0, position, speed);
  assert(fabs(speed - 9.0) < EPSILON); 
  assert(fabs(position + 11.0) < EPSILON);
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  // Verify cruiseSpdSgn uses targetDist instead of targetPosition has been fixed.
  ///////////////////////////////////////////////////////////////////////////////////
  initialPosition = -1.3;
  initialSpeed = 0.0;
  targetPosition = 0.0;
  maxSpeed = 1.0;
  maxAccel = 1.0;
  trapezoid.setLimits(maxSpeed,maxAccel);
  trapezoid.initializeState(initialPosition, initialSpeed, targetPosition);
  halfTrapz.setLimits(maxSpeed,maxAccel);
  halfTrapz.initializeState(initialPosition, initialSpeed);
  cout << trapezoid << endl;
  cout << halfTrapz << endl;
  assert(fabs(targetPosition - trapezoid.derivedTargetPosition()) < EPSILON);
  assert(fabs(trapezoid.trajectoryTime() - 2.3) < EPSILON);

  // climb test
  trapezoid.computeState(0.25, position, speed);
  assert(fabs(speed - .25) < EPSILON);          
  assert(fabs(position + 1.26875) < EPSILON);
  halfTrapz.computeState(0.25, position, speed);
  assert(fabs(speed - .25) < EPSILON);          
  assert(fabs(position + 1.26875) < EPSILON);

  // cruise test
  trapezoid.computeState(1.1, position, speed);
  assert(fabs(speed - 1.0) < EPSILON);
  assert(fabs(position + 0.7) < EPSILON);
  halfTrapz.computeState(1.1, position, speed);
  assert(fabs(speed - 1.0) < EPSILON);
  assert(fabs(position + 0.7) < EPSILON);

  // trapezoid descend test
  trapezoid.computeState(1.4, position, speed);
  assert(fabs(speed - 0.9) < EPSILON); 
  assert(fabs(position + 0.405) < EPSILON);
  // half trap cruise test
  halfTrapz.computeState(1.4, position, speed);
  assert(fabs(speed - 1.0) < EPSILON); 
  assert(fabs(position + 0.4) < EPSILON);
  ///////////////////////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////////////////////
  // Bug #001 found using TouchPad interface -- verify it has been fixed
  ///////////////////////////////////////////////////////////////////////////////////
  trapezoid.setLimits(1,4);
  trapezoid.initializeState(0.228277,0.669917,0.427001);
  assert(fabs(0.427001 - trapezoid.derivedTargetPosition()) < EPSILON);
  cout << trapezoid;

  ///////////////////////////////////////////////////////////////////////////////////
  // Bug #002 found using TouchPad interface -- verify it has been fixed
  ///////////////////////////////////////////////////////////////////////////////////
  trapezoid.setLimits(1,4);
  trapezoid.initializeState(-.0998555, 0.893781, 0.);
  assert(fabs(0 - trapezoid.derivedTargetPosition()) < EPSILON);
  cout << trapezoid;

  

  // If we get here, none of the asserts failed.
  std::cout << "\nNo errors detected in TrapezoidProfile class.\n\n";

  return 0;
}

