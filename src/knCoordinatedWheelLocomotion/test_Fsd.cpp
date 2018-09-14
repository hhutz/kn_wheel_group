// -*- c++ -*- ///////////////////////////////////////////////////////////////
//
// This file generates matlab plots to help validate FSD locomotion.
//
// Author: Xavier Bouyssounouse
//
//////////////////////////////////////////////////////////////////////////////

#include "FsdMatlabPlots.h"

int main(int argc, char **argv) 
{
  FsdMatlabPlots plots;
  // wheelAccel, steerAccel, steerRate, time, wheelSpeed, curvature

  double steerRate = .5;
  double steerAccel = 1.0;
  DoubleVector steerPositions(4);
  steerPositions[0] = 0.1;
  steerPositions[1] = 0.2;
  steerPositions[2] = 0.3;
  steerPositions[3] = 0.4;
  plots.initUnalignedSteerPositions(steerRate, steerAccel, steerPositions);
  plots.computeAlignmentPath(5.0);
  plots.outputMatlab("path", "Align0");

  plots.initUnalignedSteerPositions(steerRate, steerAccel, steerPositions);
  plots.computeAlignmentPath(1.0);
  plots.outputMatlab("path", "Align1");

  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m 
  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    2,                          // path time 
    .5,                         // target speed
    0);                         // target curvature
  plots.outputMatlab("path", "0a");

  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m 
  plots.appendPath(
    2,                          // path time 
    0,                         // target speed
    0);                         // target curvature
  plots.outputMatlab("path", "0b");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    10,                          // path time 
    0.5,                        // target speed
    0,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.outputMatlab("path", "C");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    1,                           // path time 
    0.5,                        // target speed
    0,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.appendPath(
    9,                          // path time 
    0.5,                        // target speed
    0,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.outputMatlab("path", "s1C");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    5,                           // path time 
    0.5,                        // target speed
    0,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.appendPath(
    5,                          // path time 
    0.5,                        // target speed
    0,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.outputMatlab("path", "s5C");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    10,                          // path time 
    0.5,                         // target speed
    2,                           // target curvature
    0);                          // target crab = 30 degrees
  plots.outputMatlab("path", "K");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    1,                           // path time 
    0.5,                         // target speed
    2,                           // target curvature
    0);                          // target crab = 30 degrees
  plots.appendPath(
    9,                          // path time 
    0.5,                        // target speed
    2,                          // target curvature
    0);                         // target crab = 30 degrees
  plots.outputMatlab("path", "s1K");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    5,                           // path time 
    0.5,                         // target speed
    2,                           // target curvature
    0);                          // target crab = 30 degrees
  plots.appendPath(
    5,                          // path time 
    0.5,                        // target speed
    2,                          // target curvature
    0);                         // target crab = 30 degrees
  plots.outputMatlab("path", "s5K");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    10,                          // path time 
    0.5,                        // target speed
    2,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.outputMatlab("path", "KC");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    1,                           // path time 
    0.5,                        // target speed
    2,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.appendPath(
    9,                          // path time 
    0.5,                        // target speed
    2,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.outputMatlab("path", "s1KC");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    5,                           // path time 
    0.5,                        // target speed
    2,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.appendPath(
    5,                          // path time 
    0.5,                        // target speed
    2,                          // target curvature
    30);                        // target crab = 30 degrees
  plots.outputMatlab("path", "s5KC");

  //  return 0;
  
  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    1,                          // path time 
    0.5,                        // target speed
    2,                          // target curvature
    30.0);                      // target crab
  plots.appendPath(
    9,
    0.5,
    2,
    30);
  plots.outputMatlab("path", "0a2C");
  
  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    2,                          // path time 
    -.5,                         // target speed
    0);                         // target curvature
  plots.outputMatlab("path", "0an");

  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m 
  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    2,                          // path time 
    0,                         // target speed
    1);                         // target curvature
  plots.outputMatlab("path", "0Rns");

  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m 
  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    2,                          // path time 
    .5,                         // target speed
    1);                         // target curvature
  plots.outputMatlab("path", "0R");

  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m 
  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    2,                          // path time 
    .5,                         // target speed
    -2);                         // target curvature
  plots.outputMatlab("path", "0L");

  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    2,                          // path time 
    .5,                         // target speed
    -2);                         // target curvature
  plots.appendPath(
    2,                          // path time 
    .5,                         // target speed
    2);                         // target curvature
  plots.outputMatlab("path", "0LR");

  plots.initPath(
    0.,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    2,                          // path time 
    0.,                         // target speed
    -2);                         // target curvature
  plots.appendPath(
    2,                          // path time 
    0.,                         // target speed
    2);                         // target curvature
  plots.outputMatlab("path", "0LRnd");


  plots.initPath(
    .5,                         // wheelSpeed
    1.0,                        // wheelAccel
    1.0,                        // steerRate
    1.0                         // steerAccel 
  );
  plots.computePath(
    3,                          // path time 
    .5,                         // target speed
    0);                         // target curvature
  plots.appendPath(
    3,                          // path time 
    .5,                         // target speed
    2);                         // target curvature
  plots.outputMatlab("path", "0g");

  plots.appendPath(
    2,                          // path time 
    0,                         // target speed
    1);                         // target curvature
  plots.outputMatlab("path", "0c");

  plots.appendPath(
    2,                          // path time 
    .5,                         // target speed
    1);                         // target curvature
  plots.outputMatlab("path", "0d");

  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m 
  plots.initPath(
    .5,                         // wheelSpeed (0.5 m/s)
    1.0,                        // wheelAccel (1.0 m/s/s)
    1.0,                        // steerRate  (1.0 rad/s)
    1.0                         // steerAccel (1.0 rad/s/s)
  );
  plots.computePath(
    3,                          // path time 
    0,                         // target speed
    2);                         // target curvature
  plots.outputMatlab("path", "1c");

  plots.appendPath(
    3,                          // path time 
    0,                         // target speed
    0);                         // target curvature
  plots.outputMatlab("path", "1d");

  plots.computePath(
    3,                          // path time 
    0,                         // target speed
    -2);                         // target curvature
  plots.outputMatlab("path", "1e");

  plots.initPath(
    .4,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(7,  -.2,  4);
  plots.appendPath(3,  .2,  0);
  plots.appendPath(20,  .4,  4);
  plots.outputMatlab("path", "1");

  plots.initPath(
    0.,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(7,  0.,  4);
  plots.appendPath(3,   0.,  0);
  plots.appendPath(20,  0.,  4);
  plots.outputMatlab("path", "1nd");

  plots.initPath(
    .4,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(20,  -.2,  8);
  plots.appendPath(10,  .2,  0);
  plots.appendPath(20,  .4,  4);
  plots.outputMatlab("path", "1b");

  // equivalent to locomotor_client: fsd, 1 second, .5 m/s, 1 / m 
  plots.initPath(
    .5,                         // wheelSpeed (0.518 m/s)
    1.152,                        // wheelAccel (1.152 m/s/s)
    0.38 * 180. / M_PI,          // steerRate  (0.38  rad/s)
    3.00 * 180. / M_PI           // steerAccel (3.00  rad/s/s)
  );
  plots.computePath(
    0.6,                          // path time 
    0,                          // target speed
    0.15);                       // target curvature
  plots.outputMatlab("path", "1f");

  plots.initPath(
    .4,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );

  plots.computePath(7,  -.2,  4);
  plots.initPathFromCurrentState();
  plots.appendPath(3,  .2,  0);
  plots.appendPath(20,  .4,  4);
  plots.outputMatlab("path", "2");

  plots.outputMatlabComp("path", "1", "2");

  plots.initPath(
    .4,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(4,  -.2,  4);
  plots.appendPath(3,  .2,  0);
  plots.appendPath(20,  .4,  4);
  plots.outputMatlab("path", "3");

  plots.initPath(
    .4,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(7,  -.2,  4);
  plots.initPathFromPreviousState(-3);
  plots.appendPath(3,  .2,  0);
  plots.appendPath(20,  .4,  4);
  plots.outputMatlab("path", "4");
  
  plots.outputMatlabComp("path", "3", "4");

  // Results in NAN curvature values.
  // Fixed by setting CURVATURE_RATE_LIMIT
  //  plots.initPath();
  //  plots.computePath(.50, 40, 50,  2,  .2,  4);
  //  plots.appendPath( .50, 40, 50,  3,  .2,  0);
  //  plots.appendPath( .50,  5, 50,  30, .2,  4);
  //  plots.outputMatlab("path", "zeroCross");

  // Accelerate without turning
  plots.initPath(
    .2,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(20,  .2,  0);
  plots.outputMatlab("path", "5");

  // Accelerate with single turn
  plots.initPath(
    .2,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(20,  .2,  4);
  plots.outputMatlab("path", "6");

  // Accelerate with single turn
  plots.initPath(
    .2,                         // wheelSpeed
    .25,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(1,  .1,  4);
  plots.appendPath(NAN, 0, NAN);
  plots.outputMatlab("path", "6s");

  // Accelerate without turning                                                                                   
  plots.initPath(
    .4,                         // wheelSpeed
    .02,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(20,  .2,  0);
  plots.appendPath(20,  .4,  0);
  plots.outputMatlab("path", "7");

  // Accelerate twice without turning
  plots.initPath(
    .4,                         // wheelSpeed
    .02,                        // wheelAccel
    50,                         // steerRate
    40                          // steerAccel 
  );
  plots.computePath(20,  -.2,  0);
  plots.appendPath(20,  -.4,  0);
  plots.outputMatlab("path", "8");

  /////////////////////////////////////////////////////////
  // Stop for 5 seconds, turn hard, stop for 5 seconds.
  // Causes Acceleration to exceed due to wheel with fastest steer
  // rate not same wheel as one with fastest steer acceleration.
  /////////////////////////////////////////////////////////

  plots.initPath(
    0.,                         // wheelSpeed
    1.0,                        // wheelAccel
    57.2958,                    // steerRate
    57.2958                     // steerAccel 
  );
  plots.computePath(5,  0,  0);
  plots.appendPath(5,   0,  -2);
  plots.appendPath(5,   0,  0);
  plots.outputMatlab("path", "9a");


  /////////////////////////////////////////////////////////
  // Causes Acceleration to exceed due to wheel with fastest steer
  // rate not same wheel as one with fastest steer acceleration.
  /////////////////////////////////////////////////////////
  plots.initPath(
    0.,                         // wheelSpeed
    1.0,                        // wheelAccel
    57.2958,                    // steerRate
    57.2958                     // steerAccel 
  );
  plots.computePath(5,  0,  0);
  plots.appendPath(5,   0,  2);
  plots.appendPath(5,   0,  -2);
  plots.outputMatlab("path", "9b");


  // If an argument is given, assume it is the directory of plots to compare against.
  if (argc > 1) 
    plots.regressionTest(argv[1]);

  return 0;
}

