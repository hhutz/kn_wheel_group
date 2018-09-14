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

#include "FsdMatlabPlots.h"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>

#include <boost/progress.hpp>

using namespace std;
using namespace kn;

namespace
{
  bool const printFsdDebug = false;
  bool const printFsdWarnings = false;
}

const string FsdMatlabPlots::s_wheelNames[]  = {"Left Front", "Right Front", "Left Rear", "Right Rear"};
const string FsdMatlabPlots::s_wheelOper[]  = {"Drive Speed", "Drive Accel", "Steer Speed", "Steer Accel"};
const string FsdMatlabPlots::s_wheelUnit[]  = {"(m/s)", "(m/s^2)", "(rad/s)", "(rad/s^2)"};

void FsdMatlabPlots::init()
{
  m_lfcMat = string("magenta");
  m_rfcMat = string("green");
  m_lrcMat = string("blue");
  m_rrcMat = string("red");
  m_ocMat  = string("--black");

  if (!m_generatePlots) {
    string matName = "regression.txt";
    m_mat.open(matName.c_str());
    m_mat << setprecision(4);
  }
}

FsdMatlabPlots::FsdMatlabPlots(bool generatePlots)
  : m_OwnFsd(true), m_generatePlots(generatePlots), m_sampleFrequency(30.), m_time(0.),
    m_maxWheelSpeed(NAN), m_maxWheelAccel(NAN), m_maxSteerRate(NAN), m_maxSteerAccel(NAN)
{
  init();
  // Hard code values from K10 Red/Black Rovers
  Vector2Vector wheelLocations(4);

  double wheelBaseWidth  = .778;
  double wheelBaseLength = .812;
  unsigned int numWheels = 0;
  
  for (int y = 0; y < 2; ++y)
    for (int x = 0; x < 2; ++x) {
      Vector2 m((1 - 2 * y) * wheelBaseLength / 2.0, -(1 - 2 * x) * wheelBaseWidth / 2.0);
      wheelLocations[numWheels++] = m;
    }
  double wheelRadius = 0.127;

  m_fsd = new kn::FastestSteerAndDriveCWL(
    m_sampleFrequency, wheelRadius, wheelLocations);
}

FsdMatlabPlots::FsdMatlabPlots(kn::FastestSteerAndDriveCWL* fsd) 
  : m_fsd(fsd), m_OwnFsd(false), m_sampleFrequency(30.),
    m_maxWheelSpeed(NAN), m_maxWheelAccel(NAN), m_maxSteerRate(NAN), m_maxSteerAccel(NAN)
{
  init();
}

FsdMatlabPlots::~FsdMatlabPlots()
{
  if (m_generatePlots) {
    ofstream plots("pathAll.m");
    for (std::list<std::string>::iterator iter = m_plotNames.begin(); iter != m_plotNames.end(); ++iter)
      plots << "disp('path_" << *iter << "'); path_" << *iter << endl;
  } else {
    m_mat.close();
  }

  if (m_OwnFsd)
    delete m_fsd;
}

void FsdMatlabPlots::initPath(
  const double wheelSpeed,
  const double wheelAccel, 
  const double steerRate,
  const double steerAccel) 
{
  m_commands.str("");

  m_commands << "% WheelAccel(m/s^2) \tSteerAccel(rad/s/s) \tSteerRate(rad/s)\n"
             << "% " << wheelAccel << "\t\t\t" << steerAccel << "\t\t\t" << steerRate << endl << endl
             << "% PathTime(s)      \tWheelSpeed(m/s) \tCurvature(1/m)         \tCrab(rad)\n";

  m_time = 0.;

  m_fsd->setTransition(wheelAccel, steerAccel, steerRate);

  m_maxWheelSpeed = wheelSpeed;
  m_maxWheelAccel = wheelAccel;
  m_maxSteerRate = steerRate;
  m_maxSteerAccel = steerAccel;

  if (m_OwnFsd) {
    kn::CoordinatedWheelLocomotion::CwlVector empty(m_fsd->numWheels() * 2, 0.);  
    m_fsd->initPath(kn::WheelGroupState(ACE_Time_Value::zero, empty, empty, 0., 0., 0., 0., 0.));
  }
}

void FsdMatlabPlots::computePath(const double time, const double targetWheelSpeed, const double targetCurvature, const double targetCrab) 
{
  m_commands << "% " << time << "\t\t\t" << targetWheelSpeed << "\t\t\t" << targetCurvature << "\t\t\t" << targetCrab << endl;

  boost::timer timer;
  timer.restart();
  m_fsd->clearPath();
  m_fsd->appendPath(time, targetWheelSpeed, targetCurvature, targetCrab);
  m_time += time;
  double elapsed = timer.elapsed();
  if (elapsed > 0)
    cout << "Real Time Ratio: " << time / elapsed << endl;
  if (printFsdDebug || printFsdWarnings)
    m_fsd->printWarnings(cerr, printFsdDebug);
}

double FsdMatlabPlots::computeSlowTime(const double accel, const double fastTime, 
                                       const double fastSpeed, const double alpha)
{
  double slowSpeed = alpha * fastSpeed;
  double decelTime = (fastSpeed - slowSpeed) / accel;
  double decelPathLength = fastSpeed * decelTime + accel * decelTime * decelTime / 2.;
  double slowTime = 0.;

  if (fastTime > 0.) {
    // Time left over after reaching target curvature & speed, so add a slowdown path
    double pathLength = fastTime * fastSpeed; 

    if (decelPathLength > pathLength) {
      // Not enough path length to reach target slowdown speed;
      slowTime = (sqrt(fastSpeed * fastSpeed + 2 * accel * pathLength) - fastSpeed) / accel;
    } else {
      // Target slowdown speed reached.
      slowTime = decelTime + (pathLength - decelPathLength) / slowSpeed;
    }
  }
  return slowTime;
}

void FsdMatlabPlots::computeAlphaPath(const double wheelAccel, const double steerAccel, const double steerRate, 
                                      const double time, const double targetWheelSpeed, 
                                      const double targetCurvature, const double alpha) 
{
  initPath(wheelAccel, steerAccel, steerRate, targetWheelSpeed);
  computePath(NAN, targetWheelSpeed, targetCurvature);

  double slowTime = computeSlowTime(fabs(wheelAccel), time - m_fsd->getElapsedTime(), fabs(targetWheelSpeed), alpha);
  if (slowTime > 0)
    appendPath(slowTime, alpha * targetWheelSpeed, targetCurvature);
}

void FsdMatlabPlots::appendPath(const double time, const double targetWheelSpeed, const double targetCurvature, const double targetCrab) 
{
  m_commands << "% " << time << "\t\t\t" << targetWheelSpeed << "\t\t\t" << targetCurvature << "\t\t\t" << targetCrab << endl;

  boost::timer timer;
  timer.restart();
  m_time += time;
  m_fsd->appendPath(m_time, targetWheelSpeed, targetCurvature, targetCrab);
  double elapsed = timer.elapsed();
  if (elapsed > 0)
    cout << "Real Time Ratio: " << time / elapsed << endl;
  if (printFsdDebug || printFsdWarnings)
    m_fsd->printWarnings(cerr, printFsdDebug);
}

void FsdMatlabPlots::computeAlignmentPath(const double pathTime)
{
  m_commands << "% " << pathTime << endl;

  m_fsd->clearPath();
  unsigned int numPoints = m_fsd->appendAlignmentPath(pathTime);
  m_time += static_cast<double>(numPoints) / m_sampleFrequency;

  if (printFsdDebug || printFsdWarnings)
    m_fsd->printWarnings(cerr, printFsdDebug);
}

void FsdMatlabPlots::appendAlphaPath(const double wheelAccel, const double steerAccel, const double steerRate, 
                                     const double time, const double targetWheelSpeed, const double targetCurvature,
                                     const double alpha) 
{
  m_fsd->setTransition(wheelAccel, steerAccel, steerRate);
  appendPath(NAN, targetWheelSpeed, targetCurvature);

  double slowTime = computeSlowTime(fabs(wheelAccel), time - m_fsd->getElapsedTime(), fabs(targetWheelSpeed), alpha);
  appendPath(slowTime, alpha * targetWheelSpeed, targetCurvature);
}


void FsdMatlabPlots::initPathFromCurrentState()
{
  m_commands << "% initPathFromCurrentState" << endl;

  // get the current state from FSD as it is extracted from 

  double speed = m_fsd->getPpmSpeeds().back();
  double curvature = m_fsd->getPpmCurvatures().back();
  double curvatureRate = m_fsd->getPpmCurvatureRates().back();
  double crab = m_fsd->getPpmCrabs().back();
  double crabRate = m_fsd->getPpmCrabRates().back();
  
  DoubleVectorVector const& ppmPath = m_fsd->getPpmMotorPositions();
  DoubleVectorVector const& ppmMotorSpeeds =  m_fsd->getPpmMotorSpeeds();
  
  DoubleVector motorPositions;
  DoubleVector motorSpeeds;

  {
    motorPositions.reserve(ppmPath.size());
    DoubleVectorVector::const_iterator first, last = ppmPath.end();
    for (first = ppmPath.begin(); first != last; ++first) {
      motorPositions.push_back(first->back());
    }
  }
  
  {
    motorSpeeds.reserve(ppmMotorSpeeds.size());
    DoubleVectorVector::const_iterator first, last = ppmMotorSpeeds.end();
    for (first = ppmMotorSpeeds.begin(); first != last; ++first) {
      motorSpeeds.push_back(first->back());
    }
  }
      

  m_fsd->initPath(kn::WheelGroupState(ACE_Time_Value::zero,
                                      motorPositions, motorSpeeds,
                                      curvature, curvatureRate, speed,
                                      crab, crabRate));
}

void FsdMatlabPlots::initUnalignedSteerPositions(const double steerRate, const double steerAccel, 
                                                 const DoubleVector& steerPositions)
{
  m_commands << "% initUnalignedSteerPositions" << endl;

  m_maxSteerAccel = steerAccel;
  m_maxSteerRate  = steerRate;
  m_maxWheelSpeed = 0.;
  m_maxWheelAccel = 0.;

  m_fsd->setTransition(m_maxWheelAccel, m_maxSteerAccel, m_maxSteerRate);

  double speed = 0.;
  double curvature = NAN;
  double curvatureRate = 0.;
  double crab = 0.;
  double crabRate = 0.;

  DoubleVector motorPositions;
  DoubleVector motorSpeeds;
  for (unsigned int i = 0; i < m_fsd->numWheels(); ++i) {
    motorPositions.push_back(0.);
    motorSpeeds.push_back(0.);
  }

  for (unsigned int i = 0; i < m_fsd->numWheels(); ++i) {
    motorPositions.push_back(steerPositions[i]);
    motorSpeeds.push_back(0.);
  }
  
  m_time = 0.;

  m_fsd->initPath(kn::WheelGroupState(ACE_Time_Value::zero,
                                      motorPositions, motorSpeeds,
                                      curvature, curvatureRate, speed,
                                      crab, crabRate));
}

void FsdMatlabPlots::initPathFromPreviousState(double timeOffset)
{
  DoubleVectorVector const& ppmPath = m_fsd->getPpmMotorPositions();
  int numSamples = ppmPath[0].size();

  int sampleInterrupt = 0;
  if (timeOffset <= 0) {
    // Assume time offset is from end of path
    sampleInterrupt = numSamples + static_cast<int>(timeOffset * m_sampleFrequency - .5);
  } else {
    // Assume time offset is from start of path
    sampleInterrupt = static_cast<int>(timeOffset * m_sampleFrequency + .5);
  }
  if (sampleInterrupt < 0) sampleInterrupt = 0;
  if (sampleInterrupt >= numSamples) sampleInterrupt = numSamples - 1;
  
  m_fsd->resizePath(sampleInterrupt);
}

void FsdMatlabPlots::setVariableNames(const string& suffix)
{
  m_stimeMat = string("time_") + suffix;
  m_xMat = string("x_") + suffix;
  m_yMat = string("y_") + suffix;

  m_lfdMat   = string("left_front_drive_") + suffix;
  m_rfdMat   = string("right_front_drive_") + suffix;
  m_lrdMat   = string("left_rear_drive_") + suffix;
  m_rrdMat   = string("right_rear_drive_") + suffix;
  m_driveMat = string("drive_") + suffix;
  m_driveSpeedMat = string("driveSpeed_") + suffix;
  m_maxDriveSpeedMat = string("maxDriveSpeed_") + suffix;

  m_lfsMat   = string("left_front_steer_") + suffix;
  m_rfsMat   = string("right_front_steer_") + suffix;
  m_lrsMat   = string("left_rear_steer_") + suffix;
  m_rrsMat   = string("right_rear_steer_") + suffix;
  m_steerMat = string("steer_") + suffix;
  m_steerRateMat = string("steerRate_") + suffix;
  m_maxSteerRateMat = string("maxSteerRate_") + suffix;

  m_fMat = string("sampleFrequency_") + suffix;
  m_curvatureMat = string("curvature_") + suffix;
  m_crabMat = string("crab_") + suffix;
  m_curvatureRateMat = string("curvatureRate_") + suffix;
  m_crabRateMat = string("crabRate_") + suffix;
  m_speedMat = string("speed_") + suffix;
}

void FsdMatlabPlots::outputMatlabPlots(const string& name, const string& suffix)
{
  const string varName = name + "Vars_" + suffix;

  outputMatlabVariables(varName, suffix);

  if (m_generatePlots) {
    const string matName = name + "_" + suffix + ".m";
    ofstream mat(matName.c_str());

    // Close al windows and clear all variables
    mat << "close all\n";
    mat << "clear all\n";
    
    // Load all the variables;
    mat << varName << endl;

    mat << m_driveMat << " = [" << m_lfdMat << "' " << m_rfdMat << "' " << m_lrdMat << "' " << m_rrdMat << "']';" << endl;
    mat << m_driveSpeedMat << " = diff(" << m_driveMat << "')' * " << m_fMat << ";" << endl;
    mat << m_maxDriveSpeedMat << " = max(abs(" << m_driveSpeedMat << "));" << endl;
    
    mat << m_steerMat << " = [" << m_lfsMat << "' " << m_rfsMat << "' " << m_lrsMat << "' " << m_rrsMat << "']';" << endl;
    mat << m_steerRateMat << " = diff(" << m_steerMat << "')' * " << m_fMat << ";" << endl;
    mat << m_maxSteerRateMat << " = max(abs(" << m_steerRateMat << "));" << endl << endl;
    
    // Generate sheet of matlab plots
    stdSheet(mat);
    sheetPostamble(mat, name, suffix);
    
    // Generate sheet of matlab error plots for wheel variables
    errSheet_1(mat);
    sheetPostamble(mat, name + "Wheel", suffix);
    
    // Generate detailed sheet
    errSheet_1a(mat, suffix);
    
    // Generate sheet of matlab error plots for vehicle variables
    errSheet_2(mat);
    sheetPostamble(mat, name + "Rover", suffix);
  }
}

void FsdMatlabPlots::sheetPostamble(ofstream& mat, const string& name, const string& suffix)
{
  mat << "orient landscape\n";

  mat << "axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off',"
      << "'Units','normalized', 'clipping' , 'off');\n";
  mat << "text(0.5, 1,'\\bf " << name << "\\_" << suffix << "',"
      << "'HorizontalAlignment','center','VerticalAlignment', 'top', 'fontsize', 24);\n";

  mat << "print -dpdf " << name << "_" << suffix << endl;
}

void FsdMatlabPlots::stdSheet(ofstream& mat) 
{
  mat << "figure('visible','off');\n";
  mat << "subplot(5,3,1)\n";
  mat << "plot(" << m_xMat << "," << m_yMat << ")\n";
  mat << "grid on\n";
  mat << "axis equal\n";
  mat << "xlabel({'X (m)'})\n";
  mat << "ylabel({'Y';'(m)'})\n\n";

  mat << "subplot(5,3,2)\n";
  mat << "plot(" << m_stimeMat << "," << m_speedMat << ")\n";
  mat << "grid on\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'speed';'(m/s)'})\n\n";  

  mat << "subplot(5,3,3)\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_speedMat << ") * " << m_fMat << ")\n";
  mat << "grid on\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'acceleration';'(m/s^2)'})\n\n";  

  mat << "subplot(5,3,4)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "," << m_lfdMat << ", '" << m_lfcMat << "')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "," << m_rfdMat << ", '" << m_rfcMat << "')\n";
  mat << "plot(" << m_stimeMat << "," << m_lrdMat << ", '" << m_lrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "," << m_rrdMat << ", '" << m_rrcMat << "')\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'Wheel Dist';'(m)'})\n\n";

  mat << "subplot(5,3,5)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_lfdMat << ") * " << m_fMat << ", '" << m_lfcMat << "')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_rfdMat << ") * " << m_fMat << ", '" << m_rfcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_lrdMat << ") * " << m_fMat << ", '" << m_lrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_rrdMat << ") * " << m_fMat << ", '" << m_rrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "," << m_speedMat  << ", '" << m_ocMat << "')\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'wheel speed';'(m/s)'})\n\n";

  mat << "subplot(5,3,6)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_lfdMat  << ")) * " << m_fMat << " ^2, '" << m_lfcMat << "')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_rfdMat << ")) * " << m_fMat << " ^2, '" << m_rfcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_lrdMat << ")) * " << m_fMat << " ^2, '" << m_lrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_rrdMat << ")) * " << m_fMat << " ^2, '" << m_rrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff("    << m_speedMat  << ") * " << m_fMat << ", '"    << m_ocMat << "')\n";
  mat << "xlabel({'time';'(s)'})\n";
  mat << "ylabel({'wheel accel';'(m/s^2)'})\n\n";

  mat << "subplot(5,3,7)\n";
  mat << "plot(" << m_stimeMat << "," << m_curvatureMat << ")\n";
  mat << "grid on\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'curvature';'(1/m)'})\n\n";  

  mat << "subplot(5,3,8)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "," << m_curvatureRateMat << ")\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_curvatureMat << ") * " << m_fMat << ", '" << m_ocMat << "')\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'dk/dt';'(1/m/s)'})\n\n";  

  mat << "subplot(5,3,9)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_curvatureRateMat << ") * " << m_fMat << ")\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_curvatureMat << ")) * " << m_fMat << "^2, '" << m_ocMat << "')\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'d^2k/dt^2';'(1/m/s^2)'})\n\n";  

  mat << "subplot(5,3,10)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "," << m_lfsMat << ", '" << m_lfcMat << "')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "," << m_rfsMat << ", '" << m_rfcMat << "')\n";
  mat << "plot(" << m_stimeMat << "," << m_lrsMat << ", '" << m_lrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "," << m_rrsMat << ", '" << m_rrcMat << "')\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'wheel rotation';'(rad)'})\n\n";

  mat << "subplot(5,3,11)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_lfsMat  << ") * " << m_fMat << ", '" << m_lfcMat << "')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_rfsMat << ") * " << m_fMat << ", '" << m_rfcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_lrsMat << ") * " << m_fMat << ", '" << m_lrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_rrsMat << ") * " << m_fMat << ", '" << m_rrcMat << "')\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'steer speed';'(rad/s)'})\n\n";

  mat << "subplot(5,3,12)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_lfsMat  << ")) * " << m_fMat << "^2, '" << m_lfcMat << "')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_rfsMat << ")) * " << m_fMat << "^2, '" << m_rfcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_lrsMat << ")) * " << m_fMat << "^2, '" << m_lrcMat << "')\n";
  mat << "plot(" << m_stimeMat << "(1:end-2), diff(diff(" << m_rrsMat << ")) * " << m_fMat << "^2, '" << m_rrcMat << "')\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'steer accel';'(rad/s^2)'})\n\n";

  mat << "subplot(5,3,13)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "," << m_crabMat << ")\n";
  mat << "grid on\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'crab';'(rad)'})\n\n";

  mat << "subplot(5,3,14)\n";
  mat << "hold off\n";
  mat << "plot(" << m_stimeMat << "," << m_crabRateMat << ")\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_stimeMat << "(1:end-1), diff(" << m_crabMat << ") * " << m_fMat << ")\n";
  mat << "xlabel({'time (s)'})\n";
  mat << "ylabel({'d/dt crab';'(rad/s)'})\n\n";  

  mat << "subplot(5,3,15)\n";
  mat << "hold off\n";
  mat << "plot(" << m_lfsMat << "(1:end-1), diff(" << m_lfsMat  << ") * " << m_fMat << ", '" << m_lfcMat << "')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << m_rfsMat << "(1:end-1), diff(" << m_rfsMat << ") * " << m_fMat << ", '" << m_rfcMat << "')\n";
  mat << "plot(" << m_lrsMat << "(1:end-1), diff(" << m_lrsMat   << ") * " << m_fMat << ", 'blue')\n";
  mat << "plot(" << m_rrsMat << "(1:end-1), diff(" << m_rrsMat  << ") * " << m_fMat << ", 'red')\n";
  mat << "xlabel({'steer position (rad)'})\n";
  mat << "ylabel({'steer speed';'(rad/s)'})\n\n";
}

void FsdMatlabPlots::setDerivative(ofstream& mat, const string& var, unsigned int level)
{
  mat << "der = ";
  for (unsigned int i = 0; i < level; ++i)
    mat << "diff(";
  mat << var;
  for (unsigned int i = 0; i < level; ++i)
    mat << ")";
  if (level > 0)
    mat << " * " << m_fMat << "^" << level;
  mat << ";\n";
}


// Wheel Variables
void FsdMatlabPlots::errSheet_1(ofstream& mat) 
{
  const unsigned int numCols = 4;
  const unsigned int numRows = 4;

  string times[3] = {m_stimeMat, m_stimeMat + "(1:end-1)", m_stimeMat + "(1:end-2)"};

  string colorWheelNames[numCols] = {m_lfcMat, m_rfcMat, m_lrcMat, m_rrcMat};
  string driveWheelNames[numCols] = {m_lfdMat, m_rfdMat, m_lrdMat, m_rrdMat};
  string steerWheelNames[numCols] = {m_lfsMat, m_rfsMat, m_lrsMat, m_rrsMat};

  mat << "figure('visible','off');\n";
  for (unsigned int row = 0; row < numRows; ++row)
    for (unsigned int col = 0; col < numCols; ++col)
    {
        mat << "\nsubplot(" << numRows << "," << numCols << "," << row * numRows + col + 1 << ")\n";
        mat << "grid on\n";
        unsigned int derLev = row % 2 + 1;
        bool isDriveWheel = (row <= 1);
        string wheelId = isDriveWheel ? driveWheelNames[col] : steerWheelNames[col];

        if (derLev == 1) 
          mat << "der = diff(" << wheelId << ") * " << m_fMat << ";\n";
        else
          mat << "der = diff(diff(" << wheelId << ")) * " << m_fMat << "^2;\n";
        
        mat << "plot(" << times[derLev] << ", der, '" << colorWheelNames[col] << "')\n";
        if (col == 0)
          mat << "ylabel({'" << s_wheelOper[row] << "';'" << s_wheelUnit[row] << "'})\n";
        if (row == numRows - 1)
          mat << "xlabel('time(s)')\n";

        mat << "hold on\n";

        double maxWheel = NAN;
        if (derLev == 1)
          maxWheel = isDriveWheel ? m_maxWheelSpeed : m_maxSteerRate;
        else
          maxWheel = isDriveWheel ? m_maxWheelAccel : m_maxSteerAccel;

        //        mat << "if (max(der) > " << maxWheel << ")\n";
        mat << "  plot(" << times[derLev] << "," << maxWheel << ")\n";
        //        mat << "end\n";

        //        mat << "if (min(der) < " << -maxWheel << ")\n";
        mat << "  plot(" << times[derLev] << "," << -maxWheel << ")\n";
        //        mat << "end\n";

        if (row == 0)
          mat << "title({'" << s_wheelNames[col] << "'})\n";
        
        mat << "hold off\n";
    }
}

// Wheel Variables
void FsdMatlabPlots::errSheet_1a(ofstream& mat, const string& suffix) 
{
  const unsigned int numCols = 4;
  const unsigned int numRows = 4;

  string times[3] = {m_stimeMat, m_stimeMat + "(1:end-1)", m_stimeMat + "(1:end-2)"};
  string driveWheelNames[numCols] = {m_lfdMat, m_rfdMat, m_lrdMat, m_rrdMat};
  string steerWheelNames[numCols] = {m_lfsMat, m_rfsMat, m_lrsMat, m_rrsMat};

  size_t maxNameLen = 0;
  for (unsigned int nameIdx = 0; nameIdx < numCols; ++nameIdx) 
    maxNameLen = max(maxNameLen, s_wheelNames[nameIdx].length());

  size_t maxOperLen = 0;
  for (unsigned int operIdx = 0; operIdx < numCols; ++operIdx) 
    maxOperLen = max(maxOperLen, s_wheelOper[operIdx].length());

  const unsigned int filename_sublen = maxNameLen + maxOperLen;

  for (unsigned int row = 0; row < numRows; ++row)
    for (unsigned int col = 0; col < numCols; ++col)
    {
        mat << "grid on\n";
        unsigned int derLev = row % 2 + 1;
        bool isDriveWheel = (row <= 1);
        string wheelId = isDriveWheel ? driveWheelNames[col] : steerWheelNames[col];

        double maxWheel = NAN;
        if (derLev == 1) {
          maxWheel = isDriveWheel ? m_maxWheelSpeed : m_maxSteerRate;
          mat << "der = diff(" << wheelId << ") * " << m_fMat << ";\n";
        } else {
          maxWheel = isDriveWheel ? m_maxWheelAccel : m_maxSteerAccel;
          mat << "der = diff(diff(" << wheelId << ")) * " << m_fMat << "^2;\n";
        }

        // Always have at least two underscores separting filename and suffix.  Keep total filename length same for
        // nice formatting.
        unsigned int numUnderScores = filename_sublen + 2 - s_wheelNames[col].length() - s_wheelOper[row].length();
        string filename = s_wheelNames[col];
        
        for (unsigned int idx = 0; idx < numUnderScores; ++idx)
          filename = filename + "_";

        filename = filename + s_wheelOper[row];

        // Swallow white space.
        size_t s = filename.find(' ');
        while (s != string::npos) {
          filename.erase(s,1);
          s = filename.find(' ');
        }

        mat << "limitPlot('" << filename << "', '" << suffix << "', '" << s_wheelNames[col] << "', 'time (s)', " 
            << times[derLev] << ", '" << s_wheelOper[row] << " " << s_wheelUnit[row] << "', der, " 
            << -maxWheel << ", " << maxWheel << ")\n";
    }
}


// Vehicle Variables
void FsdMatlabPlots::errSheet_2(ofstream& mat) 
{
  string times[3] = {m_stimeMat, m_stimeMat + "(1:end-1)", m_stimeMat + "(1:end-2)"};

  ////////////////////////////////////////////////////////////////////////
  //                            //                                      //
  // rover speed                //      angular speed                   //      
  //                            //                                      //
  // rover acceleration         //      centripetal acceleration        //
  //                            //                                      //
  // rover jerk                 //      centripetal jerk                //
  //                            //                                      //
  ////////////////////////////////////////////////////////////////////////


  const unsigned int numCols = 2;
  const unsigned int numRows = 3;
  
  unsigned int col = 0;
  unsigned int row = 0;
  const string roverOper[]  = {
    "'Rover Speed';'(m/s)'", "'Rover Acceleration';'(m/s^2)'", "'Rover Jerk';'(m/s^3)'"};

  mat << "figure('visible','off');\n";
  for (row = 0; row < numRows; ++row) {
    
    mat << "\nsubplot(" << numRows << "," << numCols << "," << row * numCols + col + 1 << ")\n";
    mat << "grid on\n";

    setDerivative(mat, m_speedMat, row);
    
    mat << "plot(" << times[row] << ", der)\n";

    // Display limits on plot (only wheel limits for now).
    double maxWheel = NAN;
    if (row == 0)
      maxWheel = m_maxWheelSpeed;
    else if (row == 1)
      maxWheel = m_maxWheelAccel;
    mat << "hold on\n";
    mat << "  plot(" << times[row] << "," << maxWheel << ")\n";
    mat << "hold off\n";
    if (row == numRows - 1)
      mat << "xlabel('time(s)')\n";
    mat << "ylabel({" << roverOper[row] << "})\n";
  }

  col = 1;
  row = 0;
  mat << "\nsubplot(" << numRows << "," << numCols << "," << row * numCols + col + 1 << ")\n";
  mat << "plot(" << times[0] << ", " << m_speedMat << " .* " << m_curvatureMat << ")\n";
  mat << "ylabel({'Angular Speed';'(rad/s)'})\n";

  col = 1;
  row = 1;
  string centAccel = "centripetalAccel";
  mat << "\nsubplot(" << numRows << "," << numCols << "," << row * numCols + col + 1 << ")\n";
  mat << centAccel << " = " << m_speedMat << " .^2 .* " << m_curvatureMat << ";\n";
  mat << "plot(" << times[0] << ", " << centAccel << ")\n";
  mat << "ylabel({'Centripetal Acceleration';'(m/s^2)'})\n";

  col = 1;
  row = 2;
  mat << "\nsubplot(" << numRows << "," << numCols << "," << row * numCols + col + 1 << ")\n";
  int derLevel = 1;
  setDerivative(mat, centAccel, derLevel);
  mat << "plot(" << times[derLevel] << ", der)\n";
  mat << "ylabel({'Centripetal Jerk';'(m/s^3)'})\n";
  mat << "xlabel('time (s)')\n";
}

void FsdMatlabPlots::outputMatlabVariables(const string& varName, const string& suffix)
{
  // Generate sheet of matlab plots

  if (m_generatePlots) {
       string matName = varName + ".m";
       m_mat.open(matName.c_str());
       m_mat << setprecision(10);
  }

  double f = m_fsd->getSampleFrequency();

  std::vector<CwlVector > const& ppmPath = m_fsd->getPpmMotorPositions();
  ATrans2 pose = ATrans2::Identity();
  ATrans2Vector path;
  m_fsd->renderTrajectory(pose, path);
  m_mat << m_commands.str() << endl;

  m_mat << "sampleFrequency_" << suffix << " = " << f << ";\n\n";

  m_mat << "time_" << suffix << " = ...\n[";
  for (unsigned int i = 0; i < path.size(); ++i)
    m_mat << i / f << " ";
  m_mat << "];\n\n";

  CwlVector ppmSpeeds = m_fsd->getPpmSpeeds();
  m_mat << "speed_" << suffix << " = ...\n[";
  for (unsigned int i = 0; i < path.size(); ++i)
    m_mat << ppmSpeeds[i] << " ";
  m_mat << "];\n\n";

  CwlVector ppmCurvatures = m_fsd->getPpmCurvatures();
  m_mat << "curvature_" << suffix << " = ...\n[";
  for (unsigned int i = 0; i < path.size(); ++i)
    m_mat << ppmCurvatures[i] << " ";
  m_mat << "];\n\n";

  CwlVector ppmCurvatureRates = m_fsd->getPpmCurvatureRates();
  m_mat << "curvatureRate_" << suffix << " = ...\n[";
  for (unsigned int i = 0; i < path.size(); ++i)
    m_mat << ppmCurvatureRates[i] << " ";
  m_mat << "];\n\n";

  CwlVector ppmCrabs = m_fsd->getPpmCrabs();
  m_mat << "crab_" << suffix << " = ...\n[";
  for (unsigned int i = 0; i < path.size(); ++i)
    m_mat << ppmCrabs[i] << " ";
  m_mat << "];\n\n";

  CwlVector ppmCrabRates = m_fsd->getPpmCrabRates();
  m_mat << "crabRate_" << suffix << " = ...\n[";
  for (unsigned int i = 0; i < path.size(); ++i)
    m_mat << ppmCrabRates[i] << " ";
  m_mat << "];\n\n";

  // Wheels
  const string wheels[] = {
    "left_front_drive", "right_front_drive", "left_rear_drive", "right_rear_drive",
    "left_front_steer", "right_front_steer", "left_rear_steer", "right_rear_steer"
  };
  for (unsigned int j = 0; j < 8; ++j) {
    m_mat << wheels[j] << "_" << suffix << " = ...\n[";
    for (unsigned int i = 0; i < ppmPath[j].size(); ++i) {

      double value = ppmPath[j][i];
      if (j < 4)
        value *= m_fsd->wheelRadius();
      m_mat << value << " ";  
    }
    m_mat << "];\n\n";
  }

  const string var[] = {"x", "y", "heading"};
  for (unsigned int j = 0; j < 3; ++j) {
    m_mat << var[j] << "_" << suffix << " = ...\n[";
    for (unsigned int i = 0; i < path.size(); ++i) {
      if (j < 2) {
	m_mat << path[i].translation()[j] << " ";
      }
      else {
	m_mat << atan2(path[i].rotation()(1,0), path[i].rotation()(0,0)) << " ";
      }
    }
    m_mat << "];\n\n";
  }

  if (m_generatePlots)
    m_mat.close();
}

void FsdMatlabPlots::outputMatlab(const string& name, const string& suffix)
{
  cout << name << "_" << suffix << endl;
  // Output matlab code to set all variables from data.

  setVariableNames(suffix);
  
  // Output matlab code to plot variables.
  outputMatlabPlots(name, suffix);
  
  // Keep track of plots generated
  m_plotNames.push_back(suffix);
}

// Compares an entire directory of matlab plots with this directory, assuming the same testFsd.cpp file.
// Any differences will be automatically plotted and saved as pdf files.
void FsdMatlabPlots::regressionTest(const string& dir)
{
  // Generate sheet of matlab plots
  ofstream loadExt("loadExt.m");

  loadExt << "function [";
  for (std::list<std::string>::iterator iter = m_plotNames.begin(); iter != m_plotNames.end(); ++iter)
    loadExt << "x" + *iter << " "
            << "y" + *iter << " ";
  loadExt << "] = loadExt(d)\n"
          << "  curDir = pwd;\n"
          << "  cd(d);\n";
  for (std::list<std::string>::iterator iter = m_plotNames.begin(); iter != m_plotNames.end(); ++iter)
    loadExt  << "  pathVars" + *iter << "\n";
  loadExt << "  cd(curDir);\n"
          << "end\n";
  
  loadExt.close();

  ofstream pathComp("pathComp.m");

  pathComp << "[";
  for (std::list<std::string>::iterator iter = m_plotNames.begin(); iter != m_plotNames.end(); ++iter)
    pathComp << "x_ext" + *iter << " "
             << "y_ext" + *iter << " ";
  pathComp << "] = loadExt('" << dir << "');\n";
  for (std::list<std::string>::iterator iter = m_plotNames.begin(); iter != m_plotNames.end(); ++iter)
    pathComp  << "pathVars" + *iter << ";\n";
  
  for (std::list<std::string>::iterator iter = m_plotNames.begin(); iter != m_plotNames.end(); ++iter) {
    pathComp << "maxDiff = max(sqrt((x" 
             << *iter << "-x_ext" << *iter << ").^2+(y" << *iter << "-y_ext" << *iter << ")));\n";
    pathComp << "if (maxDiff > 0)\n";
    pathComp << "  figure('visible','off');\n";
    pathComp << "  plot("
             << "x_ext" << *iter << ",y_ext" << *iter << ", 'r',"
             << "x"     << *iter << ",y"     << *iter << ", 'g');\n" ;
    pathComp << "  grid on;\n";
    pathComp << "  str = sprintf('path\\\\" << *iter << " Max Offset: %g', maxDiff);\n";
    pathComp << "  disp(str);\n";
    pathComp << "  title(str);\n";
    pathComp << "  h = legend('" << dir << "', 'current',2);\n";
    pathComp << "  orient landscape\n";
    pathComp << "  print -dpdf " << "CompDiff" << *iter << ".pdf\n";
    pathComp << "end\n\n";
  }
  pathComp.close();
  cout << "pathComp\n";
}

void FsdMatlabPlots::outputMatlabComp(const string& base, const string& sufA, const string& sufB) const
{
  const string matName = base + "Comp" + sufA + sufB + ".m";
  ofstream mat(matName.c_str());
  
  mat << setprecision(10);

  // Load the variables from both sets
  mat << base + "Vars" + sufA << endl;
  mat << base + "Vars" + sufB << endl;

  const string tA = string("time") + sufA;
  const string tB = string("time") + sufB;

  const string xA = string("x") + sufA;
  const string xB = string("x") + sufB;
  const string yA = string("y") + sufA;
  const string yB = string("y") + sufB;
  const string kA = string("curvature") + sufA;
  const string kB = string("curvature") + sufB;
  const string vA = string("speed") + sufA;
  const string vB = string("speed") + sufB;

  mat << "subplot(2,2,1)\n";
  mat << "plot(" << xA << "," << yA << ", 'red')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << xB << "," << yB << ", 'blue')\n";
  mat << "axis equal\n";
  mat << "xlabel('X (meters)')\n";
  mat << "ylabel('Y (meters)')\n";
  mat << "str = sprintf('Max distance offset: %f', max(sqrt((x_1-x_2).^2+(y_1-y_2).^2)));\n";
  mat << "title(str)\n";

  mat << "subplot(2,2,2)\n";
  mat << "plot(" << tA << "," << vA << ", 'red')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << tB << "," << vB << ", 'blue')\n";
  mat << "xlabel('time (s)')\n";
  mat << "ylabel('speed (m/s)')\n";  

  mat << "subplot(2,2,3)\n";
  mat << "plot(" << tA << "," << kA << ", 'red')\n";
  mat << "grid on\n";
  mat << "hold on\n";
  mat << "plot(" << tB << "," << kB << ", 'blue')\n";
  mat << "xlabel('time (s)')\n";
  mat << "ylabel('curvature (m/s)')\n";  

  mat << "orient landscape\n";
  mat << "print -dpdf " << base << "Comp" << sufA << sufB << endl;
}


