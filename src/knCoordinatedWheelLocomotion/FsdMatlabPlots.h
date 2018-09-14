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

#ifndef TEST_FSD_H
#define TEST_FSD_H

#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <list>

#include "FastestSteerAndDriveCWL.h"

typedef std::vector<double> DoubleVector;
typedef std::vector<DoubleVector> DoubleVectorVector;

class FsdMatlabPlots
{
 public:
  typedef std::vector<double> CwlVector;

  FsdMatlabPlots(bool generatePlots = true);
  FsdMatlabPlots(kn::FastestSteerAndDriveCWL* m_fsd);
  ~FsdMatlabPlots();

  void computePath(const double time, const double targetSpeed, const double targetCurvature, const double targetCrab = 0.0);
  void appendPath(const double time, const double targetSpeed, const double targetCurvature, const double targetCrab = 0.0);
  void computeAlignmentPath(const double pathTime);

  void computeAlphaPath(const double wheelAccel, const double steerAccel, const double steerRate, 
                        const double time, const double targetSpeed, const double targetCurvature, 
                        const double alpha);
  void appendAlphaPath(const double wheelAccel, const double steerAccel, const double steerRate, 
                       const double time, const double targetSpeed, const double targetCurvature,
                       const double alpha);

  void initPathFromCurrentState();
  void initUnalignedSteerPositions(const double steerRate, const double steerAccel, const DoubleVector& steerPositions);
  void initPathFromPreviousState(double timeOffset);
  void outputMatlab(const std::string& name, const std::string& varSuffix = "");
  void outputMatlabComp(const std::string& base, const std::string& compSuffixA, const std::string& compSuffixB) const;
  void initPath(const double wheelSpeed, const double wheelAccel, const double steerRate, const double steerAccel);
  void regressionTest(const std::string& dir);

private:
  void init();
  void outputMatlabVariables(const std::string& name, const std::string& suffix);
  void outputMatlabPlots(const std::string& name, const std::string& varSuffix);
  void sheetPostamble(std::ofstream& mat, const std::string& name, const std::string& suffix);
  void setDerivative(std::ofstream& mat, const std::string& var, unsigned int level);
  void stdSheet(std::ofstream& mat);
  void errSheet_1(std::ofstream& mat);
  void errSheet_1a(std::ofstream& mat, const std::string& suffix);
  void errSheet_2(std::ofstream& mat);
  double computeSlowTime(const double accel, const double fastTime, const double fastSpeed, const double alpha);
  void setVariableNames(const std::string& suffix);

  kn::FastestSteerAndDriveCWL* m_fsd;    
  bool m_OwnFsd;
  bool m_generatePlots;
  std::ofstream m_mat;

  double m_sampleFrequency;
  std::stringstream m_commands;
  std::list<std::string> m_plotNames;
  double m_time;
  double m_maxWheelSpeed;
  double m_maxWheelAccel;
  double m_maxSteerRate;
  double m_maxSteerAccel;

  std::string m_stimeMat;
  std::string m_xMat;
  std::string m_yMat;

  std::string m_lfdMat;
  std::string m_rfdMat;
  std::string m_lrdMat;
  std::string m_rrdMat;
  std::string m_driveMat;
  std::string m_driveSpeedMat;
  std::string m_maxDriveSpeedMat;
  std::string m_maxDriveAccelMat;

  std::string m_lfsMat;
  std::string m_rfsMat;
  std::string m_lrsMat;
  std::string m_rrsMat;
  std::string m_steerMat;
  std::string m_steerRateMat;
  std::string m_maxSteerRateMat;
  std::string m_maxSteerAccelMat;

  std::string m_lfcMat;
  std::string m_rfcMat;
  std::string m_lrcMat;
  std::string m_rrcMat;
  std::string m_ocMat;

  std::string m_fMat;
  std::string m_curvatureMat;
  std::string m_crabMat;
  std::string m_curvatureRateMat;
  std::string m_crabRateMat;
  std::string m_speedMat;

  static const std::string s_wheelNames[];
  static const std::string s_wheelOper[];
  static const std::string s_wheelUnit[];
};

#endif
