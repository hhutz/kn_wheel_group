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

#include "FastestSteerAndDriveCWL.h"
#include "TrapezoidProfile.h"

#include <cmath>
#include <iostream>
#include <float.h>
#include <exception>
#include <stdexcept>

// Detect when fastest steering wheel not same as fastest steer accelerating wheel.
//#define CHECK_CONT_ACCEL_LIMIT

using namespace std;

namespace
{
  double const LOC_EPSILON = 0.0001;
} 

namespace kn {

  typedef vector<double> DoubleVector;
  typedef vector<DoubleVector> DoubleVectorVector;

  FastestSteerAndDriveCWL::FastestSteerAndDriveCWL(double sampleFrequency, double wheelRadius, 
                                                  Vector2Vector const& wheelLocations) :
    CoordinatedWheelLocomotion(sampleFrequency, wheelRadius, wheelLocations),
    m_targetSteerAcceleration(0.),
    m_targetSteerRate(0.),
    m_maxDriveSpeed(0.),
    m_alignmentPath(m_numWheels)
  {
  }
  
  // set variables used to achieve targets
  void 
  FastestSteerAndDriveCWL::setTransition(double wheelAcceleration, double targetSteerAcceleration, 
                                         double targetSteerRate) throw()
  {
    m_wheelAcceleration = wheelAcceleration;
    m_targetSteerAcceleration = targetSteerAcceleration;
    m_targetSteerRate = targetSteerRate;
    m_steerProfile.setLimits(m_targetSteerRate, m_targetSteerAcceleration);
#ifdef ENABLE_FSD_CRABBING
    m_crabProfile.setLimits(m_targetSteerRate, m_targetSteerAcceleration);
#endif

    for (unsigned int i = 0; i < m_numWheels; ++i)
      m_alignmentPath[i].setLimits(m_targetSteerRate, m_targetSteerAcceleration);
  }

  unsigned int
  FastestSteerAndDriveCWL::maxPoints(double pathTime)
  {
    unsigned int numPathPoints = CWL_MAX_PATH;
        
    if (pathTime < 0.)
      numPathPoints = 0;

    if (!isnan(pathTime)) {
      numPathPoints = min(numPathPoints,
                          static_cast<unsigned int>(ceil(pathTime * m_sampleFrequency)));
    }

    // saturating substraction
    numPathPoints = (getPpmSpeeds().size() < numPathPoints)?
      numPathPoints - getPpmSpeeds().size() : 0;

    return numPathPoints;
  }

  unsigned int 
  FastestSteerAndDriveCWL::appendAlignmentPath(double pathTime)
  {
    double steerTime = 0.;
    for (unsigned int i = 0; i < m_numWheels; ++i) {
      try {
        m_alignmentPath[i].initializeState(
          m_state.motorPositions[i + m_numWheels], 
          m_state.motorSpeeds[i + m_numWheels], 0.0);
      } catch (const std::logic_error& logicError) {
        std::cerr << "Alignment Path: " << logicError.what() << std::endl;
        throw logicError;
      }
      steerTime = max(steerTime, m_alignmentPath[i].trajectoryTime());
    }

    m_state.speed = 0.;
    updateCrabAngle(0.);
    m_state.curvature = NAN;
    m_state.crabAngle = NAN;

    const unsigned int numPoints = min(CWL_MAX_PATH,
        static_cast<unsigned int>(ceil(min(pathTime, steerTime) * m_sampleFrequency)));

    reservePpm(numPoints);
    double time = 0.;
    for (unsigned int p = 0; p < numPoints; ++p) {
      time += m_sampleInterval;

      for (unsigned int i = 0; i < m_numWheels; ++i) {
        m_alignmentPath[i].computeState(time,
            m_state.motorPositions[i + m_numWheels],
            m_state.motorSpeeds[i + m_numWheels]);
      }
      if (p != numPoints - 1) 
      {
        // don't push last state yet in case we
        // can set a valid curvature.
        cwlPushState();
      }
    }

    if (pathTime >= steerTime && numPoints < CWL_MAX_PATH)
    {
      // Wheels should be aligned now.
      m_state.curvature = 0.;  
      m_state.crabAngle = 0;
    }
    cwlPushState();

    // max drive speed is a function of all commanded drive speeds since the last rover stop.
    resetMaxDriveSpeed();

    return getPpmSpeeds().size();
}

// parameters are target variables which are not used to achieve other targets 
unsigned int 
FastestSteerAndDriveCWL::appendPath(double pathTime, double targetWheelSpeed, double targetCurvature, double targetCrab)
  {
    if (isnan(m_state.curvature)) {
      // cannot start moving until wheels are aligned.
      return 0;
    }

    bool stopAtNodalPt = false;
    if (isnan(targetCurvature) || isnan(targetCrab)) {
      targetCurvature = m_state.curvature;
      targetCrab = m_state.crabAngle;
      stopAtNodalPt = true;
    }

    CwlVector targetWheelAngleForAllWheels(m_numWheels);
    CwlVector targetWheelSpeedRatios(m_numWheels);
    setMaxDriveSpeed(targetWheelSpeed);
    m_driveProfile.setLimits(targetWheelSpeed, m_wheelAcceleration);

    // Target steer variables
    getWheelSteerVariables(targetWheelAngleForAllWheels.begin(), targetWheelSpeedRatios.begin(), targetCurvature, targetCrab);

    bool const computePathTime = isnan(pathTime);
    unsigned int const numPathPoints = maxPoints(pathTime);

    CwlVector wheelSpeeds(m_numWheels);
    CwlVector wheelSteerAngles(m_numWheels);
    getWheelVariables(wheelSpeeds.begin(), wheelSteerAngles.begin(), m_wheelSpeedRatios.begin(),
                      m_state.speed / m_wheelRadius, m_state.curvature, m_state.crabAngle);

    // Ratio of fastest wheel speed to rover speed.
    double driveSpeedRatio;
    unsigned int fastestSteerWheelIndex;
    unsigned int fastestDriveWheelIndex;
    double steerRate = NAN;
    double wheelSpeed = NAN;

    unsigned int lastFastestSteerWheelIndex = m_numWheels + 1;
    double steerTime = 0.;

    unsigned int lastFastestDriveWheelIndex = m_numWheels + 1;
    double driveTime = 0.;

#ifdef CHECK_CONT_ACCEL_LIMIT
    double steerUnderSpeed = 0.;
    double steerOffsetError = 0.;

    double wheelUnderSpeed = 0.;
    double wheelOffsetError = 0.;
#endif

    //    double initialCrab = m_state.crabAngle;
    //    double deltaCrab = targetCrab - initialCrab;

    for (unsigned int i = 0; i < numPathPoints; ++i) {
      double const lastCurvature = m_state.curvature;
      double const lastSpeed = m_state.speed;

      // TODO: linear change in crab angle.  This will be optimized later.
      //      m_state.crabAngle = initialCrab + deltaCrab * static_cast<float>(i)/(numPathPoints-1);
      //      updateCrabAngle(m_state.crabAngle);
      
      // determine limiting steer and drive motors
      getFastestSteerWheelIndex(targetCurvature > 0., fastestSteerWheelIndex, fastestDriveWheelIndex, driveSpeedRatio);

      double steerPosition = m_state.motorPositions[m_numWheels + fastestSteerWheelIndex]; 
      double const targetSteerPosition = targetWheelAngleForAllWheels[fastestSteerWheelIndex];
      double wheelPosition = m_state.motorPositions[fastestDriveWheelIndex]; 

      if (i == 0) {
        steerRate = m_state.motorSpeeds[m_numWheels + fastestSteerWheelIndex];
        wheelSpeed = m_state.motorSpeeds[fastestDriveWheelIndex] * m_wheelRadius;
      }

      if (fastestSteerWheelIndex != lastFastestSteerWheelIndex) {
        lastFastestSteerWheelIndex = fastestSteerWheelIndex;
	try {
	  m_steerProfile.initializeState(
              steerPosition, steerRate, targetSteerPosition, stopAtNodalPt);
	} catch (const std::logic_error& logicError) {
	  std::cerr << "Steer Profile Path: " << logicError.what() << std::endl;
	  throw logicError;
	}
#ifdef ENABLE_FSD_CRABBING
        try {
	  m_crabProfile.initializeState(
              m_state.crabAngle, m_state.crabRate, targetCrab, stopAtNodalPt);
	} catch (const std::logic_error& logicError) {
	  std::cerr << "Crab Profile Path: " << logicError.what() << std::endl;
	  throw logicError;
	}
#endif
        steerTime = 0.;
      }

      steerTime += m_sampleInterval;
      m_steerProfile.computeState(steerTime, steerPosition, steerRate);
#ifdef ENABLE_FSD_CRABBING
      m_crabProfile.computeState(steerTime, m_state.crabAngle, m_state.crabRate);
#else
      m_state.crabAngle = 0.0;
#endif
      updateCrabAngle(m_state.crabAngle);

      // update curvature and curvature-rate
      m_state.curvature = getCurvatureFromTurnAngleAndCrab(fastestSteerWheelIndex, steerPosition, m_state.crabAngle);


#ifdef CHECK_CONT_ACCEL_LIMIT
      double steerAccel = m_steerProfile.acceleration(steerTime);
      double steerAccelOW = oppositeWheelSteerAccel(fastestSteerWheelIndex, steerAccel);
      double steerAccelExceed = fabs(steerAccelOW) - m_targetSteerAcceleration;
      
      if (steerAccelExceed > MEPSILON) {
        steerUnderSpeed += steerAccelExceed;
        steerOffsetError += steerUnderSpeed;
      }
#endif // CHECK_CONT_ACCEL_LIMIT

      m_state.motorSpeeds[m_numWheels + fastestSteerWheelIndex] = steerRate;
      setCurvatureRateWheelState(fastestSteerWheelIndex);
      
      getDriveSpeedRatiosAndWheelAnglesFromCurvatureAndCrab(
        wheelSteerAngles.begin(), m_wheelSpeedRatios.begin(), m_state.curvature, m_state.crabAngle);
      
      m_state.motorSpeeds[fastestDriveWheelIndex] = wheelSpeed / m_wheelRadius;
      setCurvatureRateWheelState(fastestSteerWheelIndex);
      
      if (fastestDriveWheelIndex != lastFastestDriveWheelIndex) {
        lastFastestDriveWheelIndex = fastestDriveWheelIndex;
        m_driveProfile.initializeState(wheelPosition, wheelSpeed);
        driveTime = 0.;
      }

      driveTime += m_sampleInterval;
      m_driveProfile.computeState(driveTime, wheelPosition, wheelSpeed);
      m_state.speed = wheelSpeed / m_wheelSpeedRatios[fastestDriveWheelIndex];

      getWheelSpeedsFromRoverSpeedAndWheelSpeedRatios(
        wheelSpeeds.begin(), m_wheelSpeedRatios.begin(), m_state.speed / m_wheelRadius);

#ifdef CHECK_CONT_ACCEL_LIMIT
      double wheelAccel = m_driveProfile.acceleration(driveTime);
      double wheelAccelOW = oppositeWheelDriveAccel(fastestDriveWheelIndex, wheelAccel);
      double wheelAccelExceed = fabs(wheelAccelOW) - m_wheelAcceleration;
      
      if (wheelAccelExceed > MEPSILON) {
        wheelUnderSpeed += wheelAccelExceed;
        wheelOffsetError += wheelUnderSpeed;
      }
#endif // CHECK_CONT_ACCEL_LIMIT

      for (unsigned int j = 0; j < m_numWheels; ++j) {
        setSteerRateWheelState(j);
        m_state.motorSpeeds[j] = wheelSpeeds[j];

        double deltaWheelAngle = wheelSpeeds[j] * m_sampleInterval;
        m_state.motorPositions[j] += deltaWheelAngle;
        m_state.motorPositions[m_numWheels + j] = wheelSteerAngles[j];
      }

      // save state sample
      cwlPushState();
      
      // if no time specified, exit once target curvature/speed is reached
      if (computePathTime && 
          (fabs(m_state.curvature - lastCurvature) < MEPSILON &&
           fabs(m_state.speed - lastSpeed) < MEPSILON)) {
        break;
      }
    }

#ifdef CHECK_CONT_ACCEL_LIMIT
    if (steerOffsetError > MEPSILON) {
      // Assume acceleration capped at maximum acceleration,
      // and compute steer angle error.
      double steerOffsetErrorInDegrees = (180. / M_PI / 2.) * steerOffsetError * m_sampleInterval * m_sampleInterval;
      cout << "Steer Offset Error: " << steerOffsetErrorInDegrees << " degrees" << endl;
    }

    if (wheelOffsetError > MEPSILON) {
      // Assume acceleration capped at maximum acceleration,
      // and compute wheel distance.
      double wheelOffsetErrorMeters = wheelOffsetError * m_sampleInterval * m_sampleInterval / 2.;
      cout << "Wheel Offset Error: " << wheelOffsetErrorMeters << " meters" << endl;
    }
#endif

    return getPpmSpeeds().size();
  }

  void 
  FastestSteerAndDriveCWL::getFastestSteerWheelIndex(const bool& rightTurn, unsigned int& fastestSteerWheelIndex, 
                                                     unsigned int& fastestDriveWheelIndex, 
                                                     double& driveSpeedRatio) const
  {
    // Assume that closest wheel to turn center is fastest steering, and furthest is fastest driving.
    // This may not be true for fastest steering wheel if crab angle is changing very quickly.

    // all speeds equal, that's not necessarily returning the right wheel...

    
    if (fabs(m_state.curvature) > MEPSILON) {
      CwlVector::const_iterator minI = min_element(m_wheelSpeedRatios.begin(), m_wheelSpeedRatios.end());
      CwlVector::const_iterator maxI = max_element(m_wheelSpeedRatios.begin(), m_wheelSpeedRatios.end());

      fastestDriveWheelIndex = distance(m_wheelSpeedRatios.begin(), maxI);
      fastestSteerWheelIndex = distance(m_wheelSpeedRatios.begin(), minI);
      driveSpeedRatio = 1./ (*maxI);
    } else {
      // Special case when all wheels straight -- look at direction we
      // are going to turn so that fastest wheel does not change.
      driveSpeedRatio = 1.0;
      if (rightTurn) {
        fastestSteerWheelIndex = 1;
        fastestDriveWheelIndex = 0;
      } else {
        fastestSteerWheelIndex = 0;
        fastestDriveWheelIndex = 1;
      }
    }
  }

  //---------------  DEBUG FUNCTIONS ------------------

  namespace {
    std::string addMarker(int index,  
                          int excessDriveDelta, int excessSteerDelta,
                          int excessDriveSpeed, int excessSteerSpeed,
                          int excessDriveAcc, int excessSteerAcc) 
    {
      string s;
      
      if (index == excessDriveDelta)
        s += "dD";
      if (index == excessSteerDelta)
        s += "sD";
      if (index == excessDriveSpeed)
        s += "dS";
      if (index == excessSteerSpeed)
        s += "sS";
      if (index == excessDriveAcc)
        s += "dA";
      if (index == excessSteerAcc)
        s += "sA";

      if (!s.empty())
        s += " ";
      return s;
    }
  }

  bool
  FastestSteerAndDriveCWL::printWarnings(std::ostream& ostr, bool printDebug)
  {
    DoubleVectorVector const& ppmMotorPositions = getPpmMotorPositions();
    DoubleVectorVector const& ppmMotorSpeeds =  getPpmMotorSpeeds();
    DoubleVector const& ppmCurvatures = getPpmCurvatures();
    DoubleVector const& ppmCurvatureRates = getPpmCurvatureRates();
    DoubleVector const& ppmSpeeds = getPpmSpeeds();
    
    // analysis: check for excess speed/acc
    int excessDriveDelta = -1;
    int excessSteerDelta = -1;
    int excessDriveSpeed = -1;
    int excessSteerSpeed = -1;
    int excessDriveAcc = -1;
    int excessSteerAcc = -1;

    double driveDeltaE = 0.;
    double steerDeltaE = 0.;
    double driveSpeedE = 0.;
    double steerSpeedE = 0.;
    double driveAccE = 0.;
    double steerAccE = 0.;
    
    for (unsigned int i = 0; i < ppmMotorPositions.size(); ++i) {
      for (unsigned int j = 1; j < ppmMotorPositions[i].size(); ++j) {
        if (i < 4) {
          double const driveDelta = fabs(ppmMotorPositions[i][j] - ppmMotorPositions[i][j - 1]);
          if ((driveDelta * m_sampleFrequency * m_wheelRadius - m_maxDriveSpeed) > LOC_EPSILON &&
              excessDriveDelta < 0) {
            excessDriveDelta = j;
            driveDeltaE = driveDelta;
          }
        }
        else {
          double const steerDelta = fabs(ppmMotorPositions[i][j] - ppmMotorPositions[i][j - 1]);
          if ((steerDelta * m_sampleFrequency - m_targetSteerRate) > LOC_EPSILON &&
              excessSteerDelta < 0) {
            excessSteerDelta = j;
            steerDeltaE = steerDelta;
          }
        }
      }
    }
    
    for (unsigned int i = 0; i < ppmMotorSpeeds.size(); ++i) {
      for (unsigned int j = 1; j < ppmMotorSpeeds[i].size(); ++j) {
        if (i < 4) {
          double const driveSpeed = fabs(ppmMotorSpeeds[i][j]);
          if ((driveSpeed * m_wheelRadius -  m_maxDriveSpeed) > LOC_EPSILON &&
              excessDriveSpeed < 0) {
            excessDriveSpeed = j;
            driveSpeedE = driveSpeed;
          }
        }
        else {
          double const steerSpeed = fabs(ppmMotorSpeeds[i][j]);
          if ((steerSpeed - m_targetSteerRate) > LOC_EPSILON &&
              excessSteerSpeed < 0) {
            excessSteerSpeed = j;
            steerSpeedE = steerSpeed;
          }
        }
        
        if (j > 0) {
          if (i < 4) {
            double const driveSpeedDelta = fabs(ppmMotorSpeeds[i][j] - ppmMotorSpeeds[i][j - 1]);
            if ((driveSpeedDelta * m_sampleFrequency * m_wheelRadius - m_wheelAcceleration) > LOC_EPSILON &&
                excessDriveAcc < 0) {
              excessDriveAcc = j;
              driveAccE = driveSpeedDelta;
            }
          }
          else {
            double const steerSpeedDelta = fabs(ppmMotorSpeeds[i][j] - ppmMotorSpeeds[i][j - 1]);
            if ((steerSpeedDelta * m_sampleFrequency - m_targetSteerAcceleration) > LOC_EPSILON &&
                excessSteerAcc < 0) {
              excessSteerAcc = j;
              steerAccE = steerSpeedDelta;
            }
          }
        }
      }
    }
    
    bool const excess =
      (excessDriveDelta >= 0 ||
       excessSteerDelta >= 0 ||
       excessDriveSpeed >= 0 ||
       excessSteerSpeed >= 0 ||
       excessDriveAcc >= 0 ||
       excessSteerAcc >= 0);
    
    if (printDebug || excess) {
      
      if (excessSteerDelta >= 0) {
        ostr << "steerDelta deg per sec=" << steerDeltaE * 180. / M_PI * m_sampleFrequency
             << " exceeds " <<  m_targetSteerRate * 180. / M_PI << endl;
        ostr << m_steerProfile;
      }
      if (excessSteerSpeed >= 0) {
        ostr << "steerSpeed deg per sec=" << steerSpeedE * 180. / M_PI * m_sampleFrequency
             << " exceeds " <<  m_targetSteerRate * 180. / M_PI << endl;
        ostr << m_steerProfile;
      }
      if (excessSteerAcc >= 0) {
        ostr << "steerAcc deg per sec*sec=" << steerAccE * 180. / M_PI * m_sampleFrequency 
             << " exceeds " <<  m_targetSteerAcceleration * 180. / M_PI << endl;     
        ostr << m_steerProfile;
      }
      
    
      if (excessDriveDelta >= 0) {
        ostr << "dirveDelta m/s=" << driveDeltaE * m_wheelRadius * m_sampleFrequency
             << " exceeds " <<  m_maxDriveSpeed << endl;
        ostr << m_driveProfile;
      }
      if (excessDriveSpeed >= 0) {
        ostr << "dirveSpeed m/s=" << driveSpeedE * m_wheelRadius
             << " exceeds " <<  m_maxDriveSpeed << endl;
        ostr << m_driveProfile;
      }
      if (excessDriveAcc >= 0) {
        ostr << "driveAcc m/s/s=" << driveAccE * m_wheelRadius * m_sampleFrequency 
             << " exceeds " <<  m_wheelAcceleration << endl;     
        ostr << m_driveProfile;
      }
      
      
      for (unsigned int i = 0; i < ppmMotorPositions.size(); ++i) {
        ostr << ppmMotorPositions[i].size() << "{";
        for (unsigned int j = 0; j < ppmMotorPositions[i].size(); ++j) {
          if (j != 0) {
            ostr << ", ";
          }
          ostr << addMarker(j,  
                            excessDriveDelta, excessSteerDelta,
                            excessDriveSpeed, excessSteerSpeed,
                            excessDriveAcc, excessSteerAcc);
          ostr << ppmMotorPositions[i][j] * 180. / M_PI;
        }
        ostr << "}" << endl;
      }
      
      ostr << "speeds: "  << endl;
      for (unsigned int i = 0; i < ppmMotorSpeeds.size(); ++i) {
        ostr << ppmMotorSpeeds[i].size() << "{";
        for (unsigned int j = 0; j < ppmMotorSpeeds[i].size(); ++j) {
          if (j != 0) {
            ostr << ", ";
          }
          ostr << addMarker(j,  
                            excessDriveDelta, excessSteerDelta,
                            excessDriveSpeed, excessSteerSpeed,
                            excessDriveAcc, excessSteerAcc);
          ostr << ppmMotorSpeeds[i][j] * 180. / M_PI;
        }
        ostr << "}" << endl;
      }
      
      
      ostr << "curvatures:" << ppmCurvatures.size() << endl;
      for (unsigned int i = 0; i < ppmCurvatures.size(); ++i) {
        if (i != 0)
          ostr << ", ";
        ostr << addMarker(i,  
                          excessDriveDelta, excessSteerDelta,
                          excessDriveSpeed, excessSteerSpeed,
                          excessDriveAcc, excessSteerAcc);
        ostr << ppmCurvatures[i];
      }
      ostr << endl;
      
      ostr << "curvature rates:" << ppmCurvatureRates.size() << endl;
      for (unsigned int i = 0; i < ppmCurvatureRates.size(); ++i) {
        if (i != 0)
          ostr << ", ";
        ostr << addMarker(i,  
                          excessDriveDelta, excessSteerDelta,
                          excessDriveSpeed, excessSteerSpeed,
                          excessDriveAcc, excessSteerAcc);
        ostr << ppmCurvatureRates[i];
      }
      ostr << endl;
      ostr << "speeds:" << ppmSpeeds.size() << endl;
      for (unsigned int i = 0; i < ppmSpeeds.size(); ++i) {
        if (i != 0)
          ostr << ", ";
        ostr << addMarker(i,  
                          excessDriveDelta, excessSteerDelta,
                          excessDriveSpeed, excessSteerSpeed,
                          excessDriveAcc, excessSteerAcc);
        ostr << ppmSpeeds[i];
      }
      ostr << endl;
    }

    return excess;
  }
  
  void 
  FastestSteerAndDriveCWL::resetMaxDriveSpeed() {
    m_maxDriveSpeed = 0.;
  }
  void 
  FastestSteerAndDriveCWL::setMaxDriveSpeed(double speed) {
    m_maxDriveSpeed = max(m_maxDriveSpeed,fabs(speed));
  }

}
