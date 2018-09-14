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

#include "CoordinatedWheelLocomotion.h"

#include "knMath/Matrix.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace kn 
{
  using namespace std;

  double const CoordinatedWheelLocomotion::MEPSILON = 1E-6;
  const unsigned int CoordinatedWheelLocomotion::CWL_MAX_PATH;

  CoordinatedWheelLocomotion::CoordinatedWheelLocomotion(double sampleFrequency, double wheelRadius, 
                                                         Vector2Vector const& wheelLocations,
                                                         unsigned int maxPath) :
    m_numWheels(wheelLocations.size()), 
    m_sampleFrequency(sampleFrequency), 
    m_sampleInterval(1. / sampleFrequency), 
    m_wheelRadius(wheelRadius), 
    m_wheelLocations(wheelLocations),
    m_wheelCrabLocations(wheelLocations),
    m_wheelSpeedRatios(wheelLocations.size()),
    m_state(ACE_Time_Value::zero, 
            CwlVector(m_numWheels * 2, 0.), 
            CwlVector(m_numWheels * 2, 0.), 
            0., 0., 0., 0., 0.),
    m_ppmMotorPositions(2*m_numWheels),
    m_ppmMotorSpeeds(2*m_numWheels)
  {
    assert(m_numWheels > 0);

    // Reserve all arrays
    for (unsigned int i = 0; i < m_ppmMotorPositions.size(); ++i) {
      m_ppmMotorPositions[i].reserve(maxPath);
      m_ppmMotorSpeeds[i].reserve(maxPath);
    }
    m_ppmSpeeds.reserve(maxPath);
    m_ppmCurvatures.reserve(maxPath);
    m_ppmCurvatureRates.reserve(maxPath);
    m_ppmCrabs.reserve(maxPath);
    m_ppmCrabRates.reserve(maxPath);
  }

  void
  CoordinatedWheelLocomotion::reservePpm(unsigned int maxPoints)
  {
    for (unsigned int i = 0; i < m_numWheels; ++i) {
      m_ppmMotorPositions[i].reserve(maxPoints);
      m_ppmMotorSpeeds[i].reserve(maxPoints);
    }
  }
  
  void
  CoordinatedWheelLocomotion::fillPpm(unsigned int maxPoints)
  {
    // fill the vectors up with current-positions, so that they are of equal length:
    for (unsigned int i = 0; i < m_numWheels; ++i) {
      double driveFill = m_state.motorPositions[i];
      double steerFill = m_state.motorPositions[m_numWheels + i];
        
      m_ppmMotorPositions[i].resize(maxPoints, driveFill);
      m_ppmMotorPositions[i + m_numWheels].resize(maxPoints, steerFill);
      m_ppmMotorSpeeds[i].resize(maxPoints, 0.);
      m_ppmMotorSpeeds[i + m_numWheels].resize(maxPoints, 0.);
    }
  }

  void
  CoordinatedWheelLocomotion::renderTrajectory(ATrans2 const& pose, ATrans2Vector& path) const
  {
    path.resize(m_ppmCurvatures.size());

    ATrans2 p = pose;
    CwlVector::const_iterator curvature, curvatureLast = m_ppmCurvatures.end();
    CwlVector::const_iterator crab = m_ppmCrabs.begin();
    CwlVector::const_iterator crabRate = m_ppmCrabRates.begin();
    CwlVector::const_iterator speed = m_ppmSpeeds.begin();
    unsigned int i = 0;
    for (curvature = m_ppmCurvatures.begin(); curvature != curvatureLast;
         ++curvature, ++crab, ++crabRate, ++speed, ++i) {
      path[i] = step(p, *curvature, *crab, *crabRate, *speed, m_sampleInterval);
      p = path[i];
    }
  }

  void 
  CoordinatedWheelLocomotion::initPath(WheelGroupState const& state) 
  {
    assert (state.motorPositions.size() == m_numWheels * 2 &&
            state.motorSpeeds.size() == m_numWheels * 2);

    updateCrabAngle(state.crabAngle);
    
    m_state = state;
  }
  
  void 
  CoordinatedWheelLocomotion::clearPath()
  {
    // Reset all arrays
    for (unsigned int i = 0; i < m_ppmMotorPositions.size(); ++i) {
      m_ppmMotorPositions[i].clear();
      m_ppmMotorSpeeds[i].clear();
    }
    m_ppmSpeeds.clear();
    m_ppmCurvatures.clear();
    m_ppmCurvatureRates.clear();
    m_ppmCrabs.clear();
    m_ppmCrabRates.clear();
  }
  
  void 
  CoordinatedWheelLocomotion::resizePath(unsigned int newSize) 
  {
    if (newSize == 0) {
      clearPath();
    }
    else {
      assert(newSize < m_ppmMotorSpeeds[0].size());
      
      // Reset all arrays 
      for (unsigned int i = 0; i < m_ppmMotorPositions.size(); ++i) {
        m_ppmMotorPositions[i].resize(newSize);
        m_ppmMotorSpeeds[i].resize(newSize);
      }
      m_ppmSpeeds.resize(newSize);
      m_ppmCurvatures.resize(newSize);
      m_ppmCurvatureRates.resize(newSize);
      m_ppmCrabs.resize(newSize);
      m_ppmCrabRates.resize(newSize);
      
      // initialize state from last sampled state
      for (unsigned int i = 0; i < m_state.motorPositions.size(); ++i) {
        m_state.motorPositions[i] = m_ppmMotorPositions[i].back();
        m_state.motorSpeeds[i] = m_ppmMotorSpeeds[i].back();
      }
      m_state.speed = m_ppmSpeeds.back();
      m_state.curvature = m_ppmCurvatures.back();
      m_state.curvatureRate = m_ppmCurvatureRates.back();
      
      m_state.crabAngle = m_ppmCrabs.back();
      m_state.crabRate = m_ppmCrabRates.back();
      updateCrabAngle(m_ppmCrabs.back());
    }
  }

  void 
  CoordinatedWheelLocomotion::cwlPushState() 
  {
    m_ppmSpeeds.push_back(m_state.speed);
    m_ppmCurvatures.push_back(m_state.curvature);
    m_ppmCurvatureRates.push_back(m_state.curvatureRate);
    m_ppmCrabs.push_back(m_state.crabAngle);
    m_ppmCrabRates.push_back(m_state.crabRate);

    for (unsigned int i = 0; i < m_state.motorPositions.size(); ++i) {
      m_ppmMotorPositions[i].push_back(m_state.motorPositions[i]);
      m_ppmMotorSpeeds[i].push_back(m_state.motorSpeeds[i]);
    }
  }

  void 
  CoordinatedWheelLocomotion::updateCrabAngle(double crabAngle) throw()
  {
      Eigen::Rotation2D<double> crabRotation(-crabAngle);
      
      Vector2Vector::const_iterator wheelLocation = m_wheelLocations.begin();
      Vector2Vector::iterator first, last = m_wheelCrabLocations.end();
      for (first = m_wheelCrabLocations.begin(); first != last; ++first, ++wheelLocation) {
        *first = crabRotation * (*wheelLocation);
      }
      m_state.crabAngle = crabAngle;
  }

  void
  CoordinatedWheelLocomotion::getWheelVariables(CwlVector::iterator wheelSpeed, CwlVector::iterator wheelAngle, CwlVector::iterator wheelSpeedRatio,
                                                const double speed, const double curvature, const double crab) const
  {
    Vector2Vector::const_iterator first, last = m_wheelCrabLocations.end();
    for (first = m_wheelCrabLocations.begin(); 
         first != last; 
         ++first, ++wheelSpeed, ++wheelAngle, ++wheelSpeedRatio) {
      
      //        Vector2 wheelRel = center - m_wheelLocations[j];
      //        wheelRel *= copysign(1., curvature); 
      //        wheelSpeed[j] = speed * norm_2(wheelRel) * fabs(curvature);
      //        wheelTurnAngle[j] = atan2(-wheelRel.x(), wheelRel.y()); 
      
      Vector2 wheelRel = (*first) * curvature;
      wheelRel.y() -= 1.;
      *wheelSpeedRatio = wheelRel.norm();
      *wheelSpeed = speed * (*wheelSpeedRatio);
      *wheelAngle = crab + atan2(wheelRel.x(), -wheelRel.y());
    }
  }

  void
  CoordinatedWheelLocomotion::getWheelSteerVariables(CwlVector::iterator wheelAngle, CwlVector::iterator wheelSpeedRatio,
                                                     const double curvature, const double crab) const
  {
    Vector2Vector::const_iterator first, last = m_wheelCrabLocations.end();
    for (first = m_wheelCrabLocations.begin(); 
         first != last; 
         ++first, ++wheelAngle, ++wheelSpeedRatio) {
      
      Vector2 wheelRel = (*first) * curvature;
      wheelRel.y() -= 1.;
      *wheelSpeedRatio = wheelRel.norm();
      *wheelAngle = crab + atan2(wheelRel.x(), -wheelRel.y()); 
    }
  }

  void
  CoordinatedWheelLocomotion::getDriveSpeedRatiosAndWheelAnglesFromCurvatureAndCrab(
    CwlVector::iterator wheelAngle, CwlVector::iterator wheelSpeedRatio, const double curvature, const double crab) const
  {
    Vector2Vector::const_iterator first, last = m_wheelCrabLocations.end();
    for (first = m_wheelCrabLocations.begin(); 
         first != last; 
         ++first, ++wheelAngle, ++wheelSpeedRatio) {
      
      Vector2 wheelRel = (*first) * curvature;
      wheelRel.y() -= 1.;
      *wheelSpeedRatio = wheelRel.norm();
      *wheelAngle = crab + atan2(wheelRel.x(), -wheelRel.y()); 
    }
  }

  void
  CoordinatedWheelLocomotion::getWheelSpeedsFromRoverSpeedAndWheelSpeedRatios(
    CwlVector::iterator wheelSpeed, CwlVector::const_iterator wheelSpeedRatio, const double speed) const
  {
    Vector2Vector::const_iterator first, last = m_wheelCrabLocations.end();
    for (first = m_wheelCrabLocations.begin(); 
         first != last; 
         ++first, ++wheelSpeed, ++wheelSpeedRatio) {
      
      *wheelSpeed = speed * (*wheelSpeedRatio);
    }
  }

  double 
  CoordinatedWheelLocomotion::getCurvatureFromTurnAngleAndCrab(int wheelIndex, double steerAngle, double crabAngle) const
  {
    const double& wx = m_wheelCrabLocations[wheelIndex][0];
    const double& wy = m_wheelCrabLocations[wheelIndex][1];

    double angle = steerAngle - crabAngle;
    double sa = sin(angle);
    double ca = cos(angle);

    return sa / (sa * wy + ca * wx);
  }

  void
  CoordinatedWheelLocomotion::setCurvatureRateWheelState(const int wheelIndex)
  {
    const double& wx = m_wheelCrabLocations[wheelIndex][0];
    const double& wy = m_wheelCrabLocations[wheelIndex][1];
    
    const double xFact = m_state.curvature * wx;
    const double yFact = (1. - m_state.curvature * wy);

    m_state.curvatureRate = ((m_state.motorSpeeds[wheelIndex + m_numWheels] - m_state.crabRate) * (xFact * xFact + yFact * yFact) + m_state.crabRate * yFact) / wx;
  }

  void
  CoordinatedWheelLocomotion::setSteerRateWheelState(const int wheelIndex)
  {
      const double wx = m_wheelCrabLocations[wheelIndex][0];
      const double wy = m_wheelCrabLocations[wheelIndex][1];

      const double xFact = m_state.curvature * wx;
      const double yFact = (1. - m_state.curvature * wy);

      m_state.motorSpeeds[wheelIndex + m_numWheels] = m_state.crabRate + (m_state.curvatureRate * wx - m_state.crabRate * yFact) / (xFact * xFact + yFact * yFact);
  }

  double
  CoordinatedWheelLocomotion::oppositeWheelSteerAccel(unsigned wheelIndex, double steerAccel) 
  {
    double oppositeWheelSteerAccel = steerAccel;

    if (fabs(m_state.curvature) > MEPSILON)  {
      double x = m_state.curvature * m_wheelCrabLocations[wheelIndex][0];
      double y = 1 - m_state.curvature * m_wheelCrabLocations[wheelIndex][1];
      double d = x*x + y*y;

      double curvatureAccel = m_state.curvature * d * steerAccel / x 
        + (2 * m_state.curvatureRate * m_state.curvatureRate / m_state.curvature) * (1 - y / d);

      // Transformation for opposite wheel.
      d = d + 4 * (1 - y);
      y = 2 - y;

      oppositeWheelSteerAccel = x / m_state.curvature / m_state.curvature / d * (m_state.curvature * curvatureAccel 
            + 2 * m_state.curvatureRate * m_state.curvatureRate * (y/d - 1));
    }

    return oppositeWheelSteerAccel;
  }

  double
  CoordinatedWheelLocomotion::oppositeWheelDriveAccel(unsigned wheelIndex, double wheelAccel) 
  {
    double oppositeWheelAccel = wheelAccel;

    if (fabs(m_state.curvature) > MEPSILON)  {
      double x = m_state.curvature * m_wheelCrabLocations[wheelIndex][0];
      double y = 1 - m_state.curvature * m_wheelCrabLocations[wheelIndex][1];
      double d = x*x + y*y;
      double p = sqrt(d);

      double curvatureRatio = m_state.curvatureRate / m_state.curvature * m_state.speed;

      double roverAccel = wheelAccel / p + curvatureRatio * (1 - y / d);

      // Transformation for opposite wheel.
      d = d + 4 * (1 - y);
      p = sqrt(d);
      y = 2 - y;

      oppositeWheelAccel = p * roverAccel + curvatureRatio * (p - y / p);
    }

    return oppositeWheelAccel;
  }
}  


