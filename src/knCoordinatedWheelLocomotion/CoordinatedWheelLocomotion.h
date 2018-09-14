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

#ifndef kn_CoordinatedWheelLocomotion_h
#define kn_CoordinatedWheelLocomotion_h

#include "knCoordinatedWheelLocomotion_Export.h"

#include "knMotorShare/WheelGroupState.h"

#include "knMath/ATrans.h"

#include <vector>

namespace kn 
{
  class knCoordinatedWheelLocomotion_Export CoordinatedWheelLocomotion
  {
  public:
    typedef std::vector<double> CwlVector;
    //typedef vw::Vector4 CwlVector;
    typedef std::vector<CwlVector> CwlArray;

    /**
       This base class provides core functionality for converting a set of parameterized rover path constraints to a
       set of wheel values which satisfy those constraints.  An interface is provided to concatenate multiple paths
       into a single sequence of wheel values.  In addition to the wheel values, the corresponding estimated rover
       path can also be computed as a sequence of {x,y,crab angle} points.  All calculations assume a flat earth
       model with perfect friction (no wheel slip), with no error correction when straying off the path due to
       non-idealized conditions.  These corrections are assumed to occur at a higher level inside a navigation loop.

       The full implementation which determines the types of wheel constraints and how they are interpreted is done
       in the derived classes.  These locomotion types could include, for example: a constant curvature rate
       clothoid; a path which achieves a target curvature and speed as fast as possible; a path which is limited by
       acceleration forces on the rover, etc.  These implementations should have the following features in common:

       1) There can be an arbitrary number of steerable wheels, at arbitrary locations.  No other wheel types are
       supported.  The derived class specifies the number of wheels and their locations.

       2) Each wheel can steer at least +/- 180 degrees relative to the orientation when driving straight.

       3) Each wheel can be driven forward or backward, up to a globally defined maximum speed.
           
       4) Other global wheel limits may be defined as needed by the derived class implementation (such as max steer
       acceleration, for example).

       5) Paths are specified by an initial state, a target state, and a transition.  

       i) The initial state refers to the complete state information at the beginning of the requested path.
       These state variables depend on the locomotion type of the derived class, and for example, may include
       the curvature and speed of the rover, and may also include other information such as steer speed, steer
       acceleration, rate of change of centripetal force, jerk, etc.

       ii) The final state refers to the target values which will be attempted to be reached.  Each derived class
       chooses these variables, and for example, may include the target curvature, speed, allotted time, path
       length, etc.  Note that there is no requirement for the final state to include all variables from the
       initial state, or visa versa.

       iii) The transition is a parameterized description of how the rover will go from the initial state to the
       final state.  For example, This could be a clothoid path with a fixed curvature rate and constant speed
       change; it could push the limits of the hardware; etc.

       iv) The final state may not always be reachable, and thus it is up to the derived class to decide the
       "closest path".  For example, perhaps the speed and curvature cannot be simultaneously met during a tight
       turn, and the derived class chooses to meet the target curvature at the fastest speed possible.

       v) The variables in the initial state, final state, and parameterized transition are all differentiable
       constraints on the motor states.  For example, a given wheel angle configuration may result in a
       given curvature over flat ground, and a different curvature over uneven terrain.  Only the idealized flat
       ground curvature can be used as a constraint.  As another example, a constraint such as an x,y position
       relative to the start of the path is not a valid constraint because it requires the integration of the
       wheel positions and angles.
              
       3) The start state includes the starting speed, curvature, crab angle, and additional state information
       specific to the derived class.  This additional derived class specific state information is used to ensure
       continuity between paths.  For example, this could include: drive or steer accelerations or jerks (rate of
       change of acceleration), rate of change of centripetal force, etc.

       4) A smooth slowdown and wheel straightening segment is automatically concatenated at the end of the path.
       Typically this slowdown segment will not be executed since the path will generally be interrupted with a new
       path as the rover obtains updated situational awareness in a higher level navigation loop.  This smooth
       slowdown segment is specified by the derived class.
    */
    CoordinatedWheelLocomotion(double sampleFrequency, double wheelRadius,
                               Vector2Vector const& wheelLocations,
                               unsigned int maxPath = CWL_MAX_PATH);

    // accessor methods

    unsigned int numWheels() const throw() { return m_numWheels; }
    double wheelRadius() const throw() { return m_wheelRadius; }

    CwlArray const& getPpmMotorPositions() const throw() { return m_ppmMotorPositions; }
    CwlArray const& getPpmMotorSpeeds() const throw() { return m_ppmMotorSpeeds;}
    CwlVector const& getPpmCurvatures() const throw() { return m_ppmCurvatures;}
    CwlVector const& getPpmCurvatureRates() const throw() { return m_ppmCurvatureRates; }
    CwlVector const& getPpmSpeeds() const throw() { return m_ppmSpeeds;}
    CwlVector const& getPpmCrabs() const throw() { return m_ppmCrabs;}
    CwlVector const& getPpmCrabRates() const throw() { return m_ppmCrabRates;}

    double getSampleFrequency() const throw() { return m_sampleFrequency;}

    // This is probably only useful for generating multiple-fidelity trajectories for path prediction.
    void setSampleFrequency(double sampleFrequency) throw() {m_sampleFrequency = sampleFrequency; m_sampleInterval = 1./sampleFrequency; }
    double getElapsedTime() const throw() { return m_ppmSpeeds.size() * m_sampleInterval; }


    void renderTrajectory(ATrans2 const& pose, ATrans2Vector& path) const;

    void resizePath(unsigned int newSize);
    void clearPath();
    void initPath(WheelGroupState const& state);


  protected:
    void updateCrabAngle(double crabAngle) throw();
    void cwlPushState();

    void getWheelVariables(CwlVector::iterator wheelSpeed, CwlVector::iterator wheelAngle, CwlVector::iterator wheelSpeedRatio, 
                           const double speed, const double curvature, const double crab) const;
    void getWheelSteerVariables(CwlVector::iterator wheelAngle, CwlVector::iterator wheelSpeedRatio, 
                           const double curvature, const double crab) const;

    void getDriveSpeedRatiosAndWheelAnglesFromCurvatureAndCrab(
      CwlVector::iterator wheelAngle, CwlVector::iterator wheelSpeedRatio, const double curvature, const double crab) const;

    void getWheelSpeedsFromRoverSpeedAndWheelSpeedRatios(
      CwlVector::iterator wheelSpeed, CwlVector::const_iterator wheelSpeedRatio, const double speed) const;

    double getCurvatureFromTurnAngleAndCrab(int wheelIndex, double turnAngle, double crabAngle) const;

    void setCurvatureRateWheelState(const int wheelIndex);
    void setSteerRateWheelState(const int wheelIndex);
    
    void fillPpm(unsigned int maxPoints);

    static void updateVariable(double& x, 
                               double xDot, double deltaT, double xf) throw();

    double oppositeWheelSteerAccel(unsigned wheelIndex, double steerAccel);
    double oppositeWheelDriveAccel(unsigned wheelIndex, double driveAccel);

  protected:
    void reservePpm(unsigned int maxPoints);

    unsigned int const m_numWheels;

    double m_sampleFrequency;
    double m_sampleInterval; //!< 1/m_sampleFrequency
    double const m_wheelRadius;
    //! Location of the wheels
    Vector2Vector const m_wheelLocations;
    //! Locations of the wheels rotated by -crabAngle
    Vector2Vector m_wheelCrabLocations;

    CwlVector m_wheelSpeedRatios;

    WheelGroupState m_state;

    static const double MEPSILON;
    static const unsigned int CWL_MAX_PATH = 4069;

  private:
    CwlArray m_ppmMotorPositions;
    CwlArray m_ppmMotorSpeeds;
    CwlVector m_ppmSpeeds;
    CwlVector m_ppmCurvatures;
    CwlVector m_ppmCurvatureRates;
    CwlVector m_ppmCrabs;
    CwlVector m_ppmCrabRates;
  };

  inline
  void 
  CoordinatedWheelLocomotion::updateVariable(double& x, 
                                             double xDot, double deltaT, double xf) throw()
  {
    assert (xDot >= 0.);

    double d = xf - x;
    double dX = xDot * deltaT;

    if (fabs(d) > dX)  { 

      if ( d < 0.) {
        dX = -dX;
      }

      x += dX;
    }
    else {
      x = xf;
    }
  }
}

#endif // kn_CoordinatedWheelLocomotion_h
