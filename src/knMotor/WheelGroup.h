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

#ifndef kn_WheelGroup_h
#define kn_WheelGroup_h

#include "MotorGroup.h"
#include "knMotor_Export.h"

#include <ace/Time_Value.h>

namespace kn
{
  class WheelGroupParameters;
  class WheelGroupSample;
  class WheelGroupFuture;
  class WheelParameters;

  //! Abstract wheel group interface.
  /**
   * Syncronized control of a set of wheels with drive and steer motors.
   * The primary customer for this interface is the implementation of the Locomotor IDL interface.
   */
  class knMotor_Export WheelGroup : public virtual MotorGroup
  {
  public:
    typedef std::vector<WheelParameters const *> WheelParametersVector;

    //! Dtor. This method does not throw.
    virtual ~WheelGroup() throw();

    //! Number of wheels in the wheel group.
    virtual unsigned int numWheels() const throw() = 0;

    //! Accessor to the parameters struct.
    virtual WheelParameters const * wheelParams(unsigned int index) const throw() = 0;
    //! Set parameters.
    /** The class does not take ownership of the parameters. */
    virtual void setWheelParams(unsigned int index, WheelParameters const * params) throw() = 0;

    // avoid warning on hidden virtual method.
    using MotorGroup::cmdProfiles;
    //! Set the command profiles for all drive and steer motors and start motion.
    /**
     * While executing a command profile the cmdStatus is KNMOTOR_BUSY.
     * Only a stop command can interrupt a command profile execution.
     *
     * @param profiles The MotorProfile data structure specifies all parameters of the
     * command.
     *  - ctrlMode - The allowed control modes for a cmdProfile are CTRL_POSITION ans CTRL_TRAPECOID.
     *  - posMode - Can be either POS_ABS or POS_REL
     *  - position - Needs to specify a valid motor position (rad).
     *  - speed - In CTRL_TRAPECOID, specifies maximum motor speed.
     *  - acc - In CTRL_TRAPECOID, specifies maximum motor accelearion.
     *
     * @param curvature - The curvature the chassis of the robot is describing.
     *  fabs(curvature) >= 1000 is considered a point-turn.
     *   NAN, if wheels are not describing a curvature.
     * @param crabAngle - The crab-angle the chassis of the robot is moving relative to.
     */
    virtual int cmdProfiles(MotorProfileVector const& profiles,
                            double curvature, double crabAngle) = 0;

    // avoid warning on hidden virtual method.
    using MotorGroup::cmdTrajectories;
    //! Set the trajectories for all drive and steer motors and start motion.
    /**
     * While executing a trajectory, the cmdStatus is KNMOTOR_BUSY_INTERRUPTABLE.
     * Trajectory commands can be interrupted by subsequent trajectory commands.
     *
     * @param ref The targeted index at wich to overwrite the current trajectory vector.
     * If that targeted index can no longer be reached, KNMOTOR_ERR_TIMEOUT is returned.
     * @param frequency The trajectory is sampled at a fixed frequency. The frequency has to be
     * supported by the motor, otherwise KNMOTOR_ERR_PARAM_RANGE is returned.
     * @param positions Vector of absolute motor positions for each motor in rad.
     * @param speeds Vector of motor speeds in rad/s for each motor.
     *
     * Either the positions or speeds vectors have to be of non-zero size.
     * The non-zero positions and speeds vectors  size has to match numMotors().
     *
     * Either the individual positions or speeds vectors for each motor have to be of non-zero size.
     * All motor vectors have to be of the same size.
     * If positions and speeds are non-zero, their size has to match.
     * Motors are not required to support both, position & speed trajectory modes.
     * If both vectors are given, the implementation is free to chose one parameter of the other.
     *
     * @param curvatures The curvatures, the chassis of the robot is moving on.
     *   fabs(curvatures[]) >= 1000 is considered a point-turn..
     *   NAN, if wheels are not describing a curvature.
     * @param curvatureRates The rates of curvature changes of the chassis.
     * @param speeds The speed of the robot chassis along the curvature.
     *
     * Curvatures, curvatureRates and speeds vectors can be empty. If of non-empty size, they
     * have to match the size of the non-zero TrajectoryVector's members size.
     */
    virtual int cmdTrajectories(int ref,
                                double frequency,
                                TrajectoryVector const& mPositions,
                                TrajectoryVector const& mSpeeds,
                                DoubleVector const& curvatures,
                                DoubleVector const& curvatureRates,
                                DoubleVector const& crabAngles,
                                DoubleVector const& crabRates,
                                DoubleVector const& speeds) = 0;
    //! The current/latest status of the wheel group.
    /**
     * @param waitNextSample If true, the method will wait for the next telemetry update from the wheel-group before returning.
     * @param timeout If non-NULL and waitForCompletion is true,
     * the method will return after the absolute deadline is reached and timeout will then be set to zero.
     */
    virtual WheelGroupSample wheelGroupState(bool waitNextSample = false, ACE_Time_Value const * timeout = NULL) = 0;

    //! Returns the projected motor group state some time ahead in the future.
    /** This methos is designed to provide the necessary state information to overwrite
     * a cmdTrajectories() command.
     * @param state This is the expected motor state, given that no other command is executed before timestamp.
     * @param targetTime The absolutte time in the future. If ACE_Time_Value::zero is passed, the mehod
     * will return the earliest time in the future for which a overwriting cmdTrajectory cmd can be given.
     *@retun The ref parameter for the cmdTrajectory command.
     * -1 will be returned, if the Motor is currently IDLE.
     * -2 will be returned, if the Motor is IDLE at the targeted time.
     * KNMOTOR_ERR_TIMEOUT will be returned, if the targetTime is not reachable.
     *
     */
    virtual int futureWheelGroupState(WheelGroupFuture& state, ACE_Time_Value const& targetTime = ACE_Time_Value::zero) = 0;
  };
}
#endif // kn_WheelGroup_h
