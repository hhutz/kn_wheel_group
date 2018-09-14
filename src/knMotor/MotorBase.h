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

#ifndef kn_MotorBase_h
#define kn_MotorBase_h

#include "knMotor_Export.h"

#include <climits>
#include <vector>

class ACE_Time_Value;

namespace kn
{
  //! Motor states.
  /** 
   * Telemetry error states are sticky (except for estop).
   * Most are recoverable.
   *
   * If one motor of a motor or wheel group holds an error status, the group
   * is set to this error. Individual motors record, if the error is 
   * concerning their own device or if some other motor of the group is in trouble.
   */
  static int const KNMOTOR_OKAY = 0;
  static int const KNMOTOR_IDLE = 0;
  static int const KNMOTOR_BUSY = 1;
  static int const KNMOTOR_BUSY_INTERRUPTABLE = 3;
  static int const KNMOTOR_BUSY_CALIBRATING = 5;

  static int const KNMOTOR_ERR_OTHER =         INT_MIN +    1; //!< unspecified error
  static int const KNMOTOR_ERR_NUM_MOTORS =    INT_MIN +    2; //!< parameter vector length does not match number of motors
  static int const KNMOTOR_ERR_PARAM_RANGE =   INT_MIN +    4; //!< parameter out of range
  static int const KNMOTOR_ERR_CALIBRATION =   INT_MIN +    8; //!< not calibrated
  static int const KNMOTOR_ERR_TIMEOUT =       INT_MIN +   16; //!< timeout in waiting for reply or trajectory ref no longer overwriteable
  static int const KNMOTOR_ERR_TRAJ_CMD =      INT_MIN +   32; //!< inconsistend trajectory command vectors
  static int const KNMOTOR_ERR_HW_ESTOP =      INT_MIN +   64; //!< hardware is estopped
  static int const KNMOTOR_ERR_SW_ESTOP =      INT_MIN +  128; //!< sticky software estopp
  static int const KNMOTOR_ERR_TRAJ_EXEC =     INT_MIN +  256; //!< trajectory execution went south (buffer underrun)
  static int const KNMOTOR_ERR_POS_ERROR =     INT_MIN +  512; //!< motor position errors exceed safety thresholds
  static int const KNMOTOR_ERR_OVERCURRENT =   INT_MIN + 1024; //!< motor reported overcurrent
  static int const KNMOTOR_ERR_POWER_FAULT =   INT_MIN + 2048; //!< motor reported loss of power
  static int const KNMOTOR_ERR_HW_COM =        INT_MIN + 4096; //!< hardware comm problem (bus error)
  static int const KNMOTOR_ERR_NOT_SUPPORTED = INT_MIN + 8192; //!< command not supported by device

  //! Abstract motor interface base.
  /**
   * Base functionality shared between motors and motor/wheel-groups.
   */
  class knMotor_Export MotorBase
  {
  public:
    //! Motors have different capabilities for comming to a stop.
    enum StopMode {
      STOP_EMERGENCY, //!< Stop as quickly as possible.
      STOP_OFF,       //!< Disable motors.
      STOP_ABRUPTLY,  //!< Abrubt stop. Often the same as emegency stop mode.
      STOP_SMOOTHLY   //!< Smooth stop. Fully contorlled halt.
    };
    //! The interface provided different calibration capabilities.
    enum CalibrationMode {
      CALIBRATE_SYNC,  //!< Synchronous calibration.
      CALIBRATE_ASYNC, //!< Start calibration return immedidately.
      CALIBRATE_WAIT   //!< Wait for completion of an asynchronous motor calibration.
    };

    //! Vector of doubles.
    typedef std::vector<double> DoubleVector;

    //! Virtual dtor. This method does not throw.
    virtual ~MotorBase() throw();

    //! Calibrate motors.
    /** Most steer motors need to be calibrated to the steering angle.
     * Calibration can either be done synchronous or asynchornous.
     *
     * During calibration, the cmdStatus is KNMOTOR_BUSY.
     */
    virtual int calibrate(CalibrationMode mode) = 0;
    //! Attempt recovery from failure state.
    /** Most motor errors (over-current events etc), are sticky.
     * Those errors must be cleared explicitly, before operations can resume.
     *
     * During recovery, the cmdStatus is KNMOTOR_BUSY.
     */
    virtual int tryRecover() = 0;
    //! Stop in the specified control mode.
    /** The stop command is the only accepted command, while cmdStatus is KNMOTOR_BUSY.
     * During stopping, the cmdStatus is still busy.
     */
    virtual int stop(StopMode mode) = 0;
    //! Return status of command execution.
    /**
     * @param waitForCompletion If true, the method will only return with a return code <= 0.
     * @param timeout If non-NULL and waitForCompletion is true,
     * the method will return with KNMOTOR_ERR_TIMEOUT if absolute deadline is reached.
     */
    virtual int cmdStatus(bool waitForCompletion = false, ACE_Time_Value const * timeout = NULL) = 0;

    //! Return an error string for the return code.
    static char const * getErrorName(int error) throw();

    //! check for specific error in error status
    static bool isError(int error, int status) throw();
    //! clear a  specific error in error status
    static int clearError(int error, int status) throw();
  };
}
#endif // kn_MotorBase_h
