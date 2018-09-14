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
 * Module: knProtoMotor
 * Author: Vinh To
 *
 *****************************************************************************/

#ifndef kn_protoMotor_MotorInterface_h
#define kn_protoMotor_MotorInterface_h

#include "knProtoMotor_Export.h"

#include "WheelGroup.h"

#include "knMotor/MotorBase.h" // stop modes
#include "knMotor/MotorProfile.h" // stop modes
#include "knMotor/MotorGroup.h"

#include "knMotorImpl/MotorImpl.h"

#include <ace/Event_Handler.h>
#include <ace/SOCK_Dgram.h>
#include <ace/SOCK_Dgram_Bcast.h>
#include <ace/INET_Addr.h>
#include <ace/Recursive_Thread_Mutex.h>
#include <ace/Service_Config.h>
#include <ace/Service_Object.h>

#include <string>
#include <vector>
#include <map>
#include <fstream>

#define MOTOR_ARC_CMD        "ARC"
#define MOTOR_POINT_TURN_CMD "POINT"
#define MOTOR_CRAB_CMD       "CRAB"
#define MOTOR_PROFILE_CMD    "PROFILE"
#define MOTOR_TRAJECTORY_CMD "TRAJ"
#define MOTOR_C_ACK_CMD      "C_ACK"
#define MOTOR_RECOVER_CMD    "TRY_RECOVER"
#define MOTOR_REENABLE_CMD   "REENABLE"
#define MOTOR_STOP_CMD       "STOP"
#define MOTOR_V_ACK_CMD      "V_ACK"
#define MOTOR_CMD_ACK_CMD    "CMD_ACK"
#define MOTOR_STATUS_CMD     "STAT"

namespace Miro
{
  class ReactorTask;
}

namespace kn
{
  class WheelGroup;

  namespace protoMotor
  {
    class ProtoMotorParameters;
    //    class WheelGroup;

    // These values are from rocker_config.h in the krex code base
    // AXIS_STATUS bits that are used to make up the 32-bit unsigned integer sent in the STAT messages
    static const unsigned int AXIS_STATUS_FAULT_BIT              = 1 << 0; // bit 0
    static const unsigned int AXIS_STATUS_BUSY_BIT               = 1 << 1; // bit 1
    static const unsigned int AXIS_STATUS_TRACKING_ERR_BIT       = 1 << 2; // bit 2
    static const unsigned int AXIS_STATUS_WIRELESS_ESTOP_BIT     = 1 << 3; // bit 3
    static const unsigned int AXIS_STATUS_ROCKER_ESTOP_BIT       = 1 << 4; // bit 4
    static const unsigned int MOTOR_ENABLED_BIT                  = 1 << 5; // bit 5

    class knProtoMotor_Export MotorInterface : public ACE_Service_Object
    {
    public:

      struct MotorStatus {
        double current;
        double speed;
        double position;
        int status;
        unsigned short emergencyCode;
      };

      struct TrajectoryStep {
        float leftFrontDrive;
        float rightFrontDrive;
        float leftRearDrive;
        float rightRearDrive;
        float leftFrontSteer;
        float rightFrontSteer;
        float leftRearSteer;
        float rightRearSteer;
      };

      struct RockerStatus {
        MotorStatus motorStatus[4];
        double voltage;
        int cmdId;
        int trajIndex;
        ACE_Time_Value timestamp;
      };

      enum {
        INVALID = (-1),
        ARC = 0,
        POINT = 1,
        CRAB = 2,
        PROFILE = 3,
        TRAJECTORY = 4,
        TRANSITION = 5,
        TRY_RECOVER = 6,
        STOP = 7,
        CONTROLLER_ACK = 8
      };

      typedef std::vector<float> FloatVector;
      typedef std::vector<double> DoubleVector;
      typedef std::vector<MotorProfile> MotorProfileVector;
      typedef std::vector<MotorStatus> MotorStatusVector;
      typedef std::vector<TrajectoryStep> TrajectoryStepVector;
      typedef std::vector<DoubleVector> TrajectoryVector;

      MotorInterface();
      ~MotorInterface();

      virtual int init(int argc, ACE_TCHAR *argv[]);
      virtual int fini();

      void connect(ACE_Reactor * r);
      //void disconnect();

      virtual void set_handle(ACE_HANDLE handle);
      virtual ACE_HANDLE get_handle() const;
      virtual int handle_input(ACE_HANDLE fd);

      /**
      * Vehicle will only execute an arc command if it is in arc-driving mode.
      * Enter arc driving mode by sending a 0 speed arc command.
      * \param radius Is in meters.  A straight path is defined as having a radius of 1000 m or greater
      * \param speed Is in meters/second
      * \return true if the command was successful, false otherwise
      */
      bool arc(float radius, float speed);

      /**
       * Vehicle will only execute point turn command if it is in point turn mode.
       * Enter point turn mode by sending a 0 speed point turn command.
       * \param angularVelocity Is in meters/second
       * \return The command ID that was used.
       */
      bool pointTurn(float angularVelocity);

      /**
       * Vehicle will only execute crab command if it is in crab steer mode.
       * Enter crab steer mode by sending a 0 speed crab command.
       * \param steerAngle Is in degrees.
       * \param speed Is in meters/second
       * \return true if the command was successful, false otherwise
       */
      bool crab(float steerAngle, float speed);

      /**
       * Vehicle will only execute profile command if it is in profile mode.
       * Enter profile driving mode by sending a 0 position, speed, and acceleration for each motor.
       * \param MotorProfile Is a position in radians relative to the previous command position,
                             speed in radians/seconds, acceleration in radians/seconds^2
       * \return true if the command was successful, false otherwise
       */
      bool profile(MotorProfile leftFrontDrive, MotorProfile rightFrontDrive, MotorProfile leftRearDrive, MotorProfile rightRearDrive,
                   MotorProfile leftFrontSteer, MotorProfile rightFrontSteer, MotorProfile leftRearSteer, MotorProfile rightRearSteer);

      bool profile(MotorProfileVector const & profiles);

      bool trajectory(int sequenceLength,
                      DoubleVector const & period,
                      TrajectoryStepVector const & trajectorySteps,
                      int stepId = 0);

      /**
       * Try to recover from a fault
       * \return true if the command was successful, false otherwise
       */
      bool tryRecover();

      /**
       * This will cause the vehicle to stop moving all motors no matter what driving mode it's in
       * There are 3 stopping modes
       * NORMAL Stop normally with each axis' nominal deceleration
       * ABRUPT Stop quickly, with higher deceleration on all axis
       * MOTORS_OFF Cut power to the motors
       * \param String of the stop mode
       * \return true if the command was successful, false otherwise
       */
      bool stop(MotorBase::StopMode mode);

    private:
      int parseArgs(int& argc, char * argv[]);
      bool _sendCmd();
      bool _sendCmd(std::string const& cmd);
      int motorStatus(RockerStatus const & leftRocker, RockerStatus const & rightRocker);
      static void updateMotor(MotorHelper& motor,
                              MotorStatus const& motorStatus, bool moveDone, int commandType, int transitionMode,
                              ACE_Time_Value const& timestamp,
                              unsigned int step, ACE_Time_Value const& targetTime);

      ProtoMotorParameters * m_params;
      bool m_verbose;

      Miro::ReactorTask * m_reactorTask;

      ACE_Recursive_Thread_Mutex m_mutex;
      ACE_Condition_Recursive_Thread_Mutex m_condition;

      kn::protoMotor::WheelGroup * m_wheelGroup;

      ACE_SOCK_Dgram_Bcast m_outBound;
      ACE_SOCK_Dgram_Bcast m_inBound;
      ACE_HANDLE     m_handle;

      std::string  m_buffer;
      unsigned int m_cmdId;
      unsigned int m_ackId;
      char m_buff[BUFSIZ];
      char m_recvBuffer[BUFSIZ];
      char m_cmdBuffer[BUFSIZ];

      std::map<int, std::string> m_commands;
      MotorProfileVector m_profiles;
      WheelGroupSample m_prevWheelSample;
      RockerStatus m_leftRockerStatus;
      RockerStatus m_rightRockerStatus;
      TrajectoryStepVector m_trajectorySteps;
      bool m_updateLeftRocker;
      bool m_updateRightRocker;
      bool m_rightIdle;
      bool m_leftIdle;
      int m_transitionMode;
      int m_ignoreCycle;
      int m_commandType;
      int m_cmdAckStatus;
      int m_trajectoryIndex;
      int m_currentStatus;
      ACE_Time_Value m_timeSlice;
      ACE_Time_Value m_prevStatusTime;
      std::ofstream m_telemOutputlog;
      std::ofstream m_trajOutputlog;

      friend class protoMotor::WheelGroup;
    };
  }
}

// Declare dynamic services.
//ACE_STATIC_SVC_DECLARE_EXPORT(knProtoMotor, kn_MotorInterfac)
ACE_FACTORY_DECLARE(knProtoMotor, kn_MotorInterface)


#endif //kn_protoMotor_MotorInterface_h
