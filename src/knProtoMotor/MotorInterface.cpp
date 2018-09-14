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

#include "MotorInterface.h"
#include "WheelGroup.h"
#include "knMotorImpl/MotorHelper.h"
#include "ProtoMotorParameters.h"

#include "knMotorImpl/WheelGroupRepository.h"

#include "miro/Log.h"
#include "miro/ConfigDocument.h"
#include "miro/Configuration.h"
#include "miro/TimeHelper.h"
#include "miro/ReactorTask.h"

#include <ace/Guard_T.h>
#include <ace/Reactor.h>
#include <ace/Get_Opt.h>

#include <iostream>
#include <sstream>

#include <stdio.h>
#include <time.h>

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)

namespace
{
  int const modeMap[] = {
    1, // ABRUBT
    2, // MOTORS_OFF
    1, // ABRUBT
    0  // NORMAL
  };
}

namespace kn
{
  using namespace std;

  namespace protoMotor
  {
    MotorInterface::MotorInterface() :
      m_params(NULL),
      m_verbose(false),
      m_reactorTask(NULL),
      m_mutex(),
      m_condition(m_mutex),
      m_wheelGroup(NULL),
      m_handle(0),
      m_buffer(""),
      m_cmdId(0),
      m_ackId(0),
      m_updateLeftRocker(false),
      m_updateRightRocker(false),
      m_rightIdle(false),
      m_leftIdle(false),
      m_transitionMode(0),
      m_ignoreCycle(0),
      m_commandType(-1),
      m_cmdAckStatus(1),
      m_trajectoryIndex(0),
      m_currentStatus(0),
      m_prevStatusTime(0)
    {
      MIRO_LOG_CTOR("kn::protoMotor::MotorInterface");
      ostringstream fileName;
      ostringstream fileName2;

      time_t rawtime;
      struct tm * timeinfo;
      char buffer [80];
      time(&rawtime);
      timeinfo = localtime(&rawtime);

      strftime(buffer, 80, "%Y.%m.%d-%H.%M.%S.txt", timeinfo);

      fileName << "/usr/local/irg/data/motorLog/KRexMotorsTelem_" << buffer;
      m_telemOutputlog.open(fileName.str().c_str(), ifstream::out);

      fileName2 << "/usr/local/irg/data/motorLog/KRexTraj_" << buffer;
      m_trajOutputlog.open(fileName2.str().c_str(), ifstream::out);
    }

    MotorInterface::~MotorInterface()
    {
      MIRO_LOG_DTOR("kn::protoMotor::MotorInterface");
      m_telemOutputlog.close();
      m_trajOutputlog.close();
    }

    int
    MotorInterface::init(int argc, ACE_TCHAR *argv[])
    {
      MIRO_LOG(LL_NOTICE, "MotorInterface::init");

      if (parseArgs(argc, argv) != 0) {
        cout << "parseArgs failed" << endl;
        //return -1;
      }

      m_reactorTask = new Miro::ReactorTask(&m_params->reactorPriority);
      m_reactorTask->activate();

      // create wheel group
      WheelGroupParameters const& params = m_params->wheelGroup;

      cout << "MotorInterface params = " << m_params << endl;
      WheelGroup::MotorVector motors;
      {
        vector<MotorParameters>::const_iterator first, last = params.motors.end();

        for (first = params.motors.begin(); first != last; ++first) {
          motors.push_back(new MotorHelper(first.base()));
        }
      }

      WheelGroup::WheelParametersVector wParams;
      {
        vector<WheelParameters>::const_iterator first, last = params.wheels.end();

        for (first = params.wheels.begin(); first != last; ++first) {
          wParams.push_back(first.base());
        }
      }
      m_wheelGroup = new WheelGroup(this, motors,
                                    &params, wParams);

      m_wheelGroup->m_auxValues.resize(1);
      m_wheelGroup->m_auxValues[0].first = "RockerVoltages";
      m_wheelGroup->m_auxValues[0].second.resize(2);

      ACE_INET_Addr outBoundAddr((u_short)0);

      if (m_outBound.open(outBoundAddr) == -1) {
        MIRO_LOG(LL_ERROR, "Couldn't open socket connection.");
      }

      ACE_INET_Addr inBoundAddr(m_params->inBoundPort);

      if (m_inBound.open(inBoundAddr) == -1) {
        MIRO_LOG(LL_ERROR, "Couldn't open socket connection.");
      }

      set_handle(m_inBound.get_handle());

      reactor(m_reactorTask->reactor());
      connect(reactor());

      WheelGroupRepository::instance()->add("WheelGroup", m_wheelGroup);

      // the stack comes up with the wheels disabled.  This will enable them
      tryRecover();

      return 0;
    }

    int
    MotorInterface::fini()
    {
      WheelGroupRepository::instance()->remove("WheelGroup");
      reactor()->remove_handler(this, ::ACE_Event_Handler::READ_MASK);
      //delete m_wheelGroup;
      m_wheelGroup = 0;

      m_reactorTask->shutdown(false);
      m_reactorTask->wait();
      delete m_reactorTask;
      m_reactorTask = NULL;

      return 0;
    }

    void
    MotorInterface::connect(ACE_Reactor * r)
    {
      reactor(r);
      reactor()->register_handler(this, ACE_Event_Handler::READ_MASK);
    }

    void
    MotorInterface::set_handle(ACE_HANDLE handle)
    {
      m_handle = handle;
    }

    ACE_HANDLE
    MotorInterface::get_handle() const
    {
      return m_handle;
    }

    int
    MotorInterface::handle_input(ACE_HANDLE fd)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

      ACE_INET_Addr remote(m_params->inBoundPort, INADDR_BROADCAST);

      int sizeRead = m_inBound.recv(m_recvBuffer, sizeof(m_recvBuffer), remote);

      m_recvBuffer[sizeRead + 1] = '\n';
      string buffer(m_recvBuffer);
      memset(m_recvBuffer, 0, BUFSIZ);
      // Look for V_ACK messages and reply with the ackId.
      size_t found;

      found = buffer.find(MOTOR_V_ACK_CMD);

      if (found != string::npos) {
        MIRO_LOG_OSTR(LL_DEBUG, "Got a V_ACK: " << buffer);
        m_telemOutputlog << buffer << flush;
        found += ACE_OS::strlen(MOTOR_V_ACK_CMD) + 1;
        memset(m_buff, 0, BUFSIZ);
        string ackString = buffer.substr(found, buffer.length() - 1);
        ackString[ackString.length() - 1] = '\0';
        sprintf(m_buff, "%s %s 0\n", MOTOR_C_ACK_CMD, ackString.c_str());
        _sendCmd();
      }

      // Look for CMD_ACK messages
      found = string::npos;
      found = buffer.find(MOTOR_CMD_ACK_CMD);

      if (found != string::npos) {
        MIRO_LOG_OSTR(LL_DEBUG, "Got a CMD_ACK: " << buffer);
        m_telemOutputlog << buffer << flush;
        found += ACE_OS::strlen(MOTOR_CMD_ACK_CMD) + 1;
        string cmdIdString = buffer.substr(found, buffer.length() - 1);
        istringstream statusStream(cmdIdString);
        char side;
        int cmdId;// = atoi(cmdIdString.c_str());
        int extendedStatus;
        statusStream >> side;
        statusStream >> cmdId;
        statusStream >> m_cmdAckStatus;
        statusStream >> extendedStatus;

        m_commands.erase(cmdId);
      }

      // Look for STAT messages and save the rocker status
      found = string::npos;
      found = buffer.find(MOTOR_STATUS_CMD);

      if (found != string::npos) {
        RockerStatus rockerStatus;
        char side;
        //int commandType;
        MIRO_LOG_OSTR(LL_DEBUG, "Got a STAT_ACK: " << buffer);
        m_telemOutputlog << ACE_OS::gettimeofday() << " " << buffer << flush;
        found += ACE_OS::strlen(MOTOR_STATUS_CMD) + 1;
        string status = buffer.substr(found, buffer.length() - 1);
        istringstream statusStream(status, istringstream::in);
        int cmdId;
        int commandType;
        // #? = field number in the message
        statusStream >> side;               // #3
        statusStream >> commandType;        // #4
        statusStream >> cmdId;              // #5
        statusStream >> m_trajectoryIndex;  // #6

        // ignore returned cmdId if we are not in trajectory mode
        if (commandType != MotorInterface::TRAJECTORY) {
          //cout << "setting cmdId to zero.  not in traj mode" << endl;
          cmdId = 0;
        }

        if (side == 'U') {
          for (int i = 0; i < 4; ++i) {
            // i=0 -> front left drive
            // i=1 -> rear left drive
            // i=2 -> front left steer
            // i=3 -> rear left steer
            //                                                              i=  0    1    2   4
            statusStream >> m_leftRockerStatus.motorStatus[i].position;     // #7   #11  #15  #19
            statusStream >> m_leftRockerStatus.motorStatus[i].speed;        // #8   #12  #16  #20
            statusStream >> m_leftRockerStatus.motorStatus[i].current;      // #9   #13  #17  #21
            statusStream >> m_leftRockerStatus.motorStatus[i].status;       // #10  #14  #18  #22
          }

          //double time;
          statusStream >> m_leftRockerStatus.voltage;                       // #23

          for (int i = 0; i < 4; ++i) {
            // i=0 -> front right drive
            // i=1 -> rear right drive
            // i=2 -> front right steer
            // i=3 -> rear right steer
            //                                                              i=  0    1    2    4
            statusStream >> m_rightRockerStatus.motorStatus[i].position;    // #24  #28  #32  #36
            statusStream >> m_rightRockerStatus.motorStatus[i].speed;       // #25  #29  #33  #37
            statusStream >> m_rightRockerStatus.motorStatus[i].current;     // #26  #30  #34  #38
            statusStream >> m_rightRockerStatus.motorStatus[i].status;      // #27  #31  #35  #39
          }

          //cout << "telem cmdId rightFrontSteer = " << cmdId << " " << m_rightRockerStatus.motorStatus[2].position << endl;

          statusStream >> m_rightRockerStatus.voltage;                      // #40
          m_rightRockerStatus.cmdId = cmdId;
          m_rightRockerStatus.trajIndex = m_trajectoryIndex;

          m_leftRockerStatus.cmdId = cmdId;
          m_leftRockerStatus.trajIndex = m_trajectoryIndex;

          for (int i = 0; i < 4; ++i) {                                     // #41  #42  #43  #44
            statusStream >> hex >> m_leftRockerStatus.motorStatus[i].emergencyCode;
          }

          for (int i = 0; i < 4; ++i) {                                     // #45  #46  #47  #48
            statusStream >> m_rightRockerStatus.motorStatus[i].emergencyCode;
          }

          double time;
          statusStream >> dec >> time;      // # 49
          ACE_Time_Value timeNow = ACE_OS::gettimeofday();
          m_leftRockerStatus.timestamp = timeNow;
          m_rightRockerStatus.timestamp = timeNow;

          m_wheelGroup->m_auxValues[0].second[0] = m_leftRockerStatus.voltage;
          m_wheelGroup->m_auxValues[0].second[1] = m_rightRockerStatus.voltage;

          m_updateLeftRocker = true;
          m_updateRightRocker = true;

          if (m_prevStatusTime != ACE_Time_Value::zero) {
            ACE_Time_Value diff = timeNow - m_prevStatusTime;

            if (diff > ACE_Time_Value(0, 40000)) {
              cout << timeNow << ": Missed motor sample: " << diff << endl;
            }
          }

          m_prevStatusTime = timeNow;
        }

        if (m_updateLeftRocker && m_updateRightRocker) {

          if (m_currentStatus != m_rightRockerStatus.motorStatus[0].status) {
            MIRO_LOG_OSTR(LL_DEBUG, "%%%%%%%%%%%% current status changed from " << m_currentStatus << " to " << m_rightRockerStatus.motorStatus[0].status);
          }

          m_currentStatus = m_rightRockerStatus.motorStatus[0].status;

          if (m_commandType != commandType) {
            MIRO_LOG(LL_NOTICE, "Transition mode, ignoring motor enabled bit for 10 cycles...");
            m_transitionMode = 10;
          }

          m_commandType = commandType;

          WheelGroup::MotorVector& motors = m_wheelGroup->motors();

          bool driveDone = false;

          if (m_wheelGroup->cmdStatus() > KNMOTOR_IDLE && m_cmdAckStatus != 1) {
            if (m_commandType == MotorInterface::PROFILE) {
              driveDone = true;

              if (!m_profiles.empty()) {

                for (int i = 0; i < 4; ++i) {
                  if ((m_leftRockerStatus.motorStatus[i].status & AXIS_STATUS_BUSY_BIT) ||
                      m_rightRockerStatus.motorStatus[i].status & AXIS_STATUS_BUSY_BIT) {
                    driveDone = false;
                    break;
                  }
                }
              }

              if (driveDone) {
                cout << "MotorInterface::PROFILE driveDone" << endl;
              }
            }
            else if (m_commandType == MotorInterface::TRAJECTORY) {
              if (cmdId == -1 &&
                  m_ignoreCycle == 0) {
                driveDone = true;
                cout << "MotorInterface::TRAJECTORY driveDone" << endl;
              }
              else {
                if (m_ignoreCycle > 0) {
                  --m_ignoreCycle;
                }
              }
            }
          }

          ACE_Time_Value maxTimestamp = max(m_leftRockerStatus.timestamp,
                                            m_rightRockerStatus.timestamp);

          int trajStep = 0;
          int maxId = max(m_leftRockerStatus.cmdId, m_rightRockerStatus.cmdId);

          if (m_commandType == MotorInterface::TRAJECTORY && m_ignoreCycle == 0) {
            m_trajectoryIndex = maxId;
            trajStep = maxId - m_wheelGroup->indexOffset();

            //trajStep = maxId + 2 - m_wheelGroup->indexOffset();
            if (trajStep < 0 && !driveDone) {
              cout << "           maxId = " << maxId
                   << "  indexOffset = " << m_wheelGroup->indexOffset()
                   << "  left cmdId = " << m_leftRockerStatus.cmdId
                   << "  right cmdId = " << m_rightRockerStatus.cmdId
                   << "  trajStep = " << trajStep << endl;
              trajStep = 0;
              cout << "************** trajStep is NEGATIVE!!!!!!!!" << endl;
            }
          }

          WheelGroupSample s = m_wheelGroup->wheelGroupState();

          ACE_Time_Value leftTime = m_leftRockerStatus.timestamp + (double)trajStep * m_timeSlice;
          ACE_Time_Value rightTime = m_rightRockerStatus.timestamp + (double)trajStep * m_timeSlice;
          ACE_Time_Value targetTime = max(leftTime, rightTime);

          if (m_wheelGroup->cmdStatus() == KNMOTOR_BUSY_INTERRUPTABLE) {
            //cout << "trajectory:  id=" << maxId << " offset=" << m_wheelGroup->indexOffset() << endl;
          }

          // left front drive
          updateMotor(*motors[0],
                      m_leftRockerStatus.motorStatus[0], driveDone, m_commandType, m_transitionMode,
                      m_leftRockerStatus.timestamp, trajStep, targetTime);

          // right front drive
          updateMotor(*motors[1],
                      m_rightRockerStatus.motorStatus[0], driveDone, m_commandType, m_transitionMode,
                      m_rightRockerStatus.timestamp, trajStep, targetTime);

          // left rear drive
          updateMotor(*motors[2],
                      m_leftRockerStatus.motorStatus[1], driveDone, m_commandType, m_transitionMode,
                      m_leftRockerStatus.timestamp, trajStep, targetTime);

          // right rear drive
          updateMotor(*motors[3],
                      m_rightRockerStatus.motorStatus[1], driveDone, m_commandType, m_transitionMode,
                      m_rightRockerStatus.timestamp, trajStep, targetTime);

          // left front steer
          updateMotor(*motors[4],
                      m_leftRockerStatus.motorStatus[2], driveDone, m_commandType, m_transitionMode,
                      m_leftRockerStatus.timestamp, trajStep, targetTime);

          // right front steer
          updateMotor(*motors[5],
                      m_rightRockerStatus.motorStatus[2], driveDone, m_commandType, m_transitionMode,
                      m_rightRockerStatus.timestamp, trajStep, targetTime);

          // left rear steer
          updateMotor(*motors[6],
                      m_leftRockerStatus.motorStatus[3], driveDone, m_commandType, m_transitionMode,
                      m_leftRockerStatus.timestamp, trajStep, targetTime);

          // right rear steer
          updateMotor(*motors[7],
                      m_rightRockerStatus.motorStatus[3], driveDone, m_commandType, m_transitionMode,
                      m_rightRockerStatus.timestamp, trajStep, targetTime);


          if (m_wheelGroup->cmdStatus() == KNMOTOR_BUSY_INTERRUPTABLE && m_commandType == MotorInterface::TRAJECTORY) {
            //cout << "m_wheelGroup->consumeTrajectoryPoints("<<trajStep<<", "<<maxTimestamp<<", "<<targetTime<<");" << endl;
            m_wheelGroup->consumeTrajectoryPoints(trajStep, maxTimestamp, targetTime);
          }

          // and now for the whole wheel group
          if (driveDone) {
            m_wheelGroup->onCommandCompletion();
          }

          m_updateLeftRocker = false;
          m_updateRightRocker = false;

          if (m_transitionMode > 0) {
            --m_transitionMode;
          }

          m_wheelGroup->collectMotorStatus(m_condition, ACE_OS::gettimeofday());
        }
      }

      return 0;
    }

    bool
    MotorInterface::arc(float radius, float speed)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
      memset(m_buff, 0, BUFSIZ);

      // need to transition to ARC commany type if not in ARC mode
      if (m_commandType != MotorInterface::ARC) {
        sprintf(m_buff, "%s %d %f %f\n", MOTOR_ARC_CMD, m_cmdId, 0.0, 0.0);
        m_commands.insert(pair<int, string>(m_cmdId, m_buff));
        m_cmdId++;
        _sendCmd();
      }

      sprintf(m_buff, "%s %d %f %f\n", MOTOR_ARC_CMD, m_cmdId, radius, speed);
      m_commands.insert(pair<int, string>(m_cmdId, m_buff));
      m_cmdId++;

      m_trajOutputlog << ACE_OS::gettimeofday() << " " << m_buff << flush;

      return _sendCmd();
    }

    bool
    MotorInterface::pointTurn(float angularVelocity)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

      memset(m_buff, 0, BUFSIZ);

      sprintf(m_buff, "%s %d %f\n", MOTOR_POINT_TURN_CMD, m_cmdId, angularVelocity);

      m_commands.insert(pair<int, string>(m_cmdId, m_buff));
      m_cmdId++;

      m_trajOutputlog << ACE_OS::gettimeofday() << " " << m_buff << flush;

      return _sendCmd();
    }

    bool
    MotorInterface::crab(float steerAngle, float speed)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

      memset(m_buff, 0, BUFSIZ);

      sprintf(m_buff, "%s %d %f %f\n", MOTOR_CRAB_CMD, m_cmdId, steerAngle, speed);
      m_commands.insert(pair<int, string>(m_cmdId, m_buff));
      m_cmdId++;
      return _sendCmd();
    }

    bool
    MotorInterface::profile(MotorProfile leftFrontDrive, MotorProfile rightFrontDrive, MotorProfile leftRearDrive, MotorProfile rightRearDrive,
                            MotorProfile leftFrontSteer, MotorProfile rightFrontSteer, MotorProfile leftRearSteer, MotorProfile rightRearSteer)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

      ostringstream message;
      message << MOTOR_PROFILE_CMD;
      message << " ";
      message << m_cmdId << " ";
      message << "\n";

      m_trajOutputlog << ACE_OS::gettimeofday() << " " << message.str() << flush;

      m_commands.insert(pair<int, string>(m_cmdId, message.str()));
      m_cmdId++;

      return _sendCmd(message.str());
    }

    bool
    MotorInterface::profile(MotorProfileVector const & profiles)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

      m_profiles = profiles;

      ostringstream message;
      message << MOTOR_PROFILE_CMD;
      message << " ";
      message << m_cmdId << " ";

      for (unsigned i = 0; i < m_profiles.size(); ++i) {
        message << m_profiles[i].position*RAD2DEG << " ";
        message << m_profiles[i].speed*RAD2DEG << " ";
        message << m_profiles[i].acc*RAD2DEG << " ";
      }

      message << "\n";

      m_trajOutputlog << ACE_OS::gettimeofday() << " " << message.str() << flush;

      m_commands.insert(pair<int, string>(m_cmdId, message.str()));
      m_cmdId++;

      return _sendCmd(message.str());
    }

    bool
    MotorInterface::trajectory(int sequenceLength,
                               DoubleVector const & period,
                               TrajectoryStepVector const & trajectorySteps,
                               int stepId)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
      int trajId = 0;

      m_timeSlice.set(period[0]);

      m_profiles.clear();

      //cout << "reseting m_cmdId from " << m_cmdId << " to " << stepId << endl;
      trajId = stepId;
      m_cmdId = stepId;

      if (m_trajectoryIndex > trajId) {
        cerr << "********** MotorInterface::trajectory: trying to use a trajId from the past: "
             << "m_trajectoryIndex = " << m_trajectoryIndex
             << " current trajId = " << trajId << endl;
      }

      // if this is a start of a new trajectory, we need to ignore the next 2 STAT message
      // trajectory status inorder to let the motors start moving before we report
      // trajectory status
      if (stepId == 0) {
        m_ignoreCycle = 2;
      }
      else {
        m_ignoreCycle = 0;
      }

      ostringstream message;
      message << MOTOR_TRAJECTORY_CMD;
      message << " ";
      message << sequenceLength << " ";

      for (int i = 0; i < sequenceLength; ++i) {
        message << trajId << " ";
        message << period[i] << " ";
        message << trajectorySteps[i].leftFrontDrive << " ";
        message << trajectorySteps[i].rightFrontDrive << " ";
        message << trajectorySteps[i].leftRearDrive << " ";
        message << trajectorySteps[i].rightRearDrive << " ";
        message << trajectorySteps[i].leftFrontSteer << " ";
        message << trajectorySteps[i].rightFrontSteer << " ";
        message << trajectorySteps[i].leftRearSteer << " ";
        message << trajectorySteps[i].rightRearSteer << " ";

        m_trajectorySteps.push_back(trajectorySteps[i]);
        m_commands.insert(pair<int, string>(m_cmdId, message.str()));
        ++trajId;
      }

      message << "\n";

      m_trajOutputlog << ACE_OS::gettimeofday() << " " << message.str() << flush;

      // target time should be 2 steps * freq;

      if (stepId == 0 && m_commandType == MotorInterface::TRAJECTORY) {
        //cout << "MotorInterface::trajectory consumeTrajectoryPoints" << endl;
        ACE_Time_Value now = ACE_OS::gettimeofday();
        m_wheelGroup->consumeTrajectoryPoints(2, now, now + 2.0 * m_timeSlice);

        WheelGroup::MotorVector& motors = m_wheelGroup->motors();
        WheelGroup::MotorVector::iterator first, last = motors.end();

        for (first = motors.begin(); first != last; ++first) {
          (*first)->consumeTrajectoryPoints(2, now, now + 2.0 * m_timeSlice);
        }
      }

      return _sendCmd(message.str());
    }

    bool
    MotorInterface::tryRecover()
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);

      memset(m_buff, 0, BUFSIZ);

      sprintf(m_buff, "%s %d\n", MOTOR_REENABLE_CMD, m_cmdId);
      m_commands.insert(pair<int, string>(m_cmdId, m_buff));

      m_cmdId++;
      return _sendCmd();
    }

    bool
    MotorInterface::stop(MotorBase::StopMode mode)
    {
      ACE_Guard<ACE_Recursive_Thread_Mutex> guard(m_mutex);
      m_profiles.clear();

      memset(m_buff, 0, BUFSIZ);

      sprintf(m_buff, "%s %d %d\n", MOTOR_STOP_CMD, m_cmdId, modeMap[mode]);
      m_commands.insert(pair<int, string>(m_cmdId, m_buff));

      return _sendCmd();
    }

    int
    MotorInterface::parseArgs(int& argc, char* argv[])
    {
      int rc = 0;
      int c;

      // initialize parameters with global instance
      m_params = ProtoMotorParameters::instance();

      // reset defaults
      ProtoMotorParameters defaultParams;
      *m_params = defaultParams;
      m_verbose = false;

      // initialize parameters from config file
      Miro::Configuration::init(argc, argv, /* KNHW_INSTALL_PREFIX FIXME point to devel-space */ "/etc");
      Miro::ConfigDocument * config = Miro::Configuration::document();
      config->setSection("Locomotion");
      config->getParameters("kn::protoMotor::ProtoMotorParameters", *m_params);

      // initialize parameters from command line
      ACE_Get_Opt get_opts(argc, argv, "v?");

      while ((c = get_opts()) != -1) {
        switch (c) {
        case 'v':
          m_verbose = true;
          break;

        case '?':
        default:
          cerr << "usage: " << argv[0] << "[-v?]" << endl
               << "  -v verbose mode" << endl
               << "  -? help: emit this text and stop" << endl;
          rc = 1;
        }
      }

      if (m_verbose) {
        cerr << "ProtoMotor parameters:" << endl;
        cerr << *m_params << endl;
      }

      return rc;
    }

    int
    MotorInterface::motorStatus(RockerStatus const & leftRocker, RockerStatus const & rightRocker)
    {

      if (!m_profiles.empty()) {
        // first check the speed
        for (int i = 0; i < 4; ++i) {
          if (abs(m_leftRockerStatus.motorStatus[i].speed) > 3.0 ||
              abs(m_rightRockerStatus.motorStatus[1].speed) > 3.0) {
            cout << "speed not zero" << endl;
            return KNMOTOR_BUSY;
          }
        }

        // now check if the drive motors are where they are supposed to be

        double delta = 2.0 * DEG2RAD;

        for (int i = 0; i < 4; ++i) {
          double diff = m_profiles[i].position - m_leftRockerStatus.motorStatus[i].position;

          if (fabs(diff) > delta) {
            cout << "not there yet" << endl;
            return KNMOTOR_BUSY;
          }
        }
      }

      return KNMOTOR_IDLE;
    }

    bool
    MotorInterface::_sendCmd()
    {
      MIRO_LOG_OSTR(LL_DEBUG, ACE_OS::gettimeofday() << " " << m_buff);

      ACE_INET_Addr remote(m_params->outBoundPort, m_params->rockerAddr.c_str());

      if (m_outBound.send(m_buff, ACE_OS::strlen(m_buff),
                          remote) == 1) {
        MIRO_LOG(LL_ERROR, "Error sending message");
        return false;
      }

      return true;
    }

    bool
    MotorInterface::_sendCmd(string const& cmd)
    {
      MIRO_LOG_OSTR(LL_DEBUG, ACE_OS::gettimeofday() << " " << cmd);

      ACE_INET_Addr remote(m_params->outBoundPort, m_params->rockerAddr.c_str());

      if (m_outBound.send(cmd.c_str(), cmd.length(),
                          remote) == 1) {
        MIRO_LOG(LL_ERROR, "Error sending message");
        return false;
      }

      return true;
    }

    void
    MotorInterface::updateMotor(MotorHelper& motor,
                                MotorStatus const& motorStatus, bool moveDone, int commandType, int transitionMode,
                                ACE_Time_Value const& timestamp,
                                unsigned int step, ACE_Time_Value const& targetTime)
    {
      MotorSample& sample = motor.sample();

      sample.state.position = motorStatus.position * DEG2RAD;
      sample.state.speed = motorStatus.speed * DEG2RAD;
      sample.current = motorStatus.current;
      sample.timestamp = timestamp;

      if ((sample.status != KNMOTOR_BUSY) && (sample.status != KNMOTOR_BUSY_INTERRUPTABLE)) {

        if ((motorStatus.status == 0) || (motorStatus.status == (int)MOTOR_ENABLED_BIT)) {
          sample.status = 0;
        }
      }

      // map rocker error states to motor error states
      sample.status |= (motorStatus.status & AXIS_STATUS_FAULT_BIT) ?  KNMOTOR_ERR_OTHER : 0;

      // only set power fault if we're not in a transition state
      if (!transitionMode) {
        sample.status |= !(motorStatus.status & MOTOR_ENABLED_BIT) ?   KNMOTOR_ERR_POWER_FAULT : 0;
        //cerr << "Setting MOTOR_ENABLED_BIT" << endl;
      }

      // only set busy bit if we're not in trajectory mode.
      // busy bit is always high in trajectory mode since the motors are in velocity control mode
      if (commandType != MotorInterface::TRAJECTORY) {
        sample.status |= (motorStatus.status & AXIS_STATUS_BUSY_BIT) ? KNMOTOR_BUSY : 0;
      }

      //sample.status |= (motorStatus.status & AXIS_STATUS_ROCKER_ESTOP_BIT) ?  KNMOTOR_ERR_POWER_FAULT : 0;
      //sample.status |= (motorStatus.status & AXIS_STATUS_WIRELESS_ESTOP_BIT) ?  KNMOTOR_ERR_HW_ESTOP : 0;

      if (motorStatus.status & AXIS_STATUS_ROCKER_ESTOP_BIT) {
        sample.status |= KNMOTOR_ERR_POWER_FAULT;
        motor.onCommandCompletion();
      }

      if (motorStatus.status & AXIS_STATUS_WIRELESS_ESTOP_BIT) {
        sample.status |= KNMOTOR_ERR_HW_ESTOP;
        motor.onCommandCompletion();
      }

      if (moveDone) {
        sample.state.cmd.ctrlMode =  MotorProfile::CTRL_HOLD;
        sample.state.cmd.posMode =  MotorProfile::POS_REL;
        sample.state.cmd.position =  0;
        sample.status = KNMOTOR_IDLE;
        //MIRO_LOG(LL_ERROR, "MotorInterface::updateMotor MOVE DONE *********");
        motor.onCommandCompletion();
      }

      if (sample.status == KNMOTOR_BUSY_INTERRUPTABLE && commandType == MotorInterface::TRAJECTORY) {
        //cout << "motor.consumeTrajectoryPoints("<< step<<", "<<timestamp<<", "<<targetTime<<");" << endl;
        motor.consumeTrajectoryPoints(step, timestamp, targetTime);
      }
    }
  }
}

ACE_FACTORY_NAMESPACE_DEFINE(knProtoMotor, kn_MotorInterface, kn::protoMotor::MotorInterface);
