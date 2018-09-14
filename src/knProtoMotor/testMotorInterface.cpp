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

#include "miro/ReactorTask.h"
#include "miro/Log.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

using namespace kn::protoMotor;
using namespace std;

#define RAD2DEG (180.0/M_PI)
#define DEG2RAD (M_PI/180.0)


class CommandLoop
{
public:
  virtual ~CommandLoop() {}

  void svc() {
    string cmd;
    printOptions();
    while (cin >> cmd) {
      if (cmd[0] == 'q')
	break;

      int rc = evalCmd(cmd[0]);
      if (rc == -1) {
	cerr << "C error on cmd: " << strerror(errno) << endl;
	return;
      }
      else if (rc == 1) {
	cerr << "Error: picservo bus in error mode" << endl;
	abort();
      }
      else if (rc == 2) {
	cerr << "Error: cmd reply timed out" << endl;
	return;
      }
      else if (rc == 3) {
	cerr << "Error: end of input stream" << endl;
	return;
      }
      printOptions();
    }
  }

  virtual void printOptions() = 0;
  virtual int evalCmd(char c) = 0;
};

class MotorInterfaceCommands : public CommandLoop
{
public:
  MotorInterfaceCommands(MotorInterface * interface) :
    m_interface(interface) {}

  void printOptions() {
    cout << "a - Drive Arc" << endl
         << "p - Point turn" << endl
         << "c - Crab" << endl
         << "f - Profile" << endl
         << "t - Trajectory" << endl
         << "r - Recover" << endl
         << "n - Normal stop" << endl
         << "e - Abrupt stop" << endl
         << "k - Kill power to motors" << endl
         << "q - quit" << endl;
  }

  int evalCmd(char c) {
    switch(c) {
    case 'a':
      {
        float radius = 0;
        float speed = 0;

        cout << "Radius in meters: ";
        cin >> radius;

        cout << "Speed in m/s: ";
        cin >> speed;

        m_interface->arc(radius, 0);
        m_interface->arc(radius, speed);
      }
      break;
    case 'p':
      {
        float angularVelocity = 0;

        cout << "Angular velocity in m/s: ";
        cin >> angularVelocity;

        m_interface->pointTurn(0);
        m_interface->pointTurn(angularVelocity);
      }
      break;
    case 'c':
      {
        float steerAngle = 0;
        float speed = 0;

        cout << "Steer angle in degrees: ";
        cin >> steerAngle;

        cout << "Speed in m/s: ";
        cin >> speed;

        m_interface->crab(steerAngle, 0);
        m_interface->crab(steerAngle, speed);
      }
      break;

    case 'r':
      {
        cout << "Trying to recover motor";

        m_interface->tryRecover();
      }
      break;

    case 'n':
      {
        cout << "Sending Normal Stop";

        m_interface->stop(kn::MotorBase::STOP_SMOOTHLY);
      }
      break;

    case 'e':
      {
        cout << "Sending Abrupt Stop";

        m_interface->stop(kn::MotorBase::STOP_ABRUPTLY);
      }
      break;

    case 'k':
      {
        cout << "Killing motor power";

        m_interface->stop(kn::MotorBase::STOP_OFF);
      }
      break;

    case 'f':
      {
        cout << "Send motor profile" << endl;;

	char choice;
	cout << "1 - Read profiles from a file" << endl
	     << "2 - Read profiles from keyboard" << endl;
	cin >> choice;

	switch(choice) {
	case '1':
	  {
	    cout << "File format" << endl;
	    cout << "[left_front_drive_profile] [right_front_drive_profile] \
[left_rear_drive_profile] [right_rear_drive_profile] [left_front_steer_profile] \
[right_front_steer_profile] [left_rear_steer_profile] [right_rear_steer_profile]" << endl;
	    string fileName;
	    cout << "File name: ";
	    cin >> fileName;
	    ifstream file(fileName.c_str());
	    if (!file.good()) {
	      cerr << "Couldn't read file: " << fileName << endl;
	      break;
	    }

	    string line;
	    kn::MotorProfileVector profiles;
	    while (getline(file, line)) {
	      istringstream input(line);
	      
	      for(int i = 0; i < 8; ++i) {
		kn::MotorProfile profile;
		input >> profile.position;
		profile.position *= DEG2RAD;
		input >> profile.speed;
		profile.speed *= DEG2RAD;
		input >> profile.acc;
		profile.acc *= DEG2RAD;
		profiles.push_back(profile);
	      }
	      m_interface->profile(profiles);
	      profiles.clear();
	    }
	  }
	  break;
	case '2':
	  {
	    cout << "not working..." << endl;
	  }
	  break;
	}
      }
      break;

    case 't':
      {
        char choice;
        cout << "1 - Read trajectories from a file" << endl
             << "2 - Input from keyboard" << endl
	     << "3 - traj from loop" << endl
	     << "4 - Zero traj" << endl;
        cin >> choice;

        switch(choice) {
        case '2':
          {
            MotorInterface::TrajectoryStepVector trajs;
            
            double period;
            vector<double> periods;
            MotorInterface::TrajectoryStep traj;
	    int stepId = 0;

            cout << "Period: ";
            cin >> period;
            periods.push_back(period);
            cout << "leftFrontDrive: ";
            cin >> traj.leftFrontDrive;
            cout << "rightFrontDrive: ";
            cin >> traj.rightFrontDrive;
            cout << "leftRearDrive: ";
            cin >> traj.leftRearDrive;
            cout << "rightRearDrive: ";
            cin >> traj.rightRearDrive;
            cout << "leftFrontSteer: ";
            cin >> traj.leftFrontSteer;
            cout << "rightFrontSteer: ";
            cin >> traj.rightFrontSteer;
            cout << "leftRearSteer: ";
            cin >> traj.leftRearSteer;
            cout << "rightRearSteer: ";
            cin >> traj.rightRearSteer;

	    cout << "stepId: ";
	    cin >> stepId;

            trajs.push_back(traj);
            
            m_interface->trajectory(1, periods, trajs, stepId);
          }
          break;
        case '1':
          {
            cout << "File format" << endl;
            cout << "[period_sec] [left_front_drive_position] [right_front_drive_position] \
[left_rear_drive_position] [right_rear_drive_position] [left_front_steer_position] \
[right_front_steer_position] [left_rear_steer_position] [right_rear_steer_position]" << endl;
            string fileName;
            cout << "File name: ";
            cin >> fileName;

            ifstream file(fileName.c_str());
            if (!file.good()) {
              cerr << "Couldn't read file: " << fileName << endl;
              break;
            }

            string line;
            vector<MotorInterface::TrajectoryStep> trajectories;

            int steps = 0;
            double period;
            vector<double> periods;
            while (getline(file, line)) {
              istringstream input(line);
              MotorInterface::TrajectoryStep traj;

              input >> period;
              input >> traj.leftFrontDrive;
              input >> traj.rightFrontDrive;
              input >> traj.leftRearDrive;
              input >> traj.rightRearDrive;
              input >> traj.leftFrontSteer;
              input >> traj.rightFrontSteer;
              input >> traj.leftRearSteer;
              input >> traj.rightRearSteer;
              
              periods.push_back(period);
              trajectories.push_back(traj);
              ++steps;
            }
                
            m_interface->trajectory(steps, periods, trajectories);

          }
          break;
	case '3':
	  {
            MotorInterface::TrajectoryStepVector trajs;

            vector<double> periods;
            periods.push_back(1.0);

	    for (int i = 0; i < 500; ++i) {
	      MotorInterface::TrajectoryStep traj;
	      traj.leftFrontDrive = i*2.5;
	      traj.rightFrontDrive = i*2.5;
	      traj.leftRearDrive = i*2.5;
	      traj.rightRearDrive = i*2.5;
	      traj.leftFrontSteer = 0;
	      traj.rightFrontSteer = 0;
	      traj.leftRearSteer = 0;
	      traj.rightRearSteer = 0;

	      trajs.push_back(traj);
	      
	      m_interface->trajectory(1, periods, trajs);
	      usleep(100000);
	      trajs.clear();
	    }

	  }
	  break;
        case '4':
          {
            MotorInterface::TrajectoryStepVector trajs;
            
            double period;
            vector<double> periods;
            MotorInterface::TrajectoryStep traj;
	    int stepId = 0;

            cout << "Period: ";
            cin >> period;
            periods.push_back(period);

            traj.leftFrontDrive = 0.0;
            traj.rightFrontDrive = 0.0;
            traj.leftRearDrive = 0.0;
            traj.rightRearDrive = 0.0;
            traj.leftFrontSteer = 0.0;
            traj.rightFrontSteer = 0.0;
            traj.leftRearSteer = 0.0;
            traj.rightRearSteer = 0.0;

	    stepId = 0;

            trajs.push_back(traj);
            
            m_interface->trajectory(1, periods, trajs, stepId);
          }
          break;
        default:
          cout << "Invalid choice" << endl;
          break;
        }
      }
      break;

    case 'x':
      return 0;
    default:
      cout << "Invaild selection" << endl;
      break;
    }
    return 0;
  }
  
protected:
  MotorInterface * m_interface;
};

int main (int argc, char * argv[])
{
  Miro::ReactorTask rt;
  Miro::Log::init(argc, argv);

  MotorInterface motorsInterface;
  motorsInterface.init(argc, argv);
  motorsInterface.connect(rt.reactor());
  rt.activate();

  MotorInterfaceCommands cmds(&motorsInterface);
  cmds.svc();

  motorsInterface.fini();
  rt.shutdown(false);
  rt.wait();

  return 0;
}
