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

#include "ace/Log_Msg.h"
#include "ace/SOCK_Dgram.h"
#include "ace/INET_Addr.h"

#include <pthread.h>

#include <iostream>
#include <sstream>

using namespace std;

/* Use the typical TCP/IP port address for receiving datagrams.  */
static const u_short IN_PORT = 5555;
static const u_short OUT_PORT = 4444;

// I know this is not good, just needed a fast way to write up the test server...
ACE_INET_Addr g_local((u_short)0);
ACE_SOCK_Dgram_Bcast g_client;
ACE_INET_Addr g_remote(IN_PORT, INADDR_BROADCAST);

// Thread to send vehicle ACK
void * ackThread(void * args)
{
  cout << "start ack thread..." << endl;

  unsigned int ackId = 0;

  char buf[128];

  while (true) {
    memset(buf, 0, 127);
    sprintf(buf, "%s %d\n", MOTOR_V_ACK_CMD, ackId++);
    if (g_client.send(buf,
                      ACE_OS::strlen (buf) + 1,
                      g_remote) == -1) {
      cout << "send failed" << endl;
    }
    else {
      cout << "Sent: " << buf;
    }
    sleep(2);
  }

  return 0;
}

// Thread to send Rocker status
void * statusThread(void * args)
{
  cout << "start status thread..." << endl;

  int cmdId = -1;
  int numMotors = 4;
  int timestamp = 4324114;

  kn::protoMotor::MotorInterface::RockerStatus rockerStatus;

  memset(&rockerStatus, 0, sizeof(kn::protoMotor::MotorInterface::RockerStatus));

  char buf[BUFSIZ];

  while (true) {
    memset(buf, 0, BUFSIZ);
    char side = ((timestamp % 2) == 0) ? 'L' : 'R';
    ostringstream message;
    message << MOTOR_STATUS_CMD << " " << side << " " << cmdId << " ";
    for (int i = 0; i < numMotors; ++i) {
      message << rockerStatus.motorStatus[i].current << " ";
      message << rockerStatus.motorStatus[i].speed << " ";
      message << rockerStatus.motorStatus[i].position << " ";
      message << rockerStatus.motorStatus[i].status << " ";
    }

    message << "15.5" << " " << timestamp++ << "\n";
    strcpy(buf, message.str().c_str());
    if (g_client.send(buf,
                      ACE_OS::strlen (buf) + 1,
                      g_remote) == -1) {
      cout << "send failed" << endl;
    }
    else {
      cout << "Sent: " << buf;
    }
    sleep(2);
  }

  return NULL;
}

int
main (int, char**)
{
  ACE_INET_Addr local(OUT_PORT, INADDR_BROADCAST);

  g_client.open(g_local);

  ACE_SOCK_Dgram_Bcast dgram;

  if (dgram.open (local) == -1)
    ACE_ERROR_RETURN ((LM_ERROR,
                       "%p\n",
                       "open"),
                      -1);

  char buf[BUFSIZ];

  ACE_INET_Addr remote;
  
  ACE_DEBUG ((LM_DEBUG,
              "(%P|%t) starting up server daemon\n"));

  pthread_t thread1, thread2;
  pthread_create(&thread1, NULL, ackThread, NULL);
  pthread_create(&thread2, NULL, statusThread, NULL);

  while(dgram.recv (buf,
                    sizeof (buf),
                    remote) != -1)
  {
    cout << "---> receive message: " << buf;

    string buffer(buf);

    size_t found;
    found = buffer.find(MOTOR_ARC_CMD);
    if (found != string::npos) {
      cout << "Got ARC cmd: " << buffer << endl;
      found += ACE_OS::strlen(MOTOR_ARC_CMD) + 1;
      memset(buf, 0, BUFSIZ);
      string ackString = buffer.substr(found, buffer.length() -1);
      int cmdId = atoi(ackString.c_str());
      sprintf(buf, "%s %d OK\n", MOTOR_CMD_ACK_CMD, cmdId);
    }
    
    if (g_client.send (buf,
                     ACE_OS::strlen (buf) + 1,
                     g_remote) == -1)
      ACE_ERROR_RETURN ((LM_ERROR,
                         "%p\n",
                         "send"),
                        -1);
  }

  pthread_join(thread1, NULL);
  pthread_join(thread2, NULL);
  return 0;
}
