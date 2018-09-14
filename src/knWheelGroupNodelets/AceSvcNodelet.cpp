#include "AceSvcNodelet.h"

#include <ace/Service_Object.h>
#include <ace/ARGV.h>

#include <ros/node_handle.h>

#include <cstring>

namespace kn
{
  AceSvcNodeletBase::~AceSvcNodeletBase()
  {
    m_service->fini();
  }

  void AceSvcNodeletBase::genericInit()
  {
    ros::NodeHandle pnh = this->getPrivateNodeHandle();
    std::string name = this->getName();
    std::string parameters;

    if (!pnh.getParam("parameters", parameters)) {
      pnh.setParam("parameters", parameters);
    }

    std::string commandLine(name + " " + parameters);
    char buffer[512];
    strncpy(buffer, commandLine.c_str(), sizeof(buffer));
    buffer[sizeof(buffer) - 1] = 0;
    char * argv[1];
    argv[0] = buffer;
    ACE_ARGV args(argv);
    m_service->init(args.argc(), args.argv());
  }
}

