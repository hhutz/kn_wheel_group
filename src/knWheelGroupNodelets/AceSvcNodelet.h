#ifndef AceSvcNodelet_h
#define AceSvcNodelet_h

#include <nodelet/nodelet.h>

#include <ace/Service_Object.h>

#include <boost/shared_ptr.hpp>

namespace kn
{
  class AceSvcNodeletBase : public nodelet::Nodelet
  {
  public:
    virtual ~AceSvcNodeletBase();

  protected:
    void genericInit();

    boost::shared_ptr<ACE_Service_Object> m_service;

  };

  template<class DerivedService>
  class AceSvcNodelet : public AceSvcNodeletBase
  {
  public:
     virtual void onInit() {
      m_service.reset(new DerivedService());
      genericInit();
    }
  };
}

#endif // AceSvcNodelet_h
