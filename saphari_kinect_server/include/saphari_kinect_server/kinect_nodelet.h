#include "saphari_kinect_server/driver.h"
#include <nodelet/nodelet.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace saphari_kinect_server
{

  class KinectServerNodelet : public nodelet::Nodelet
  {
	public:

	~KinectServerNodelet();
	virtual void onInit();
	void onInitImpl ();
  
	private:

	boost::thread init_thread_;
    boost::shared_ptr<saphari_kinect_server::OpenNINode> ht;

  };

}
