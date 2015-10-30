#include <saphari_kinect_server/kinect_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <boost/algorithm/string/replace.hpp>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(saphari_kinect_server, KinectServerNodelet, saphari_kinect_server::KinectServerNodelet, nodelet::Nodelet)

namespace saphari_kinect_server
{
  
  KinectServerNodelet::~KinectServerNodelet()
  {

    init_thread_.interrupt();
    init_thread_.join();

  }

  void KinectServerNodelet::onInit() 
  {
    
    NODELET_INFO("Initializing kinect server nodelet...");
    init_thread_ = boost::thread(boost::bind(&KinectServerNodelet::onInitImpl, this));  
  
  }

  void KinectServerNodelet::onInitImpl ()
  {

    // Get nodelet node handle
    ros::NodeHandle nh = this->getPrivateNodeHandle();
    // Create OpenNINode
    ht.reset(new saphari_kinect_server::OpenNINode(nh));

  }

}


