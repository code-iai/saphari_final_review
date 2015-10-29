#include "saphari_kinect_server/driver.h"
#include <boost/algorithm/string/replace.hpp>
#include <sensor_msgs/distortion_models.h>

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
	}
		//return nRetVal

using namespace std;
using namespace saphari_kinect_server;
using namespace openni_wrapper;

namespace enc = sensor_msgs::image_encodings;

  OpenNINode::OpenNINode (ros::NodeHandle &nh)
    : nh_ (nh)
    , depth_width_ (640)
    , depth_height_ (480)
    , userTr(nh)
  {
      setupDevice();
  }//OpenNINode


  void OpenNINode::setupDevice() {
 	OpenNIDriver& driver = OpenNIDriver::getInstance ();

    int num_dev;
    nh_.getParam("num_dev",num_dev);

    do {
	  driver.updateDeviceList ();

	  if (driver.getNumberDevices () == 0)
	  {
	    ROS_INFO ("No devices connected.... waiting for devices to be connected");
	    boost::this_thread::sleep(boost::posix_time::seconds(3));
	    continue;
	  }

	  ROS_INFO ("Number devices connected: %d", driver.getNumberDevices ());
      for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx) {
	    ROS_INFO("%u. device on bus %03u:%02u is a %s (%03x) from %s (%03x) with serial id \'%s\'",
		         deviceIdx + 1, driver.getBus(deviceIdx), driver.getAddress(deviceIdx),
		         driver.getProductName(deviceIdx), driver.getProductID(deviceIdx),
		         driver.getVendorName(deviceIdx), driver.getVendorID(deviceIdx),
		         driver.getSerialNumber(deviceIdx));
	  }

	  try {
        std::string device_id1;
        std::string device_id2;


        if ((!nh_.getParam("device_id1", device_id1)) || (!nh_.getParam("device_id2", device_id2)))
	    {
	      ROS_WARN ("~device_id is not set! Using first device.");
          device_1 = driver.getDeviceByIndex (0);
	    }
        else if (device_id1[0] == '#' && device_id2[0] == '#')
	    {
          unsigned index = atoi (device_id1.c_str () + 1);
          ROS_INFO ("Searching for device 1 with index = %d", index);
          device_1 = driver.getDeviceByIndex (index - 1);

          if(num_dev == 2) {
              if(driver.getNumberDevices () != 2) {
                  ROS_WARN("Kinect 2 is not connected. Press Ctrl-C..");
                  return;
              }

              else {
                  unsigned index = atoi (device_id2.c_str () + 1);
                  ROS_INFO ("Searching for device 2 with index = %d", index);
                  device_2 = driver.getDeviceByIndex (index - 1);
              }
          }

	    }
	    else
	    {
          ROS_INFO ("Searching for device 1 with serial number = '%s'", device_id1.c_str ());
          device_1 = driver.getDeviceBySerialNumber (device_id1);

          if(num_dev == 2)
          {
              ROS_INFO ("Searching for device 2 with serial number = '%s'", device_id2.c_str ());
              device_2 = driver.getDeviceBySerialNumber (device_id2);
          }
	    }
	  }
	  catch (const OpenNIException& exception)
	  {
          if (!device_1)
	    {
              ROS_INFO ("No matching device found.... waiting for devices. Reason: %s", exception.what ());
              boost::this_thread::sleep(boost::posix_time::seconds(3));
              continue;
	    }
	    else
	    {
              ROS_ERROR ("Could not retrieve device. Reason: %s", exception.what ());
              exit (-1);
	    }
      }
    } while (!device_1);

    ROS_INFO ("Opened '%s' on bus %d:%d with serial number '%s'", device_1->getProductName (),
              device_1->getBus (), device_1->getAddress (), device_1->getSerialNumber ());
    if(num_dev == 2)
    {
        ROS_INFO ("Opened '%s' on bus %d:%d with serial number '%s'", device_2->getProductName (),
              device_2->getBus (), device_2->getAddress (), device_2->getSerialNumber ());
    }

    device_1->registerDepthCallback(&OpenNINode::depthCb1, *this);
    device_1->registerImageCallback(&OpenNINode::imgCb1, *this);
    if(device_2) {
        device_2->registerDepthCallback(&OpenNINode::depthCb2, *this);
        device_2->registerImageCallback(&OpenNINode::imgCb2, *this);
    }

    init();

  }

  // Depth publish
  void OpenNINode::depthCb1(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie) {
    publishDepthImage(*depth_image, depth_pub_1);

    userTr.userMainLoop("/camera_depth_frame");
  }

  void OpenNINode::depthCb2(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie) {
    publishDepthImage(*depth_image, depth_pub_2);
  }

  // Image publish
  void OpenNINode::imgCb1(boost::shared_ptr<openni_wrapper::Image> rgb_image, void* cookie) {
      publishRgbImage(*rgb_image, rgb_pub_1);
  }

  void OpenNINode::imgCb2(boost::shared_ptr<openni_wrapper::Image> rgb_image, void* cookie) {
      publishRgbImage(*rgb_image, rgb_pub_2);
  }


  void OpenNINode::init()
  {

    std::string serial_number = device_1->getSerialNumber();
	std::string ir_name, ir_info_url;
	if (serial_number.empty())
	{
	  ir_name  = "depth";
	  ir_info_url = "";
	}
	else
	{
	  ir_name  = "depth_" + serial_number;
	  ir_info_url = "";
	}


    ir_info_manager_1  = boost::make_shared<camera_info_manager::CameraInfoManager>(nh_,  ir_name,  ir_info_url);
	
    if (!ir_info_manager_1->isCalibrated())
      ROS_WARN("Using default parameters for IR camera 1 calibration.");

	  
	  image_transport::ImageTransport depth_it(nh_);
      depth_pub_1 = depth_it.advertiseCamera("depth_output1", 1);
      rgb_pub_1 = depth_it.advertiseCamera("rgb_output1", 1);

      if(device_2) {
          std::string serial_number = device_2->getSerialNumber();
          std::string ir_name, ir_info_url;
          if (serial_number.empty()) {
            ir_name  = "depth";
            ir_info_url = "";
          }
          else {
            ir_name  = "depth_" + serial_number;
            ir_info_url = "";
          }

          ir_info_manager_2  = boost::make_shared<camera_info_manager::CameraInfoManager>(nh_,  ir_name,  ir_info_url);

          if (!ir_info_manager_2->isCalibrated())
            ROS_WARN("Using default parameters for IR camera 2 calibration.");


            //image_transport::ImageTransport depth_it(nh_);
            depth_pub_2 = depth_it.advertiseCamera("depth_output2", 1);
            rgb_pub_2   = depth_it.advertiseCamera("rgb_output2", 1);
      }

      userTr.initUserTracker();

	  runLoop();

  }


  OpenNINode::~OpenNINode() {
    if (device_1)
      device_1->shutdown();

    if (device_2)
      device_2->shutdown();

  }//~OpenNINode


  void OpenNINode::runLoop() {
	ROS_INFO("main loop");
    device_1->startDepthStream();
    device_1->startImageStream();
    if(device_2) {
        device_2->startDepthStream();
        device_2->startImageStream();
    }

    //userTr.userMainLoop("/camera_depth_frame");

  }//runLoop

  void OpenNINode::publishRgbImage(const openni_wrapper::Image& rgb_img, image_transport::CameraPublisher img_pub) const {
      sensor_msgs::ImagePtr rgb_msg = boost::make_shared<sensor_msgs::Image>();
      ros::Time time = ros::Time::now();
      rgb_msg->header.stamp    = time;
      rgb_msg->encoding        = enc::RGB8;
      rgb_msg->width           = depth_width_;
      rgb_msg->height          = depth_height_;
      rgb_msg->step            = rgb_msg->width * 3;
      rgb_msg->data.resize(rgb_msg->height * rgb_msg->step);

      rgb_img.fillRGB(rgb_msg->width, rgb_msg->height, reinterpret_cast<unsigned char*>(&rgb_msg->data[0]), rgb_msg->step);

      img_pub.publish(rgb_msg, getDepthCameraInfo(rgb_msg->width, rgb_msg->height, time, 1));
    }

  void OpenNINode::publishDepthImage(const openni_wrapper::DepthImage& depth, image_transport::CameraPublisher depth_pub) const {
      sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
      ros::Time time = ros::Time::now();
      depth_msg->header.stamp    = time;
      depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_msg->width           = depth_width_;
      depth_msg->height          = depth_height_;
      depth_msg->step            = depth_msg->width * sizeof(float);
      depth_msg->data.resize(depth_msg->height * depth_msg->step);

      depth.fillDepthImage(depth_msg->width, depth_msg->height, reinterpret_cast<float*>(&depth_msg->data[0]), depth_msg->step);

      depth_pub.publish(depth_msg, getDepthCameraInfo(depth_msg->width, depth_msg->height, time, 1));
      //pub.publish(depth_msg);
    }


  /*void OpenNINode::publishDepthImage1(const openni_wrapper::DepthImage& depth, ros::Time time) const
  {
	sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
	depth_msg->header.stamp    = time;
	depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
	depth_msg->width           = depth_width_;
	depth_msg->height          = depth_height_;
	depth_msg->step            = depth_msg->width * sizeof(float);
	depth_msg->data.resize(depth_msg->height * depth_msg->step);

	depth.fillDepthImage(depth_msg->width, depth_msg->height, reinterpret_cast<float*>(&depth_msg->data[0]), depth_msg->step);

    depth_pub_1.publish(depth_msg, getDepthCameraInfo(depth_msg->width, depth_msg->height, time, 1));
	//pub.publish(depth_msg);
  }

  void OpenNINode::publishDepthImage2(const openni_wrapper::DepthImage& depth, ros::Time time) const
  {
    sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
    depth_msg->header.stamp    = time;
    depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_msg->width           = depth_width_;
    depth_msg->height          = depth_height_;
    depth_msg->step            = depth_msg->width * sizeof(float);
    depth_msg->data.resize(depth_msg->height * depth_msg->step);

    depth.fillDepthImage(depth_msg->width, depth_msg->height, reinterpret_cast<float*>(&depth_msg->data[0]), depth_msg->step);

    depth_pub_2.publish(depth_msg, getDepthCameraInfo(depth_msg->width, depth_msg->height, time, 2));
    //pub.publish(depth_msg);
  }*/

  sensor_msgs::CameraInfoPtr OpenNINode::getDefaultCameraInfo(int width, int height, double f) const
  {
	sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

	info->width  = width;
	info->height = height;

	// No distortion
	info->D.resize(5, 0.0);
	info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	// Simple camera matrix: square pixels (fx = fy), principal point at center
	info->K.assign(0.0);
	info->K[0] = info->K[4] = f;
	info->K[2] = (width / 2) - 0.5;
	// Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
	// This formula keeps the principal point the same in VGA and SXGA modes
	info->K[5] = (width * (3./8.)) - 0.5;
	info->K[8] = 1.0;

	// No separate rectified image plane, so R = I
	info->R.assign(0.0);
	info->R[0] = info->R[4] = info->R[8] = 1.0;

	// Then P=K(I|0) = (K|0)
	info->P.assign(0.0);
	info->P[0]  = info->P[5] = f; // fx, fy
	info->P[2]  = info->K[2];     // cx
	info->P[6]  = info->K[5];     // cy
	info->P[10] = 1.0;

	return info;
  }

  sensor_msgs::CameraInfoPtr OpenNINode::getDepthCameraInfo(int width, int height, ros::Time time, int camera_id) const
  {
	sensor_msgs::CameraInfoPtr info;

    if(camera_id == 1)
    {
        if (ir_info_manager_1->isCalibrated())
        {
          info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_1->getCameraInfo());
          if ( info->width != width )
          {
            // Use uncalibrated values
            ROS_WARN_ONCE("Image resolution doesn't match calibration of the IR camera. Using default parameters.");
            info = getDefaultCameraInfo(width, height, device_1->getImageFocalLength(width));
          }
        }
        else
        {
          // If uncalibrated, fill in default values
          info = getDefaultCameraInfo(width, height, device_1->getDepthFocalLength(width));
        }
    }

    else if(camera_id == 2)
    {
        if (ir_info_manager_2->isCalibrated())
        {
          info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_2->getCameraInfo());
          if ( info->width != width )
          {
            // Use uncalibrated values
            ROS_WARN_ONCE("Image resolution doesn't match calibration of the IR camera. Using default parameters.");
            info = getDefaultCameraInfo(width, height, device_2->getImageFocalLength(width));
          }
        }
        else
        {
          // If uncalibrated, fill in default values
          info = getDefaultCameraInfo(width, height, device_2->getDepthFocalLength(width));
        }
    }

	// Fill in header
	info->header.stamp    = time;
	//info->header.frame_id = depth_frame_id_;

	return info;
  }














