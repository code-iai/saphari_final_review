#include <sys/time.h>

#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/String.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sstream>

#include <openni_camera/openni_depth_image.h>
#include <openni_camera/openni_driver.h>
//#include <openni_camera/openni_image.h>
#include <openni_camera/openni_image_bayer_grbg.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include "saphari_kinect_server/user_tracker.h"

namespace saphari_kinect_server
{
    class OpenNINode
    {
        public:

        OpenNINode (ros::NodeHandle &nh);
        ~OpenNINode ();

        void runLoop ();
        void init (float minUserDist, float maxUserDist);
        void setupDevice();
        void depthCb1(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);
        void depthCb2(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);
        void publishDepthImage(const openni_wrapper::DepthImage& depth, image_transport::CameraPublisher depth_pub) const;
        void publishDepthImage1(const openni_wrapper::DepthImage& depth, ros::Time time) const;
        void publishDepthImage2(const openni_wrapper::DepthImage& depth, ros::Time time) const;

        void imgCb1(boost::shared_ptr<openni_wrapper::Image> rgb_image, void* cookie);
        void imgCb2(boost::shared_ptr<openni_wrapper::Image> rgb_image, void* cookie);
        void publishRgbImage(const openni_wrapper::Image& depth, image_transport::CameraPublisher img_pub) const;


        sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) const;
        sensor_msgs::CameraInfoPtr getDepthCameraInfo(int width, int height, ros::Time time, int camera_id) const;

        protected:
    
        int depth_width_;
        int depth_height_;
        std::string serial_number;
        std::string depth_frame_id_;
        std::string tfRefFrame;

        // ROS stuff
        ros::NodeHandle nh_;
        ros::Publisher pub;
        sensor_msgs::CameraInfoPtr camera_info_1;
        sensor_msgs::CameraInfoPtr camera_info_2;

        image_transport::CameraPublisher depth_pub_1;
        image_transport::CameraPublisher depth_pub_2;

        image_transport::CameraPublisher rgb_pub_1;
        image_transport::CameraPublisher rgb_pub_2;

        boost::shared_ptr<camera_info_manager::CameraInfoManager> ir_info_manager_1;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> ir_info_manager_2;

        boost::shared_ptr<openni_wrapper::OpenNIDevice> device_1;
        boost::shared_ptr<openni_wrapper::OpenNIDevice> device_2;

        userTracker userTr;

    };

}
