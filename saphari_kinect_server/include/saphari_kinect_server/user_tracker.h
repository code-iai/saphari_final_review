#ifndef USERTRACKER_H
#define USERTRACKER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <fstream>
#include <string>
#include <vector>

#include <saphari_msgs/Humans.h>

#define pi_ 3.1416

#define MAX_USERS 15

#define CHECK_RC(nRetVal, what)										\
    if (nRetVal != XN_STATUS_OK)									\
    {																\
        printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
        exit (nRetVal);												\
    }

//#define USE_CALIBRATED_DATA

using namespace std;

class userTracker
{
public:
    userTracker(ros::NodeHandle &node);
    ~userTracker();
    // This function is called each frame
    void userMainLoop(std::string frame_id);

    void initUserTracker();

    void loadTfTransformFromFile(string file, tf::Transform &transform);

    XnStatus registerCall();
// TODO: try to move something in protected or private areas
    static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie);
    static void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie);
 //  static void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie);

  //  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg_ptr);

    xn::Context g_Context;
    xn::DepthGenerator g_DepthGenerator;
    xn::UserGenerator g_UserGenerator;
    xn::MockDepthGenerator mockDepth;
    xn::DepthMetaData depthMD;
    xn::DepthMetaData depthMD_cb;

    int sem_;

    XnBool g_bNeedPose;
    XnChar g_strPose[20];

    // Human state publisher
    saphari_msgs::Humans humansMsg;
    saphari_msgs::Human emptyHm;
    ros::Publisher humansPub;

protected:
    XnUserID getClosestUser();

    void publishTransform(XnUserID const& user,
                          XnSkeletonJoint joint,
                          std::string const& frame_id,
                          std::string const& child_frame_id,
                          int link_indx = 0);

    void publishTransforms(std::string const& frame_id);

private:   
    ros::NodeHandle n;
    ros::Publisher userIDpub, tf_pub_;
    tf::tfMessage tf_msg_;

    void storeHumansData(XnUserID user);

    //bool isTracking_;
    //XnUserID userTracked_;

    bool publishTf, publishHumanState;
    std::string tfRefFrame, humStateRefFrame;
    tf::Transform cameraToRobot;

};

#endif // USERTRACKER_H
