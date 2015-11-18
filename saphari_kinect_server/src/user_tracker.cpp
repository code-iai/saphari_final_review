#include "saphari_kinect_server/user_tracker.h"

using namespace saphari_msgs;

userTracker::userTracker(ros::NodeHandle &node):n(node) {
    sem_ = 1;
    //isTracking_ = false;
    //userTracked_ = 0;
    g_bNeedPose = false;
    g_strPose[0] = 0x0;
    //depthMD_cb.ReAdjust(640,480);

    userIDpub = n.advertise<std_msgs::Int32>("/kinect_traker/active_user_id", 2);
    tf_pub_ = n.advertise<tf::tfMessage>("/tf", 2);
    humansPub = n.advertise<saphari_msgs::Humans>("/kinect_traker/user_state",2);

    // Get parameters from launch file
    // tf
    if(n.getParam("pub_tf_human", publishTf)) {
        if(!n.getParam("tf_frame", tfRefFrame)) {
            tfRefFrame = "/openni_depth_frame";
        }
    }
    else {
        publishTf = false;
    }

    // Human state msg
    if(!n.getParam("world_frame", humStateRefFrame)) {
        humStateRefFrame = "/world";
    }

    publishHumanState = false;
    n.getParam("pub_human_state", publishHumanState);

    string calibration;
    if(!n.getParam("calibration_file", calibration)) {
        calibration = ros::package::getPath("saphari_kinect_server") + "calibration.txt";
    }

    // loadTfTransformFromFile(calibration, cameraToRobot);
    // Read kinect robot transform
    tf::TransformListener listener;
    bool transformFound = false;
    while(!transformFound && ros::ok()){
        tf::StampedTransform tmp_;
        try{
            listener.lookupTransform(humStateRefFrame, tfRefFrame, ros::Time(0), tmp_);
            cameraToRobot.setOrigin(tmp_.getOrigin());
            cameraToRobot.setBasis(tmp_.getBasis());
            transformFound = true;
         }
         catch (tf::TransformException ex){
           ROS_WARN("%s",ex.what());
           ros::Duration(1.0).sleep();
         }
    }

    emptyHm.header.frame_id = humStateRefFrame;
    emptyHm.header.seq      = 0;
    emptyHm.header.stamp    = ros::Time::now();
    emptyHm.userID          = 0;
    BodyPart emptyBp;
    for(int i=0; i<25; ++i) {
        emptyBp.id = i;
        emptyHm.bodyParts.push_back(emptyBp);
    }
    emptyBp.id = BodyPart::HEAD;
    emptyHm.bodyParts.push_back(emptyBp);
    emptyBp.id = BodyPart::TORSO;
    emptyHm.bodyParts.push_back(emptyBp);
    emptyBp.id = BodyPart::RIGHTSHOULDER;
    emptyHm.bodyParts.push_back(emptyBp);
    emptyBp.id = BodyPart::LEFTSHOULDER;
    emptyHm.bodyParts.push_back(emptyBp);

    tf_msg_.transforms.resize(20);
}


userTracker::~userTracker() {
    //g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Context.Shutdown();
}


void userTracker::loadTfTransformFromFile(string file, tf::Transform &transform) {
    ifstream in(file.data());
    if (!in) {
        cout << "No file found: " << file << endl;

        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

        return;
    }

    int columns = 6;
    double tmpVec[6];

    for (int j = 0; j < columns; j++) {
        in >> tmpVec[j];
    }
    in.close();

    // translation
    transform.setOrigin(tf::Vector3(tmpVec[0], tmpVec[1], tmpVec[2]));

    // rotation vector (Rodriguez)
    tf::Vector3 tmpTrans(tmpVec[3], tmpVec[4], tmpVec[5]);
    tf::Quaternion tmpQuat;
    if(tmpTrans.length() > 1e-6)
        tmpQuat.setRotation(tmpTrans.normalized(), tmpTrans.length());
    else {
        tmpQuat.setX(0.); tmpQuat.setY(0.); tmpQuat.setZ(0.); tmpQuat.setW(1.);
    }

    transform.setRotation(tmpQuat);

    return;
}


void userTracker::initUserTracker() {
        XnStatus nRetVal = XN_STATUS_OK;

        xn::EnumerationErrors errors;
        std::string path = ros::package::getPath("saphari_kinect_server");
        path += "/SamplesConfig.xml";       
        nRetVal = g_Context.InitFromXmlFile(path.c_str());
        if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
            XnChar strError[1024];
            errors.ToString(strError, 1024);
            printf("%s\n", strError);
            exit (0);
        }
        else if (nRetVal != XN_STATUS_OK) {
            printf("Open failed: %s\n", xnGetStatusString(nRetVal));
            exit (1);
        }

        //nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
        //CHECK_RC(nRetVal, "Find depth generator");

        nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
        if (nRetVal != XN_STATUS_OK) {
            nRetVal = g_UserGenerator.Create(g_Context);
            CHECK_RC(nRetVal, "Find user generator");
        }

        nRetVal = registerCall();
        CHECK_RC(nRetVal, "RegisterCall");

        g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

        nRetVal = g_Context.StartGeneratingAll();
        CHECK_RC(nRetVal, "StartGenerating");

        userMainLoop(tfRefFrame);
}


void userTracker::publishTransform(XnUserID const& user,
                                   XnSkeletonJoint joint,
                                   std::string const& frame_id,
                                   std::string const& child_frame_id,
                                   int link_indx)
{
    //static tf::TransformBroadcaster br;

    //XnSkeletonJointPosition joint_position;
    //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);

    XnSkeletonJointTransformation joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(user, joint, joint_position);
    
    // Put x = -x seems to be the reason for left and right exchange
   // double x = joint_position.position.X / 1000.0;
    //double y = -joint_position.position.Y / 1000.0;
    //double z = joint_position.position.Z / 1000.0;

    double x = joint_position.position.position.X / 1000.0;
    double y = -joint_position.position.position.Y / 1000.0;
    double z = joint_position.position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;

    tf::Matrix3x3 rotation(m[0], m[1], m[2],
                         m[3], m[4], m[5],
                         m[6], m[7], m[8]);

    //double qx, qy, qz, qw;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    // Put qy, qz = -qy, -qz seems to be the reason for left and right exchange
    transform.setBasis(rotation);

    geometry_msgs::TransformStamped tmp_trans;
    tmp_trans.child_frame_id = child_frame_id;
    tmp_trans.header.frame_id = frame_id;
    tmp_trans.header.stamp = ros::Time::now();
    tmp_trans.transform.translation.x = transform.getOrigin().getX();
    tmp_trans.transform.translation.y = transform.getOrigin().getY();
    tmp_trans.transform.translation.z = transform.getOrigin().getZ();
    tmp_trans.transform.rotation.w = transform.getRotation().getW();
    tmp_trans.transform.rotation.x = transform.getRotation().getX();
    tmp_trans.transform.rotation.y = transform.getRotation().getY();
    tmp_trans.transform.rotation.z = transform.getRotation().getZ();

    tf_msg_.transforms[link_indx] = tmp_trans;
}


void userTracker::publishTransforms(std::string const& frame_id) {
    XnUserID closestUserId = getClosestUser();
    // sleep(2);

    XnUserID users[MAX_USERS];
    XnUInt16 users_count = MAX_USERS;
    g_UserGenerator.GetUsers(users, users_count);
    XnUserID user = 0;

    for (int i = 0; i < users_count; ++i) {
        user = users[i];
        if(!g_UserGenerator.GetSkeletonCap().IsTracking(user)) {
            // Remove user from human state message
            for(int i=0; i<humansMsg.observed_user_ids.size(); ++i) {
                if(humansMsg.observed_user_ids[i] == user) {
                    humansMsg.observed_user_ids.erase(humansMsg.observed_user_ids.begin()+i);
                    humansMsg.humans.erase(humansMsg.humans.begin()+i);

                    break;
                }
            }
        }
        else { //(user > 0 && g_UserGenerator.GetSkeletonCap().IsTracking(user)) {
            // Num To String
            stringstream strm;
            string strNum;
            strm << user;
            strm >> strNum;

            // Publish a tf/tfMessage instead of tfBroadcast in order to avoid
            // 500 Hz publication rate

            publishTransform(user, XN_SKEL_HEAD,  frame_id, "kinect/head_" + strNum, 0);
            publishTransform(user, XN_SKEL_NECK,  frame_id, "kinect/neck_" + strNum, 1);
            publishTransform(user, XN_SKEL_TORSO, frame_id, "kinect/torso_" + strNum, 2);
            publishTransform(user, XN_SKEL_WAIST, frame_id, "kinect/waist_" + strNum, 3);

            publishTransform(user, XN_SKEL_LEFT_SHOULDER, frame_id, "kinect/right_shoulder_" + strNum, 4);
            publishTransform(user, XN_SKEL_LEFT_ELBOW,    frame_id, "kinect/right_elbow_" + strNum, 5);
            publishTransform(user, XN_SKEL_LEFT_WRIST,    frame_id, "kinect/right_wrist_" + strNum, 6);
            publishTransform(user, XN_SKEL_LEFT_HAND,     frame_id, "kinect/right_hand_" + strNum, 7);

            publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "kinect/left_shoulder_" + strNum, 8);
            publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "kinect/left_elbow_" + strNum, 9);
            publishTransform(user, XN_SKEL_RIGHT_WRIST,    frame_id, "kinect/left_wrist_" + strNum, 10);
            publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "kinect/left_hand_" + strNum, 11);

            publishTransform(user, XN_SKEL_LEFT_HIP,  frame_id, "kinect/right_hip_" + strNum, 12);
            publishTransform(user, XN_SKEL_LEFT_KNEE, frame_id, "kinect/right_knee_" + strNum, 13);
            publishTransform(user, XN_SKEL_LEFT_ANKLE, frame_id, "kinect/right_ankle_" + strNum, 14);
            publishTransform(user, XN_SKEL_LEFT_FOOT, frame_id, "kinect/right_foot_" + strNum, 15);

            publishTransform(user, XN_SKEL_RIGHT_HIP,  frame_id, "kinect/left_hip_" + strNum, 16);
            publishTransform(user, XN_SKEL_RIGHT_KNEE, frame_id, "kinect/left_knee_" + strNum, 17);
            publishTransform(user, XN_SKEL_RIGHT_ANKLE, frame_id, "kinect/left_ankle_" + strNum, 18);
            publishTransform(user, XN_SKEL_RIGHT_FOOT, frame_id, "kinect/left_foot_" + strNum, 19);

            if(publishTf && user==closestUserId) {
                tf_pub_.publish(tf_msg_);
            }

            // Store all human data
            if(publishHumanState) {
                storeHumansData(user);
            }
        }

        // Publish tracked user ID (problems if more then 1 users)
        /*std_msgs::Int32 msg;
        msg.data = user;
        userIDpub.publish(msg);*/
    }

    if(publishHumanState) {
        humansPub.publish(humansMsg);
    }
}


void userTracker::storeHumansData(XnUserID user) {
    for(int i=0; i<humansMsg.observed_user_ids.size(); ++i) {
        if(humansMsg.observed_user_ids[i] == user) {
            humansMsg.humans[i].header.stamp = ros::Time::now();
            humansMsg.humans[i].header.seq++;
            humansMsg.humans[i].userID = user;

            // Body Parts
            int HEAD = 25;
            humansMsg.humans[i].bodyParts[HEAD].tf = tf_msg_.transforms[0];
            humansMsg.humans[i].bodyParts[HEAD].centroid.x = (float)tf_msg_.transforms[0].transform.translation.x;
            humansMsg.humans[i].bodyParts[HEAD].centroid.z = (float)tf_msg_.transforms[0].transform.translation.y;
            humansMsg.humans[i].bodyParts[HEAD].centroid.y = (float)tf_msg_.transforms[0].transform.translation.z;

            humansMsg.humans[i].bodyParts[BodyPart::NECK].tf = tf_msg_.transforms[1];
            humansMsg.humans[i].bodyParts[BodyPart::NECK].centroid.x = (float)tf_msg_.transforms[1].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::NECK].centroid.z = (float)tf_msg_.transforms[1].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::NECK].centroid.y = (float)tf_msg_.transforms[1].transform.translation.z;

            int TORSO = 26;
            humansMsg.humans[i].bodyParts[TORSO].tf = tf_msg_.transforms[2];
            humansMsg.humans[i].bodyParts[TORSO].centroid.x = (float)tf_msg_.transforms[2].transform.translation.x;
            humansMsg.humans[i].bodyParts[TORSO].centroid.z = (float)tf_msg_.transforms[2].transform.translation.y;
            humansMsg.humans[i].bodyParts[TORSO].centroid.y = (float)tf_msg_.transforms[2].transform.translation.z;

            // WAIST (3) NOT IN BODYPARTS

            int LEFTSHOULDER = 28;
            humansMsg.humans[i].bodyParts[LEFTSHOULDER].tf = tf_msg_.transforms[4];
            humansMsg.humans[i].bodyParts[LEFTSHOULDER].centroid.x = (float)tf_msg_.transforms[4].transform.translation.x;
            humansMsg.humans[i].bodyParts[LEFTSHOULDER].centroid.z = (float)tf_msg_.transforms[4].transform.translation.y;
            humansMsg.humans[i].bodyParts[LEFTSHOULDER].centroid.y = (float)tf_msg_.transforms[4].transform.translation.z;

            humansMsg.humans[i].bodyParts[BodyPart::LEFTELBOW].tf = tf_msg_.transforms[5];
            humansMsg.humans[i].bodyParts[BodyPart::LEFTELBOW].centroid.x = (float)tf_msg_.transforms[5].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTELBOW].centroid.z = (float)tf_msg_.transforms[5].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTELBOW].centroid.y = (float)tf_msg_.transforms[5].transform.translation.z;

            // Considering XN_SKEL_LEFT_WRIST as BodyPart::LEFTFOREARM
            /*humansMsg.humans[i].bodyParts[BodyPart::LEFTFOREARM].tf = tf_msg_.transforms[6];
            humansMsg.humans[i].bodyParts[BodyPart::LEFTFOREARM].centroid.x = (float)tf_msg_.transforms[6].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTFOREARM].centroid.z = (float)tf_msg_.transforms[6].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTFOREARM].centroid.y = (float)tf_msg_.transforms[6].transform.translation.z;*/

            humansMsg.humans[i].bodyParts[BodyPart::LEFTHAND].tf = tf_msg_.transforms[7];
            humansMsg.humans[i].bodyParts[BodyPart::LEFTHAND].centroid.x = (float)tf_msg_.transforms[7].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTHAND].centroid.z = (float)tf_msg_.transforms[7].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTHAND].centroid.y = (float)tf_msg_.transforms[7].transform.translation.z;

            int RIGHTSHOULDER = 27;
            humansMsg.humans[i].bodyParts[RIGHTSHOULDER].tf = tf_msg_.transforms[8];
            humansMsg.humans[i].bodyParts[RIGHTSHOULDER].centroid.x = (float)tf_msg_.transforms[8].transform.translation.x;
            humansMsg.humans[i].bodyParts[RIGHTSHOULDER].centroid.z = (float)tf_msg_.transforms[8].transform.translation.y;
            humansMsg.humans[i].bodyParts[RIGHTSHOULDER].centroid.y = (float)tf_msg_.transforms[8].transform.translation.z;

            humansMsg.humans[i].bodyParts[BodyPart::RIGHTELBOW].tf = tf_msg_.transforms[9];
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTELBOW].centroid.x = (float)tf_msg_.transforms[9].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTELBOW].centroid.z = (float)tf_msg_.transforms[9].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTELBOW].centroid.y = (float)tf_msg_.transforms[9].transform.translation.z;

            // Considering XN_SKEL_RIGHT_WRIST as BodyPart::RIGHTFOREARM
           /* humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOREARM].tf = tf_msg_.transforms[10];
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOREARM].centroid.x = (float)tf_msg_.transforms[10].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOREARM].centroid.z = (float)tf_msg_.transforms[10].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOREARM].centroid.y = (float)tf_msg_.transforms[10].transform.translation.z;*/

            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHAND].tf = tf_msg_.transforms[11];
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHAND].centroid.x = (float)tf_msg_.transforms[11].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHAND].centroid.z = (float)tf_msg_.transforms[11].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHAND].centroid.y = (float)tf_msg_.transforms[11].transform.translation.z;

            humansMsg.humans[i].bodyParts[BodyPart::LEFTHIP].tf = tf_msg_.transforms[12];
            humansMsg.humans[i].bodyParts[BodyPart::LEFTHIP].centroid.x = (float)tf_msg_.transforms[12].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTHIP].centroid.z = (float)tf_msg_.transforms[12].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTHIP].centroid.y = (float)tf_msg_.transforms[12].transform.translation.z;

            humansMsg.humans[i].bodyParts[BodyPart::LEFTKNEE].tf = tf_msg_.transforms[13];
            humansMsg.humans[i].bodyParts[BodyPart::LEFTKNEE].centroid.x = (float)tf_msg_.transforms[13].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTKNEE].centroid.z = (float)tf_msg_.transforms[13].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTKNEE].centroid.y = (float)tf_msg_.transforms[13].transform.translation.z;

            // Considering XN_SKEL_LEFT_ANKLE as BodyPart::LEFTLEG
            /* humansMsg.humans[i].bodyParts[BodyPart::LEFTLEG].tf = tf_msg_.transforms[14];
            humansMsg.humans[i].bodyParts[BodyPart::LEFTLEG].centroid.x = (float)tf_msg_.transforms[14].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTLEG].centroid.z = (float)tf_msg_.transforms[14].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTLEG].centroid.y = (float)tf_msg_.transforms[14].transform.translation.z;*/

            humansMsg.humans[i].bodyParts[BodyPart::LEFTFOOT].tf = tf_msg_.transforms[15];
            humansMsg.humans[i].bodyParts[BodyPart::LEFTFOOT].centroid.x = (float)tf_msg_.transforms[15].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTFOOT].centroid.z = (float)tf_msg_.transforms[15].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::LEFTFOOT].centroid.y = (float)tf_msg_.transforms[15].transform.translation.z;


            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHIP].tf = tf_msg_.transforms[16];
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHIP].centroid.x = (float)tf_msg_.transforms[16].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHIP].centroid.z = (float)tf_msg_.transforms[16].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTHIP].centroid.y = (float)tf_msg_.transforms[16].transform.translation.z;

            humansMsg.humans[i].bodyParts[BodyPart::RIGHTKNEE].tf = tf_msg_.transforms[17];
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTKNEE].centroid.x = (float)tf_msg_.transforms[17].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTKNEE].centroid.z = (float)tf_msg_.transforms[17].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTKNEE].centroid.y = (float)tf_msg_.transforms[17].transform.translation.z;

            // Considering XN_SKEL_RIGHT_ANKLE as BodyPart::RIGHTLEG
            /* humansMsg.humans[i].bodyParts[BodyPart::RIGHTLEG].tf = tf_msg_.transforms[18];
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTLEG].centroid.x = (float)tf_msg_.transforms[18].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTLEG].centroid.z = (float)tf_msg_.transforms[18].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTLEG].centroid.y = (float)tf_msg_.transforms[18].transform.translation.z;*/

            humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOOT].tf = tf_msg_.transforms[19];
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOOT].centroid.x = (float)tf_msg_.transforms[19].transform.translation.x;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOOT].centroid.z = (float)tf_msg_.transforms[19].transform.translation.y;
            humansMsg.humans[i].bodyParts[BodyPart::RIGHTFOOT].centroid.y = (float)tf_msg_.transforms[19].transform.translation.z;

            break;
        }
    }
}


XnUserID userTracker::getClosestUser() {
    XnUserID users[MAX_USERS];
    XnUInt16 users_count = MAX_USERS;
    g_UserGenerator.GetUsers(users, users_count);

    //cout << "users_count: " << users_count << endl;

    vector <float> dist;
    // Distances of all tracked users
    XnPoint3D com;
    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
    //    cout << "user: " << user << endl;
        if (g_UserGenerator.GetSkeletonCap().IsTracking(user)) {
            g_UserGenerator.GetCoM(user, com);

            float x = com.X;
            float y = com.Y;
            float z = com.Z;

            dist.push_back(std::sqrt(x*x + y*y + z*z));
        }
    }

    // closest user
    int dim_ = (int)dist.size();
   // cout << "dim_: " << dim_ << endl;
    int minInd;
    if(dim_ > 0) {
        minInd = users[0];
        float minVal = dist[0];
        for(int j = 1; j < dim_; ++j) {
            if(dist[j] < minVal) {
                minVal = dist[j];
                minInd = users[j];
            }
        }
    }
    else
        minInd = 0;

    // Publish closest user ID (0 if no user found)
    std_msgs::Int32 msg;
    msg.data = minInd;
    userIDpub.publish(msg);

  // cout << "minInd: " << minInd << endl;

    return (XnUserID)minInd;
}


void XN_CALLBACK_TYPE userTracker::User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    userTracker * self = (userTracker *)pCookie;
    if(nId > MAX_USERS) { // || self->isTracking_) {
        self->g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
        self->g_UserGenerator.GetSkeletonCap().Reset(nId);
        return;
    }
    printf("New User %d\n", nId);
    // New user found
    if (self->g_bNeedPose) {
            self->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(self->g_strPose, nId);
    }
    else {
           self->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
    }
}


void XN_CALLBACK_TYPE userTracker::User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    userTracker * self = (userTracker *)pCookie;
    //if(nId == self->userTracked_) {
        //self->isTracking_ = false;
        //self->userTracked_ = 0;
    printf("Lost user %d\n", nId);
    //}

    // Remove user from human state message
    for(int i=0; i<self->humansMsg.observed_user_ids.size(); ++i) {
        if(self->humansMsg.observed_user_ids[i] == nId) {
            self->humansMsg.observed_user_ids.erase(self->humansMsg.observed_user_ids.begin()+i);
            self->humansMsg.humans.erase(self->humansMsg.humans.begin()+i);

            break;
        }
    }
}


// Callback: Detected a pose
void XN_CALLBACK_TYPE userTracker::UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie) {
    userTracker * self = (userTracker *)pCookie;
    printf("Pose %s detected for user %d\n", strPose, nId);
    self->g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    printf("Pose detection stopped \n");
    self->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
    printf("Calibration required \n");
}


void XN_CALLBACK_TYPE userTracker::UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
    printf("Calibration started for user %d\n", nId);
}


// Callback: Finished calibration
void XN_CALLBACK_TYPE userTracker::UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
    userTracker * self = (userTracker *)pCookie;
    if (bSuccess && nId > 0) {
        // Calibration succeeded
        printf("Calibration complete, start tracking user %d\n", nId);
        self->g_UserGenerator.GetSkeletonCap().StartTracking(nId);
        //self->isTracking_ = true;
        //self->userTracked_ = nId;

        // Add to Humans msg
        self->humansMsg.observed_user_ids.push_back(nId);
        self->humansMsg.humans.push_back(self->emptyHm);
    }
    else {
        // Calibration failed
        printf("Calibration failed for user %d\n", nId);
        if (self->g_bNeedPose) {
            self->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(self->g_strPose, nId);
        }
        else {
            self->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, true);
        }
    }
}


void userTracker::userMainLoop(std::string frame_id) {
    // Uncomment to read depth_map from a ros topic
    /*ros::spinOnce();
        // Read next available data
        while(sem_==0)
            usleep(5000);
        sem_ = 0;
        if(depthMD_cb.XRes() == 0)
        {
            sem_ = 1;
            return;
        }*/

    g_Context.WaitAndUpdateAll();
    //g_DepthGenerator.GetMetaData(depthMD);

    // Uncomment to read depth_map from a ros topic
    // g_Context.WaitOneUpdateAll(g_DepthGenerator);

    // Uncomment to read depth_map from a ros topic
    /* g_DepthGenerator.GetMetaData(depthMD);
        depthMD.ReAdjust(depthMD.XRes(), depthMD.YRes());
        depthMD.MakeDataWritable();
        xn::DepthMap& depthMap_ = depthMD_cb.WritableDepthMap();
        xn::DepthMap& depthMap = depthMD.WritableDepthMap();
        // Modify depth map
        for (XnUInt y = 0; y < depthMap.YRes(); ++y)
        {
            for (XnUInt x = 0; x < depthMap.XRes(); ++x)
            {
                depthMap(x, y) = depthMap_(x, y);
            }
        }
        mockDepth.SetData(depthMD);*/

    // publish /tf transforms
    publishTransforms(frame_id);

    // Uncomment to read depth_map from a ros topic
    //sem_ = 1;
}


XnStatus userTracker::registerCall() {
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        printf("Supplied user generator doesn't support skeleton\n");
        return -1;
    }
    
    XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseDetected;
    
    XnStatus nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, this, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart,
                                                                            UserCalibration_CalibrationEnd,
                                                                            this,
                                                                            hCalibrationCallbacks);
    CHECK_RC(nRetVal, "Register to calibration complete");

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = true;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
            printf("Pose required, but not supported\n");
            return -1;
        }
        nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, this, hPoseDetected);
        CHECK_RC(nRetVal, "Register to Pose Detected");
        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }
    return XN_STATUS_OK;
}
