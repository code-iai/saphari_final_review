#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <saphari_msgs/Humans.h>

class HumansTfPublisher
{
  public:
    HumansTfPublisher(const ros::NodeHandle& nh) : nh_( nh )
    {}

    ~HumansTfPublisher() 
    {}

    void run()
    {
      pub_ = nh_.advertise<tf2_msgs::TFMessage>("tf", 1);
      sub_ = nh_.subscribe("humans", 1, &HumansTfPublisher::callback, this);
    }

  private:
   ros::NodeHandle nh_;
   ros::Subscriber sub_;
   ros::Publisher pub_;

   std::string get_bodypart_name(int bodypart_id)
   {
     switch (bodypart_id)
     {
       case saphari_msgs::BodyPart::HEAD:
         return "head";
       case saphari_msgs::BodyPart::NECK:
         return "neck";
       case saphari_msgs::BodyPart::TORSO:
         return "torso";
       case saphari_msgs::BodyPart::RIGHTSHOULDER:
         return "right_shoulder";
       case saphari_msgs::BodyPart::RIGHTELBOW:
         return "right_elbow";
       case saphari_msgs::BodyPart::RIGHTHAND:
         return "right_hand";
       case saphari_msgs::BodyPart::LEFTSHOULDER:
         return "left_shoulder";
       case saphari_msgs::BodyPart::LEFTELBOW:
         return "left_elbow";
       case saphari_msgs::BodyPart::LEFTHAND:
         return "left_hand";
       case saphari_msgs::BodyPart::RIGHTHIP:
         return "right_hip";
       case saphari_msgs::BodyPart::RIGHTKNEE:
         return "right_knee";
       case saphari_msgs::BodyPart::RIGHTFOOT:
         return "right_foot";
       case saphari_msgs::BodyPart::LEFTHIP:
         return "left_hip";
       case saphari_msgs::BodyPart::LEFTKNEE:
         return "left_knee";
       case saphari_msgs::BodyPart::LEFTFOOT:
         return "left_foot";
       default:
         return "unknown_bodypart";
     }
   }

   std::string get_human_name(int user_id)
   {
     return "human" + boost::lexical_cast<std::string>(user_id);
   }
          
   std::string get_child_frame_id(int user_id, int bodypart_id)
   {
     return get_human_name(user_id) + "/" + get_bodypart_name(bodypart_id);
   }

   void callback(const saphari_msgs::Humans::ConstPtr& humans)
   {
     tf2_msgs::TFMessage tf_msg;

     for(size_t i=0; i<humans->humans.size(); ++i)
       for(size_t j=0; j<humans->humans[i].bodyParts.size(); ++j)
       {
         tf_msg.transforms.push_back(humans->humans[i].bodyParts[j].tf);
         tf_msg.transforms.back().child_frame_id = get_child_frame_id(humans->humans[i].userID, 
             humans->humans[i].bodyParts[j].id);
         tf_msg.transforms.back().header.stamp = ros::Time::now();
       }
     
     pub_.publish(tf_msg);
   }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "humans_tf_publisher");

  HumansTfPublisher tf_pub(ros::NodeHandle("~"));

  tf_pub.run();

  while(ros::ok())
    ros::spin();

  return 0;
}
