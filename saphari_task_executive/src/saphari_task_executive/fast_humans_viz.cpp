#include <ros/ros.h>
#include <saphari_msgs/Humans.h>
#include <visualization_msgs/MarkerArray.h>

class FastHumansViz
{
  public:
    FastHumansViz(const ros::NodeHandle& nh) : nh_(nh) {}

    ~FastHumansViz() {}

    void run()
    {
      pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
      sub_ = nh_.subscribe("humans", 1, &FastHumansViz::callback, this);
    }

    void callback(const saphari_msgs::Humans::ConstPtr& humans)
    {
      pub_.publish(humansToMarkerArray(humans->humans));
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::string get_bodypart_name(int bodypart_id) const
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
          return "rightshoulder";
        case saphari_msgs::BodyPart::RIGHTELBOW:
          return "rightelbow";
        case saphari_msgs::BodyPart::RIGHTHAND:
          return "righthand";
        case saphari_msgs::BodyPart::LEFTSHOULDER:
          return "leftshoulder";
        case saphari_msgs::BodyPart::LEFTELBOW:
          return "leftelbow";
        case saphari_msgs::BodyPart::LEFTHAND:
          return "lefthand";
        case saphari_msgs::BodyPart::RIGHTHIP:
          return "righthip";
        case saphari_msgs::BodyPart::RIGHTKNEE:
          return "rightknee";
        case saphari_msgs::BodyPart::RIGHTFOOT:
          return "rightfoot";
        case saphari_msgs::BodyPart::LEFTHIP:
          return "lefthip";
        case saphari_msgs::BodyPart::LEFTKNEE:
          return "leftknee";
        case saphari_msgs::BodyPart::LEFTFOOT:
          return "leftfoot";
        default:
          return "unknown_bodypart";
      }
    }

    double get_bodypart_radius(int bodypart_id) const
    {
      switch (bodypart_id)
      {
        case saphari_msgs::BodyPart::RIGHTHAND:
        case saphari_msgs::BodyPart::LEFTHAND:
          return 0.13;
        case saphari_msgs::BodyPart::TORSO:
          return 0.3;
        case saphari_msgs::BodyPart::LEFTFOOT:
        case saphari_msgs::BodyPart::RIGHTFOOT:
        case saphari_msgs::BodyPart::HEAD:
          return 0.2;
        case saphari_msgs::BodyPart::NECK:
        case saphari_msgs::BodyPart::RIGHTSHOULDER:
        case saphari_msgs::BodyPart::RIGHTELBOW:
        case saphari_msgs::BodyPart::LEFTSHOULDER:
        case saphari_msgs::BodyPart::LEFTELBOW:
        case saphari_msgs::BodyPart::RIGHTHIP:
        case saphari_msgs::BodyPart::RIGHTKNEE:
        case saphari_msgs::BodyPart::LEFTHIP:
        case saphari_msgs::BodyPart::LEFTKNEE:
          return 0.15;
        default:
          return 0.1;
      }
    }

    geometry_msgs::Vector3 scalarToVector(double scalar) const
    {
      geometry_msgs::Vector3 msg;
      msg.x = scalar;
      msg.y = scalar;
      msg.z = scalar;
      return msg;
    }

    std_msgs::ColorRGBA yellow() const
    {
      std_msgs::ColorRGBA msg;
      msg.r = 1.0;
      msg.g = 224.0 / 255.0;
      msg.b = 150.0 / 255.0;
      msg.a = 1.0;

      return msg;
    }

    std::string get_human_name(int user_id) const
    {
      return "human" + boost::lexical_cast<std::string>(user_id);
    }
           
    std::string get_child_frame_id(int user_id, int bodypart_id) const
    {
      return get_human_name(user_id) + "/" + get_bodypart_name(bodypart_id);
    }

    visualization_msgs::Marker bodypartToMarker(const saphari_msgs::BodyPart& bodypart, int user_id, const ros::Time& time) const
    {
      visualization_msgs::Marker msg;

      msg.header.frame_id = bodypart.tf.header.frame_id;
      msg.header.stamp = time;
      msg.ns = get_human_name(user_id);
      msg.id = bodypart.id;
      msg.type = visualization_msgs::Marker::SPHERE;
      msg.action = visualization_msgs::Marker::ADD;
      msg.pose.position.x = bodypart.centroid.x;
      msg.pose.position.y = bodypart.centroid.y;
      msg.pose.position.z = bodypart.centroid.z;
      msg.pose.orientation.w = 1.0;
      msg.lifetime = ros::Duration(2.0);
      msg.scale = scalarToVector(get_bodypart_radius(bodypart.id));
      msg.color = yellow();

      return msg;
    }

    visualization_msgs::MarkerArray humansToMarkerArray(const std::vector<saphari_msgs::Human>& humans)
    {
      ros::Time time = ros::Time::now();

      visualization_msgs::MarkerArray msg;
      for(size_t i=0; i<humans.size(); ++i)
        for(size_t j=0; j<humans[i].bodyParts.size(); ++j)
          msg.markers.push_back(bodypartToMarker(humans[i].bodyParts[j], humans[i].userID, time));

      return msg;
    }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fast_humans_viz");

  FastHumansViz humans_viz(ros::NodeHandle("~"));

  humans_viz.run();

  while(ros::ok())
    ros::spin();

  return 0;
}
