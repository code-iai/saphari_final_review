#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <boost/bind.hpp>

class SaphariInteractiveMarkers
{
  public:
    SaphariInteractiveMarkers(const ros::NodeHandle& nh) :
        nh_( nh )
    {}

    void start()
    {
      goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_out", 1);

      server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>
          ("saphari_interactive_markers", "", false);

      visualization_msgs::InteractiveMarker beasty_6dof_marker =
          create_beasty_6dof_marker("arm_flange_link", 0.2);
      server_->insert(beasty_6dof_marker);
      server_->setCallback(beasty_6dof_marker.name, boost::bind(
          &SaphariInteractiveMarkers::feedback_callback, this, _1));

      server_->applyChanges();
    }
  
    ~SaphariInteractiveMarkers() {}

  private:
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

    void feedback_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
      if (feedback->event_type ==
          visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
      {
        geometry_msgs::PoseStamped goal_msg;
        goal_msg.header = feedback->header;
        goal_msg.pose = feedback->pose;
        goal_pub_.publish(goal_msg);

        geometry_msgs::Pose identity_pose;
        identity_pose.orientation.w = 1.0;
        server_->setPose( feedback->marker_name, identity_pose); 
        server_->applyChanges();
      }

    }

    visualization_msgs::InteractiveMarker create_beasty_6dof_marker(const std::string& frame_id, double marker_scale) const
    {
      visualization_msgs::InteractiveMarker result;
      result.name = "beasty_6dof_control";
      result.description = "BEASTY 6-DOF EE-Control";
      result.header.frame_id = frame_id;
      result.scale = marker_scale;

      visualization_msgs::InteractiveMarkerControl control;
      control.always_visible = true;
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      result.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      result.controls.push_back(control);

      return result;
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "saphari_interactive_markers");
  ros::NodeHandle nh("~");

  SaphariInteractiveMarkers sim(nh);
  sim.start();

  ros::spin();
}
