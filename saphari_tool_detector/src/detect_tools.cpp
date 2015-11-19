/**
 * Copyright 2015 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>
#include <mutex>
#include <thread>
#include <random>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include "receiver.h"
#include "perception.h"
#include "intersection.h"
#include "saphari_tool_detector/DetectTools.h"
#include "saphari_tool_detector/Tool.h"

class ToolDetection
{
private:
  enum
  {
    COLOR = 0,
    MONO,
    CANNY,
    EDGES,
    DX,
    DY,
    TOOLS
  } mode;

  std::string topic, tableFrame, cameraFrame, dataPath;
  double maxOverlap, minConfidence;
  bool fakePerception;
  double fakeMinConfidence, fakeMaxConfidence, fakeMinDelay, fakeMaxDelay;
  std::default_random_engine generator;

  Perception perception;
  Receiver receiver;

  cv::Mat cameraMatrix;
  cv::Mat color, mono;
  cv::Rect roi;

  ros::NodeHandle nh, priv_nh;
  tf::TransformListener listener;
  ros::ServiceServer service;
  ros::Publisher debug;

  tf::Transform transform;
  tf::Vector3 normal;
  double distance;
  std::vector<Tool> tools, fakeTools;
  std::vector<tf::Pose> fakePoses;
  std::vector<tf::StampedTransform> poses;

  std::thread tfPublisher;
  std::mutex lock;

public:
  ToolDetection() : mode(COLOR), nh(), priv_nh("~"), listener(nh, ros::Duration(5.0)), normal(0, 0, 1), distance(0)
  {
    bool publish_tf;

    priv_nh.param("topic", topic, std::string("/camera/image_rect_color"));
    priv_nh.param("table_frame", tableFrame, std::string("/table_frame"));
    priv_nh.param("camera_frame", cameraFrame, std::string("/camera_frame"));
    priv_nh.param("data_path", dataPath, std::string(DATA_PATH));
    priv_nh.param("max_overlap", maxOverlap, 0.4);
    priv_nh.param("min_confidence", minConfidence, 0.2);

    priv_nh.param("fake_perception", fakePerception, false);
    priv_nh.param("fake_min_conf", fakeMinConfidence, minConfidence);
    priv_nh.param("fake_max_conf", fakeMaxConfidence, 2.0);
    priv_nh.param("fake_min_delay", fakeMinDelay, 2.0);
    priv_nh.param("fake_max_delay", fakeMaxDelay, 5.0);
    priv_nh.param("publish_tf", publish_tf, false);

    perception.setDataPath(dataPath);
    perception.loadSettings();

    if(publish_tf)
    {
      tfPublisher = std::thread(&ToolDetection::publishStaticTF, this);
    }
    debug = nh.advertise<sensor_msgs::Image>("debug_image", 5);

    if(fakePerception)
    {
      service = nh.advertiseService("detect_tools", &ToolDetection::fakeResults, this);
      lookupTransform();

      fakeTools.resize(9);
      fakePoses.resize(9);
      fakeTools[0].id = 0;
      fakeTools[0].name = "retractor";
      fakePoses[0] = tf::Transform(tf::Quaternion(0, 0, 0.67559, 0.73727), tf::Vector3(0.1, 0.35, 0.005));
      fakeTools[1].id = 1;
      fakeTools[1].name = "blunt-retractor";
      fakePoses[1] = tf::Transform(tf::Quaternion(0, 0, -0.57357, 0.81915), tf::Vector3(0.25, 0.3, 0.005));
      fakeTools[2].id = 2;
      fakeTools[2].name = "bandage-scissors";
      fakePoses[2] = tf::Transform(tf::Quaternion(0, 0, 0.17364, 0.9848), tf::Vector3(0.4, 0.4, 0.005));
      fakeTools[3].id = 3;
      fakeTools[3].name = "scalpel";
      fakePoses[3] = tf::Transform(tf::Quaternion(0, 0, -0.08716, 0.99619), tf::Vector3(0.25, 0.45, 0.005));
      fakeTools[4].id = 4;
      fakeTools[4].name = "scalpel";
      fakePoses[4] = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.4, 0.48, 0.005));
      fakeTools[5].id = 5;
      fakeTools[5].name = "pincers";
      fakePoses[5] = tf::Transform(tf::Quaternion(0, 0, -0.57357, 0.81915), tf::Vector3(0.15, 0.33, 0.005));
      fakeTools[6].id = 6;
      fakeTools[6].name = "small-clamp";
      fakePoses[6] = tf::Transform(tf::Quaternion(0, 0, 0.38268, 0.92388), tf::Vector3(0.4, 0.3, 0.005));
      fakeTools[7].id = 7;
      fakeTools[7].name = "big-clamp";
      fakePoses[7] = tf::Transform(tf::Quaternion(0, 0, 0.25882, 0.96593), tf::Vector3(0.1, 0.5, 0.005));
      fakeTools[8].id = 8;
      fakeTools[8].name = "small-clamp";
      fakePoses[8] = tf::Transform(tf::Quaternion(0, 0, -0.57358, 0.81915), tf::Vector3(0.3, 0.55, 0.005));
    }
    else
    {
      service = nh.advertiseService("detect_tools", &ToolDetection::detectTools, this);

      std::cout << "loading templates..." << std::endl;
      if(!perception.loadTemplates())
      {
        std::cerr << "could not load templates" << std::endl;
        return;
      }
      receiver.start(topic);
    }

    generator.seed(ros::Time::now().nsec);
  }

  bool detectTools(saphari_tool_detector::DetectToolsRequest &request, saphari_tool_detector::DetectToolsResponse &response)
  {
    std::chrono::high_resolution_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();
    if(!receiver.get(color, cameraMatrix, true) || !receiver.get(color, cameraMatrix, true))
    {
      return false;
    }

    end = std::chrono::high_resolution_clock::now();
    std::cout << std::endl << FG_RED "--------------------------------------------------------------------------------" << std::endl << std::endl;
    std::cout << FG_BLUE "camera image received: " FG_YELLOW << (end - start).count() / 1000000.0 << " ms." NO_COLOR << std::endl;
    start = std::chrono::high_resolution_clock::now();

    ros::Time now = ros::Time::now();
    if(!lookupTransform())
    {
      return false;
    }

    if(request.width == 0 || request.height == 0)
    {
      roi = cv::Rect(0, 0, color.cols, color.rows);
    }
    else
    {
      roi = cv::Rect(request.x, request.y, request.width, request.height);
    }

    cv::cvtColor(color, mono, CV_BGR2GRAY);
    std::vector<Tool> tools;
    perception.detectTools(mono, tools, roi);

    lock.lock();
    this->tools.clear();
    poses.clear();
    for(size_t i = 0; i < tools.size(); ++i)
    {
      Tool &t = tools[i];

      if(t.confidence < minConfidence)
      {
        if(t.confidence > 0.0f)
        {
          std::cout << FG_YELLOW << t.id << ": " << t.name << " [" << t.votesPosition << "](" << t.confidence << ") low confidence." NO_COLOR << std::endl;
        }
      }
      else if(checkOverlap(t, this->tools))
      {
        std::cout << FG_CYAN << t.id << ": " << t.name << " [" << t.votesPosition << "](" << t.confidence << ") overlap." NO_COLOR << std::endl;
      }
      else
      {
        std::cout << FG_GREEN << t.id << ": " << t.name << " [" << t.votesPosition << "](" << t.confidence << ") ok." NO_COLOR << std::endl;

        std::ostringstream oss;
        oss << "/tool_" << i << '_' << t.name;

        tf::Pose pose = computePose(t);
        tf::StampedTransform stamped = tf::StampedTransform(pose, now, cameraFrame, oss.str());

        saphari_tool_detector::Tool tool;
        tool.id = t.id;
        tool.name = t.name;
        tool.confidence = t.confidence;
        tf::transformStampedTFToMsg(stamped, tool.pose);

        response.tools.push_back(tool);
        poses.push_back(stamped);
        this->tools.push_back(t);
      }
    }

    createDebugImage(now);
    lock.unlock();

    end = std::chrono::high_resolution_clock::now();
    std::cout << FG_BLUE "service call done: " FG_YELLOW << (end - start).count() / 1000000.0 << " ms." NO_COLOR << std::endl;
    return true;
  }

  bool fakeResults(saphari_tool_detector::DetectToolsRequest &request, saphari_tool_detector::DetectToolsResponse &response)
  {
    std::chrono::high_resolution_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();
    ros::Time now = ros::Time::now();
    if(!lookupTransform())
    {
      return false;
    }

    std::uniform_real_distribution<double> confidence(fakeMinConfidence, fakeMaxConfidence);
    std::uniform_int_distribution<int64_t> delay((int64_t)(fakeMinDelay * 1000.0), (int64_t)(fakeMaxDelay * 1000.0));

    lock.lock();

    poses.clear();
    tools.resize(fakeTools.size());

    for(size_t i = 0; i < fakeTools.size(); ++i)
    {
      const Tool &fake = fakeTools[i];
      Tool &t = tools[i];

      t.id = fake.id;
      t.name = fake.name;
      t.confidence = confidence(generator);
    }

    std::sort(tools.begin(), tools.end(), [](const Tool & a, const Tool & b)
    {
      return a.confidence > b.confidence;
    });

    for(size_t i = 0; i < fakeTools.size(); ++i)
    {
      const Tool &t = tools[i];
      saphari_tool_detector::Tool tool;

      std::ostringstream oss;
      oss << "/tool_" << i << '_' << t.name;

      tf::Pose pose = transform * fakePoses[t.id];
      tf::StampedTransform stamped = tf::StampedTransform(pose, now, cameraFrame, oss.str());

      tool.id = t.id;
      tool.name = t.name;
      tool.confidence = t.confidence;
      tf::transformStampedTFToMsg(stamped, tool.pose);
      response.tools.push_back(tool);
      poses.push_back(stamped);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(delay(generator)));
    lock.unlock();

    end = std::chrono::high_resolution_clock::now();
    std::cout << FG_BLUE "service call done: " FG_YELLOW << (end - start).count() / 1000000.0 << " ms." NO_COLOR << std::endl;
    return true;
  }

private:
  void createDebugImage(const ros::Time &now)
  {
    cv::Mat disp = color.clone();
    cv::rectangle(disp, roi, CV_RGB(0, 255, 0), 2, CV_AA);

    for(size_t i = 0; i < tools.size(); ++i)
    {
      const Tool &t = tools[i];
      std::vector<cv::Point2f> points(4);
      t.rect.points(points.data());

      cv::circle(disp, t.position, 5, CV_RGB(255, 255, 0), 2, CV_AA);
      cv::line(disp, t.position, t.position + t.direction * 100, CV_RGB(0, 255, 0), 2, CV_AA);

      cv::line(disp, points[0], points[1], CV_RGB(255, 0, 0), 1, CV_AA);
      cv::line(disp, points[1], points[2], CV_RGB(255, 0, 0), 1, CV_AA);
      cv::line(disp, points[2], points[3], CV_RGB(255, 0, 0), 1, CV_AA);
      cv::line(disp, points[3], points[0], CV_RGB(255, 0, 0), 1, CV_AA);

      std::ostringstream oss;
      oss << t.name << "(" << t.confidence << ")";
      cv::Point pos(t.position.x + 10, t.position.y);
      cv::putText(disp, oss.str(), pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, CV_RGB(0, 0, 0), 1, CV_AA);
    }

    size_t step, size;
    step = disp.cols * disp.elemSize();
    size = disp.rows * step;

    sensor_msgs::Image msgImage;
    msgImage.encoding = sensor_msgs::image_encodings::BGR8;
    msgImage.header.frame_id = cameraFrame;
    msgImage.header.seq = 0;
    msgImage.header.stamp = now;
    msgImage.height = disp.rows;
    msgImage.width = disp.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size);
    memcpy(msgImage.data.data(), disp.data, size);

    debug.publish(msgImage);
  }

  void publishStaticTF()
  {
    tf::TransformBroadcaster broadcaster;

    for(; ros::ok();)
    {
      ros::Time now = ros::Time::now();

      lock.lock();
      for(size_t i = 0; i < tools.size(); ++i)
      {
        tf::StampedTransform transform = poses[i];
        transform.stamp_ = now;
        broadcaster.sendTransform(transform);
      }
      lock.unlock();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  tf::Pose computePose(Tool &tool)
  {
    std::vector<cv::Point2f> points(2), undistorted(2);
    points[0] = tool.position;
    points[1] = tool.position + tool.direction;
    cv::undistortPoints(points, undistorted, cameraMatrix, cv::Mat());

    tf::Vector3 center(undistorted[0].x, undistorted[0].y, 1);
    tf::Vector3 direction(undistorted[1].x, undistorted[1].y, 1);

    center = center * (distance / normal.dot(center));
    direction = direction * (distance / normal.dot(direction));

    tf::Vector3 axisX, axisY, axisZ;
    axisX = (direction - center).normalize();
    axisZ = normal;
    axisY = -axisX.cross(axisZ).normalize();

    tf::Matrix3x3 rot;
    rot[0] = axisX;
    rot[1] = axisY;
    rot[2] = axisZ;

    tf::Pose pose;
    pose.setOrigin(center);
    pose.setBasis(rot);

    return pose;
  }

  bool lookupTransform()
  {
    tf::StampedTransform transform;
    try
    {
      listener.waitForTransform(cameraFrame, tableFrame, ros::Time(0), ros::Duration(5));
      listener.lookupTransform(cameraFrame, tableFrame, ros::Time(0), transform);

      normal = transform.getBasis() * tf::Vector3(0, 0, 1);
      normal.normalize();
      distance = normal.dot(transform.getOrigin());
      this->transform = transform;
    }
    catch(const tf::TransformException &ex)
    {
      std::cerr << ex.what() << std::endl;
      return false;
    }
    return true;
  }

  bool checkOverlap(const Tool &tool, const std::vector<Tool> &tools)
  {
    for(size_t i = 0; i < tools.size(); ++i)
    {
      const Tool &t = tools[i];
      std::vector<cv::Point2f> points, hull;
      int ret = cv::rotatedRectangleIntersection(tool.rect, t.rect, points);
      switch(ret)
      {
      case cv::INTERSECT_NONE:
        continue;
      case cv::INTERSECT_PARTIAL:
        {
          cv::convexHull(points, hull);
          double overlap = cv::contourArea(hull);
          if(overlap > maxOverlap * tool.rect.size.area() || overlap > maxOverlap * t.rect.size.area())
          {
            return true;
          }
          break;
        }
      case cv::INTERSECT_FULL:
        return true;
      }
    }
    return false;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_tools", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  //ros::AsyncSpinner spinner(0);
  //spinner.start();

  ToolDetection detection;

  std::cout << "starting tool detection..." << std::endl;

  ros::spin();

  std::cout << "tool detection stopped." << std::endl;
  ros::shutdown();
  return 0;
}
