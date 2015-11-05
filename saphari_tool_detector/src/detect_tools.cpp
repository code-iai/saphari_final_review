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

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "receiver.h"
#include "perception.h"
#include "visualizer.h"
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
  double thresholdLow, thresholdHigh, maxOverlap;
  int thresholdHough;
  bool fakePerception;

  //Visualizer visualizer;
  Perception perception;
  Receiver receiver;

  cv::Mat cameraMatrix;
  cv::Mat color, mono, bin, edges, dx, dy;
  cv::Rect roi;

  ros::NodeHandle nh, priv_nh;
  tf::TransformListener listener;
  ros::ServiceServer service;
  ros::Publisher debug;

  tf::Transform transform;
  tf::Vector3 normal;
  double distance;
  std::vector<Tool> tools;

  std::thread tfPublisher;
  std::mutex lock;

public:
  ToolDetection() : mode(COLOR), nh(), priv_nh("~"), listener(nh, ros::Duration(5.0)), normal(0, 0, 1), distance(0)
  {
    int32_t x, y, width, height;
    bool publish_tf;

    priv_nh.param("topic", topic, std::string("/camera/image_rect_color"));
    priv_nh.param("table_frame", tableFrame, std::string("/table_frame"));
    priv_nh.param("camera_frame", cameraFrame, std::string("/camera_frame"));
    priv_nh.param("data_path", dataPath, std::string(DATA_PATH));
    priv_nh.param("threshold_low", thresholdLow, 50.0);
    priv_nh.param("threshold_high", thresholdHigh, 100.0);
    priv_nh.param("threshold_hough", thresholdHough, 50);
    priv_nh.param("max_overlap", maxOverlap, 0.4);
    priv_nh.param("x", x, 200);
    priv_nh.param("y", y, 0);
    priv_nh.param("width", width, 1600);
    priv_nh.param("height", height, 1199);
    priv_nh.param("fake_perception", fakePerception, false);
    priv_nh.param("publish_tf", publish_tf, false);

    //visualizer.setDataPath(dataPath);
    perception.setDataPath(dataPath);
    roi = cv::Rect(x, y, width, height);
    perception.setROI(roi);

    if(publish_tf)
    {
      tfPublisher = std::thread(&ToolDetection::publishStaticTF, this);
    }
    debug = nh.advertise<sensor_msgs::Image>("debug_image", 5);

    if(fakePerception)
    {
      service = nh.advertiseService("detect_tools", &ToolDetection::fakeResults, this);
      lookupTransform();

      tf::Quaternion q(0, 0, 0, 1);
      tools.resize(3);
      tools[0].name = "hook";
      tools[0].id = 0;
      tools[0].pose = transform * tf::Transform(q, tf::Vector3(0.0, 0.25, 0));
      tools[1].name = "rake";
      tools[1].id = 1;
      tools[1].pose = transform * tf::Transform(q, tf::Vector3(0.1, 0.25, 0));
      tools[2].name = "scissor";
      tools[2].id = 2;
      tools[2].pose = transform * tf::Transform(q, tf::Vector3(0.2, 0.25, 0));
    }
    else
    {
      service = nh.advertiseService("detect_tools", &ToolDetection::detectTools, this);

      std::cout << "loading templates..." << std::endl;
      if(!perception.loadTemplates(thresholdHough))
      {
        std::cerr << "could not load templates" << std::endl;
        return;
      }

      receiver.start(topic);
    }
  }

  /*void start()
  {
    std::cout << "loading templates..." << std::endl;
    if(!perception.loadTemplates(thresholdHough))
    {
      std::cerr << "could not load templates" << std::endl;
      return;
    }

    receiver.start(topic);
    std::cout << "waiting for camera..." << std::endl;
    if(!receiver.get(color, cameraMatrix, true))
    {
      return;
    }

    std::cout << "running tool detection..." << std::endl;
    bool input = false;
    while(ros::ok())
    {
      if(receiver.get(color, cameraMatrix, false))
      {
        lookupTransform();

        input = true;
        cv::cvtColor(color, mono, CV_BGR2GRAY);

        perception.detectEdges(mono, edges, dx, dy, thresholdLow, thresholdHigh);

        if(mode == TOOLS)
        {
          std::vector<Tool> tools;
          perception.detectTools(tools);

          lock.lock();
          this->tools.clear();
          for(size_t i = 0; i < tools.size(); ++i)
          {
            Tool &t = tools[i];

            if(!checkOverlap(t, this->tools))
            {
              computePose(t);
              this->tools.push_back(t);
            }
          }
          lock.unlock();
        }
      }

      if(input)
      {
        show();
        input = false;
      }

      input = checkKeys();
    }
  }*/

  bool detectTools(saphari_tool_detector::DetectToolsRequest &request, saphari_tool_detector::DetectToolsResponse &response)
  {
    if(!receiver.get(color, cameraMatrix, true))
    {
      return false;
    }
    ros::Time now = ros::Time::now();
    if(!lookupTransform())
    {
      return false;
    }

    cv::cvtColor(color, mono, CV_BGR2GRAY);
    perception.detectEdges(mono, edges, dx, dy, thresholdLow, thresholdHigh);

    std::vector<Tool> tools;
    perception.detectTools(tools);

    lock.lock();
    this->tools.clear();
    for(size_t i = 0; i < tools.size(); ++i)
    {
      Tool &t = tools[i];
      if(!checkOverlap(t, this->tools))
      {
        computePose(t);

        std::ostringstream oss;
        oss << "/tool_" << i << '_' << t.name;

        saphari_tool_detector::Tool tool;
        tool.id = t.id;
        tool.name = t.name;
        tf::transformStampedTFToMsg(tf::StampedTransform(t.pose, now, cameraFrame, oss.str()), tool.pose);

        response.tools.push_back(tool);
        this->tools.push_back(t);
      }
    }

    createDebugImage(now);
    lock.unlock();

    return true;
  }

  bool fakeResults(saphari_tool_detector::DetectToolsRequest &request, saphari_tool_detector::DetectToolsResponse &response)
  {
    ros::Time now = ros::Time::now();
    tf::Quaternion q(0, 0, 0, 1);
    if(!lookupTransform())
    {
      return false;
    }

    lock.lock();
    tools[0].pose = transform * tf::Transform(q, tf::Vector3(0.1, 0.4, 0));
    tools[1].pose = transform * tf::Transform(q, tf::Vector3(0.3, 0.4, 0));
    tools[2].pose = transform * tf::Transform(q, tf::Vector3(0.5, 0.4, 0));

    for(size_t i = 0; i < tools.size(); ++i)
    {
      Tool &t = tools[i];
      saphari_tool_detector::Tool tool;

      std::ostringstream oss;
      oss << "/tool_" << i << '_' << t.name;

      tool.id = t.id;
      tool.name = t.name;
      tf::transformStampedTFToMsg(tf::StampedTransform(t.pose, now, cameraFrame, oss.str()), tool.pose);
      response.tools.push_back(tool);
    }
    lock.unlock();

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
        const Tool &t = tools[i];
        std::ostringstream oss;
        oss << "/tool_" << i << '_' << t.name;

        tf::StampedTransform transform(t.pose, now, cameraFrame, oss.str());
        broadcaster.sendTransform(transform);
      }
      lock.unlock();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void computePose(Tool &tool)
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

    /*std::cout << "Translation: " << center.x() << ", " << center.y() << ", " << center.z() << std::endl;
    std::cout << "Rotation: " << axisX.x() << ", " << axisY.x() << ", " << axisZ.x() << std::endl;
    std::cout << "          " << axisX.y() << ", " << axisY.y() << ", " << axisZ.y() << std::endl;
    std::cout << "          " << axisX.z() << ", " << axisY.z() << ", " << axisZ.z() << std::endl;*/
    tool.pose.setOrigin(center);
    tool.pose.setBasis(rot);
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

      //std::cout << "Normal: " << normal.x() << ", " << normal.y() << ", " << normal.z() << std::endl;
      //std::cout << "Distance: " << distance << std::endl;
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
    //std::cout << "checking overlap... " << std::endl;
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
          //std::cout << points << std::endl;
          //std::cout << hull << std::endl;
          //std::cout << "overlap: " << overlap << " t0 area: " << tool.rect.size.area() << " t1 area: " << t.rect.size.area() << std::endl;
          if(overlap > maxOverlap * tool.rect.size.area() || overlap > maxOverlap * t.rect.size.area())
          {
            //std::cout << "... overlap: " << overlap << " t0 area: " << tool.rect.size.area() << " t1 area: " << t.rect.size.area() << std::endl;
            return true;
          }
          break;
        }
      case cv::INTERSECT_FULL:
        //std::cout << "... overlap: full" << std::endl;
        return true;
      }
    }
    //std::cout << "... no overlap" << std::endl;
    return false;
  }

  /*void show()
  {
    cv::Mat disp;
    switch(mode)
    {
    case COLOR:
      visualizer.show(color, "Color", "Image");
      break;
    case MONO:
      visualizer.show(mono, "Mono", "Image");
      break;
    case CANNY:
      visualizer.show(edges, "Canny", "Image");
      break;
    case EDGES:
      disp = cv::abs(dx) + cv::abs(dy);
      disp.convertTo(disp, CV_8U, 0.25, 0);
      visualizer.show(disp, "Edges", "Image");
      break;
    case DX:
      disp = cv::abs(dx);
      disp.convertTo(disp, CV_8U, 0.25, 0);
      visualizer.show(disp, "DX", "Image");
      break;
    case DY:
      disp = cv::abs(dy);
      disp.convertTo(disp, CV_8U, 0.25, 0);
      visualizer.show(disp, "DY", "Image");
      break;
    case TOOLS:
      disp = color.clone();
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
      }
      visualizer.show(disp, "DY", "Image");
      break;
    }
  }

  bool checkKeys()
  {
    int32_t key = visualizer.getKey(100);
    if(key < 0)
    {
      return false;
    }

    switch(key & 0xFF)
    {
    case 'c':
      mode = COLOR;
      break;
    case 'm':
      mode = MONO;
      break;
    case 'v':
      mode = CANNY;
      break;
    case 'e':
      mode = EDGES;
      break;
    case 'x':
      mode = DX;
      break;
    case 'y':
      mode = DY;
      break;
    case 't':
      mode = TOOLS;
      break;
    case '-':
    case 173:
      thresholdHigh = std::max(thresholdHigh - 1.0, 1.0);
      thresholdLow = thresholdHigh * 0.5;
      break;
    case '+':
    case 171:
      thresholdHigh = thresholdHigh + 1.0;
      thresholdLow = thresholdHigh * 0.5;
      break;
    }
    return true;
  }*/
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
