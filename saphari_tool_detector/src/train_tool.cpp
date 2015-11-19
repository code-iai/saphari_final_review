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

#include "receiver.h"
#include "perception.h"
#include "visualizer.h"

class ToolTraining
{
private:
  enum
  {
    COLOR = 0,
    MONO,
    EDGES,
    DX,
    DY
  } mode;

  std::string topic, dataPath, name;
  int32_t id;

  Visualizer visualizer;
  Perception perception;
  Receiver receiver;

  cv::Mat cameraMatrix;
  cv::Mat color, mono, edges, dx, dy;

  ros::NodeHandle nh, priv_nh;

public:
  ToolTraining() : mode(COLOR), nh(), priv_nh("~")
  {
    priv_nh.param("topic", topic, std::string("/camera/image_rect_color"));
    priv_nh.param("data_path", dataPath, std::string(DATA_PATH));
    priv_nh.param("name", name, std::string(""));
    priv_nh.param("id", id, -1);

    visualizer.setDataPath(dataPath);
    perception.setDataPath(dataPath);
    perception.loadSettings();

    priv_nh.param("threshold_low", perception.settings.thresholdLow, perception.settings.thresholdLow);
    priv_nh.param("threshold_high", perception.settings.thresholdHigh, perception.settings.thresholdHigh);
  }

  ~ToolTraining()
  {
    perception.storeSettings();
  }

  void train()
  {
    if(id < 0)
    {
      std::cerr << "invalid id. please set an id >= 0." << std::endl;
      return;
    }
    if(name.empty())
    {
      std::cerr << "invalid name. please set a name." << std::endl;
      return;
    }

    receiver.start(topic);
    std::cout << "waiting for camera..." << std::endl;
    if(!receiver.get(color, cameraMatrix, true))
    {
      return;
    }

    std::cout << "press 'SPACE' for capturing template..." << std::endl;
    bool input = false;
    while(ros::ok())
    {
      if(receiver.get(color, cameraMatrix, false))
      {
        input = true;
        cv::cvtColor(color, mono, CV_BGR2GRAY);

        perception.detectEdges(mono, edges, dx, dy);
      }

      if(input)
      {
        show();
        input = false;
      }

      input = checkKeys();
    }

    receiver.stop();
  }

private:
  void show()
  {
    cv::Mat disp;
    const std::string message = "Press 'SPACE' to capture template.";
    switch(mode)
    {
    case COLOR:
      disp = color.clone();
      cv::line(disp, cv::Point(disp.cols / 2, 0), cv::Point(disp.cols / 2, disp.rows - 1), CV_RGB(127, 127, 127), 1, CV_AA);
      cv::line(disp, cv::Point(0, disp.rows / 2), cv::Point(disp.cols - 1, disp.rows / 2), CV_RGB(127, 127, 127), 1, CV_AA);
      visualizer.show(disp, message, "Image");
      break;
    case MONO:
      visualizer.show(mono, message, "Image");
      break;
    case EDGES:
      visualizer.show(edges, message, "Image");
      break;
    case DX:
      disp = cv::abs(dx);
      disp.convertTo(disp, CV_8U, 0.25, 0);
      visualizer.show(disp, message, "Image");
      break;
    case DY:
      disp = cv::abs(dy);
      disp.convertTo(disp, CV_8U, 0.25, 0);
      visualizer.show(disp, message, "Image");
      break;
    }
  }

  bool checkKeys()
  {
    int32_t key = visualizer.getKey(1);
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
    case 'e':
      mode = EDGES;
      break;
    case 'x':
      mode = DX;
      break;
    case 'y':
      mode = DY;
      break;
    case ' ':
      captureTemplate();
      break;
    case '-':
    case 173:
      perception.settings.thresholdHigh = std::max(perception.settings.thresholdHigh - 1.0, 1.0);
      perception.settings.thresholdLow = perception.settings.thresholdHigh * 0.5;
      break;
    case '+':
    case 171:
      perception.settings.thresholdHigh = perception.settings.thresholdHigh + 1.0;
      perception.settings.thresholdLow = perception.settings.thresholdHigh * 0.5;
      break;
    }
    return true;
  }

  void captureTemplate()
  {
    cv::Vec4i axis, roi;
    cv::Mat disp;

    cv::setMouseCallback("Image", &ToolTraining::getLine, &roi);
    while(ros::ok())
    {
      draw(roi, axis, disp);
      visualizer.show(disp, "Draw region of interest. Press 'SPACE' to continue.", "Image");

      int32_t key = visualizer.getKey(10);
      if(key > 0 && (key & 0xFF) == ' ' && roi.val[0] > 0 && roi.val[1] > 0 && roi.val[2] > 0 && roi.val[3] > 0)
      {
        break;
      }
    }
    std::cout << cv::Rect(cv::Point(std::min(roi.val[0], roi.val[2]), std::min(roi.val[1], roi.val[3])),
                          cv::Point(std::max(roi.val[0], roi.val[2]), std::max(roi.val[1], roi.val[3]))) << std::endl;

    cv::setMouseCallback("Image", &ToolTraining::getLine, &axis);
    while(ros::ok())
    {
      draw(roi, axis, disp);
      visualizer.show(disp, "Draw main axis starting from center. Press 'SPACE' to continue.", "Image");

      int32_t key = visualizer.getKey(10);
      if(key > 0 && (key & 0xFF) == ' ' && axis.val[0] > 0 && axis.val[1] > 0 && axis.val[2] > 0 && axis.val[3] > 0)
      {
        break;
      }
    }
    std::vector<cv::Mat> images;
    size_t count = 8;
    std::string message;
    {
      std::ostringstream oss;
      oss << "Rotate tool. Press 'SPACE' to capture an image. " << count << " images remaining.";
      message = oss.str();
    }
    for(size_t i = count; i > 0 && ros::ok();)
    {
      if(receiver.get(color, cameraMatrix, false))
      {
        visualizer.show(color, message, "Image");
      }

      int32_t key = visualizer.getKey(10);
      if(key >= 0 && (key & 0xFF) == ' ')
      {
        cv::Mat tmp;
        cv::cvtColor(color, tmp, CV_BGR2GRAY);
        images.push_back(tmp);
        --i;
        std::ostringstream oss;
        oss << "Rotate tool. Press 'SPACE' to capture an image. " << i << " images remaining.";
        message = oss.str();

      }
    }

    if(ros::ok())
    {
      cv::Rect rect(cv::Point(std::min(roi.val[0], roi.val[2]), std::min(roi.val[1], roi.val[3])),
                    cv::Point(std::max(roi.val[0], roi.val[2]), std::max(roi.val[1], roi.val[3])));
      cv::Point center(axis.val[0] - rect.x, axis.val[1] - rect.y);
      cv::Point direction(axis.val[2] - axis.val[0], axis.val[3] - axis.val[1]);
      perception.storeTemplate(name, id, mono, rect, center, direction, images);
    }

    ros::shutdown();
  }

  void draw(const cv::Vec4i &roi, const cv::Vec4i &axis, cv::Mat &disp)
  {
    cv::cvtColor(mono, disp, CV_GRAY2BGR);
    cv::Point p0, p1;

    p0 = cv::Point(std::min(roi.val[0], roi.val[2]), std::min(roi.val[1], roi.val[3]));
    p1 = cv::Point(std::max(roi.val[0], roi.val[2]), std::max(roi.val[1], roi.val[3]));
    cv::Rect rect(p0, p1);
    if(rect.area() != 0)
    {
      cv::rectangle(disp, rect, CV_RGB(255, 0, 0), 1, CV_AA);
    }

    p0 = cv::Point(axis.val[0], axis.val[1]);
    p1 = cv::Point(axis.val[2], axis.val[3]);
    if(p0.x > 0 && p0.y > 0)
    {
      cv::circle(disp, p0, 3, CV_RGB(0, 255, 255), 1, CV_AA);
      if(p1.x != p0.x || p1.y != p0.y)
      {
        cv::line(disp, p0, p1, CV_RGB(0, 255, 0), 1, CV_AA);
      }
    }
  }

  static void getLine(int event, int x, int y, int flags, void *data)
  {
    cv::Vec4i &vec = *(cv::Vec4i *)data;

    switch(event)
    {
    case cv::EVENT_LBUTTONDOWN:
      vec.val[0] = x;
      vec.val[1] = y;
      vec.val[2] = x;
      vec.val[3] = y;
      break;
    case cv::EVENT_MOUSEMOVE:
      if(!(flags & cv::EVENT_FLAG_LBUTTON))
      {
        break;
      }
    case cv::EVENT_LBUTTONUP:
      vec.val[2] = x;
      vec.val[3] = y;
      break;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "train_tool", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  ros::AsyncSpinner spinner(0);
  spinner.start();

  ToolTraining training;

  std::cout << "starting tool detection..." << std::endl;

  training.train();

  std::cout << "tool detection stopped." << std::endl;
  ros::shutdown();
  return 0;
}
