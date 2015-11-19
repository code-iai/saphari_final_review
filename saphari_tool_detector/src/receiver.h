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

#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#include <string>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

class Receiver
{
private:
  std::mutex lock;
  bool hasNewImage, running;

  cv::Mat image;
  cv::Mat cameraMatrix;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::CameraSubscriber sub;

public:
  Receiver();
  ~Receiver();

  void start(const std::string &topic, const uint32_t queueSize = 5, const bool useCompressed = false);
  void stop();

  bool get(cv::Mat &image, cv::Mat &cameraMatrix, const bool wait = false);

private:
  void callback(const sensor_msgs::ImageConstPtr &msgImage, const sensor_msgs::CameraInfoConstPtr &msgInfo);
};

#endif // __RECEIVER_H__
