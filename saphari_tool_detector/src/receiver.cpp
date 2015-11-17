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

#include "receiver.h"

Receiver::Receiver()
  : hasNewImage(false), running(false), nh("~"), it(nh), sub()
{
}

Receiver::~Receiver()
{
  stop();
}

void Receiver::start(const std::string &topic, const uint32_t queueSize, const bool useCompressed)
{
  if(!running)
  {
    cameraMatrix = cv::Mat();
    hasNewImage = false;
    running = true;
    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");

    sub = it.subscribeCamera(topic, queueSize, &Receiver::callback, this, hints);
  }
}

void Receiver::stop()
{
  if(running)
  {
    running = false;
    sub.shutdown();
    sub = image_transport::CameraSubscriber();
  }
}

bool Receiver::get(cv::Mat &image, cv::Mat &cameraMatrix, const bool wait)
{
  if(wait)
  {
    while(ros::ok() && !hasNewImage)
    {
      ros::spinOnce();
    }
    if(!ros::ok())
    {
      stop();
      return false;
    }
  }
  else if(!hasNewImage)
  {
    return false;
  }

  lock.lock();
  image = this->image;
  cameraMatrix = this->cameraMatrix;
  hasNewImage = false;
  lock.unlock();
  return true;
}

void Receiver::callback(const sensor_msgs::ImageConstPtr &msgImage, const sensor_msgs::CameraInfoConstPtr &msgInfo)
{
  cv::Mat image, cameraMatrix;

  cameraMatrix = cv::Mat(3, 3, CV_64F);
  memcpy(cameraMatrix.ptr<double>(0, 0), msgInfo->K.data(), sizeof(double) * 9);

  cv_bridge::CvImageConstPtr pCvImage;
  pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
  pCvImage->image.copyTo(image);

  lock.lock();
  this->image = image;
  this->cameraMatrix = cameraMatrix;
  hasNewImage = true;
  lock.unlock();
}
