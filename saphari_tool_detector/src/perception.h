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

#ifndef __PERCEPTION_H__
#define __PERCEPTION_H__

#include <string>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <tf/tf.h>

struct Tool
{
  uint32_t id;
  std::string name;
  cv::Point2f position, direction;
  float scale, angle;
  int32_t votesPosition, votesScale, votesRotation;
  cv::RotatedRect rect;
  tf::Pose pose;
};

class Perception
{
private:
  struct GHTTemplate
  {
    int32_t id;
    std::string name;
    cv::Point2f center;
    cv::Point origin, direction;
    double angle, thresholdLow, thresholdHigh;
    cv::Mat mono, edges, dx, dy;
    cv::Ptr<cv::GeneralizedHough> ght;
  };

  std::vector<GHTTemplate> templates;

  std::string dataPath;
  cv::Rect roi;
  cv::Mat cameraMatrix;
  tf::Transform transform;

  cv::Mat mono, edges, dx, dy;
  double thresholdLow, thresholdHigh;
  bool estimateScale, estimateRotation;

public:
  Perception();
  ~Perception();

  void setDataPath(const std::string &dataPath);
  void setROI(const cv::Rect &roi);
  void setCameraMatrix(const cv::Mat &cameraMatrix);
  void setTransform(const tf::Transform &transform);
  void setEstimateScale(const bool enable);
  void setEstimateRotation(const bool enable);

  bool loadTemplates(const int thresholdHough);

  void binarize(const cv::Mat &color, cv::Mat &bin, const uint32_t difference = 70);
  void detectEdges(const cv::Mat &mono, cv::Mat &edges, cv::Mat &dx, cv::Mat &dy, const double thresholdLow = 50, const double thresholdHigh = 100);
  void detectTools(std::vector<Tool> &tools);

  void storeTemplate(const std::string &name, const int32_t id, const cv::Rect &roi, const cv::Point &origin, const cv::Point &direction);

private:
  void loadTemplate(const std::string &filepath, const int thresholdHough);
};

#endif // __PERCEPTION_H__
