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

#include <opencv2/opencv.hpp>

#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"

struct Tool
{
  int32_t id;
  std::string name;
  cv::Point2f position, direction;
  float scale, angle, confidence;
  int32_t votesPosition, votesScale, votesRotation;
  cv::RotatedRect rect;
};

class Perception
{
private:
  struct GHTTemplate
  {
    int32_t id;
    std::string name, path;
    cv::Point2f center;
    cv::Point origin, direction;
    double angle, maxVote, minVote;
    int32_t width, height;
    cv::Mat edges, dx, dy;
    std::string fileMono;
    std::vector<std::string> files;

    std::vector<cv::Mat> imagesEdges;
    std::vector<cv::Mat> imagesDx;
    std::vector<cv::Mat> imagesDy;
    cv::Ptr<cv::GeneralizedHough> ght;
  };

  struct Settings
  {
    int32_t thresholdHough, levels;
    double thresholdLow, thresholdHigh, scale, minDist, dp, angleStep;
    bool estimateScale, estimateRotation;
  };

  std::vector<GHTTemplate> templates;

  std::string dataPath;
  cv::Mat cameraMatrix;

public:
  Settings settings;

  Perception();
  ~Perception();

  void setDataPath(const std::string &dataPath);
  void setCameraMatrix(const cv::Mat &cameraMatrix);

  void loadSettings();
  void storeSettings();

  bool loadTemplates(const bool checkConfidence = true);
  void trainConfidences();

  void detectEdges(const cv::Mat &mono, cv::Mat &edges, cv::Mat &dx, cv::Mat &dy);
  void detectTools(const cv::Mat &mono, std::vector<Tool> &tools, const cv::Rect &roi);

  void storeTemplate(const std::string &name, const int32_t id, const cv::Mat &mono, const cv::Rect &roi, const cv::Point &origin, const cv::Point &direction, const std::vector<cv::Mat> &images);

private:
  void resize(const cv::Mat &in, cv::Mat &out) const;
  void updateTemplates();
  void loadTemplate(const std::string &filepath);
  void loadTemplateImages();
};

#endif // __PERCEPTION_H__
