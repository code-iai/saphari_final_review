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

#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include <string>
#include <mutex>
#include <thread>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

class Visualizer
{
private:
  struct NamedImage
  {
    cv::Mat image;
    std::string name;
  };

  std::map<std::string, NamedImage> windows;
  std::vector<int32_t> params;
  size_t frame;
  std::string dataPath;

public:
  Visualizer();
  ~Visualizer();

  void setDataPath(const std::string &dataPath);
  void show(const cv::Mat &image, const std::string &name, const std::string &window = "Image");
  void remove(const std::string &window);
  int32_t getKey(const int delay = 0);

private:
  void saveImages();
};

#endif // __VISUALIZER_H__
