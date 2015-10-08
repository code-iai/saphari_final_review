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

#include "visualizer.h"

Visualizer::Visualizer() : frame(0), dataPath(DATA_PATH)
{
  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(100);
  params.push_back(0);
}

Visualizer::~Visualizer()
{
  cv::destroyAllWindows();
  cv::waitKey(1);
}

void Visualizer::setDataPath(const std::string &dataPath)
{
  this->dataPath = dataPath;
}

void Visualizer::show(const cv::Mat &image, const std::string &name, const std::string &window)
{
  cv::Mat color, disp;
  if(image.type() == CV_8UC3)
  {
    color = image;
    disp = image.clone();
  }
  else if(image.type() == CV_8U)
  {
    cv::cvtColor(image, color, CV_GRAY2BGR);
    disp = color.clone();
  }
  else
  {
    std::cerr << "Unsupported image format: " << image.type() << std::endl;
    return;
  }

  cv::putText(disp, name, cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0), 1, CV_AA);

  if(windows.find(window) == windows.end())
  {
    cv::namedWindow(window, CV_GUI_NORMAL | CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
  }
  NamedImage &namedImage = windows[window];
  namedImage.image = color;
  namedImage.name = name;

  cv::resizeWindow(window, image.cols, image.rows);
  cv::imshow(window, disp);
}

void Visualizer::remove(const std::string &window)
{
  std::map<std::string, NamedImage>::const_iterator it = windows.find(window);
  if(it != windows.end())
  {
    windows.erase(it);
    cv::destroyWindow(window);
  }
}

int32_t Visualizer::getKey(const int delay)
{
  int32_t key;
  if(delay <= 0)
  {
    while(ros::ok() && (key = cv::waitKey(1)) < 0);
  }
  else
  {
    key = cv::waitKey(delay);
  }

  if(key < 0)
  {
    return -1;
  }

  switch(key & 0xFF)
  {
  case 27: // ESCAPE
    ros::shutdown();
    break;
  case 10: // RETURN
    saveImages();
    break;
  }

  return key;
}

void Visualizer::saveImages()
{
  std::map<std::string, NamedImage>::const_iterator it = windows.begin();
  for(; it != windows.end(); ++it)
  {
    std::ostringstream oss;
    oss << dataPath << std::setfill('0') << std::setw(5) << frame << "_" << it->second.name << ".jpeg";

    std::cout << "saving image: " << oss.str() << std::endl;
    cv::imwrite(oss.str(), it->second.image, params);
  }
  ++frame;
}
