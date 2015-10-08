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

#include <dirent.h>
#include <sys/stat.h>

#include "perception.h"

Perception::Perception() : estimateScale(false), estimateRotation(true)
{
}

Perception::~Perception()
{
}

void Perception::setDataPath(const std::string &dataPath)
{
  this->dataPath = dataPath;
}

void Perception::setROI(const cv::Rect &roi)
{
  this->roi = roi;
}

void Perception::setCameraMatrix(const cv::Mat &cameraMatrix)
{
  this->cameraMatrix = cameraMatrix;
}

void Perception::setTransform(const tf::Transform &transform)
{
  this->transform = transform;
}

void Perception::setEstimateScale(const bool enable)
{
  this->estimateScale = enable;
}

void Perception::setEstimateRotation(const bool enable)
{
  this->estimateRotation = enable;
}

bool Perception::loadTemplates(const int thresholdHough)
{
  std::vector<std::string> files;

  DIR *dp;
  struct dirent *dirp;
  size_t pos;

  if((dp  = opendir(dataPath.c_str())) ==  NULL)
  {
    std::cerr << "Error opening: " << dataPath << std::endl;
    return false;
  }

  while((dirp = readdir(dp)) != NULL)
  {
    std::string filename = dirp->d_name;

    if(dirp->d_type != DT_REG)
    {
      continue;
    }

    pos = filename.rfind(".yaml");

    if(pos != std::string::npos)
    {
      files.push_back(filename);
    }
  }
  closedir(dp);

  std::sort(files.begin(), files.end());
  templates.reserve(files.size());

  for(size_t i = 0; i < files.size(); ++i)
  {
    loadTemplate(files[i], thresholdHough);
  }
  return true;
}

void Perception::binarize(const cv::Mat &color, cv::Mat &bin, const uint32_t difference)
{
  bin = cv::Mat(color.rows, color.cols, CV_8U);
  //#pragma omp parallel for
  for(size_t r = 0; r < color.rows; ++r)
  {
    const cv::Vec3b *itI = color.ptr<cv::Vec3b>(r);
    uint8_t *itO = bin.ptr<uint8_t>(r);
    for(size_t c = 0; c < color.cols; ++c, ++itI, ++itO)
    {
      int32_t v = itI->val[0] - difference;
      *itO = (v > itI->val[1] && v > itI->val[2]) ? 0 : 255;
    }
  }
}

void Perception::detectEdges(const cv::Mat &mono, cv::Mat &edges, cv::Mat &dx, cv::Mat &dy, const double thresholdLow, const double thresholdHigh)
{
  cv::Canny(mono, edges, thresholdLow, thresholdHigh);
  cv::Sobel(mono, dx, CV_32F, 1, 0);
  cv::Sobel(mono, dy, CV_32F, 0, 1);

  this->mono = mono;
  this->thresholdLow = thresholdLow;
  this->thresholdHigh = thresholdHigh;
  this->edges = edges;
  this->dx = dx;
  this->dy = dy;
}

void Perception::detectTools(std::vector<Tool> &tools)
{
  tools.clear();
  cv::Mat edges = this->edges(roi), dx = this->dx(roi), dy = this->dy(roi);

  #pragma omp parallel for
  for(size_t i = 0; i < templates.size(); ++i)
  {
    GHTTemplate &templ = templates[i];
    std::vector<cv::Vec4f> positions;
    std::vector<cv::Vec3i> votes;

    templ.ght->detect(edges, dx, dy, positions, votes);

    for(size_t i = 0; i < positions.size(); ++i)
    {
      double angle = positions[i][3] * M_PI / 180.0;
      double angleDirection = (templ.angle + positions[i][3]) * M_PI / 180.0;
      float scale = positions[i][2];
      cv::Point2f o(templ.origin.x - templ.center.x, templ.origin.y - templ.center.y);
      cv::Point2f center(positions[i][0] + roi.x, positions[i][1] + roi.y);
      cv::Point2f origin(o.x * std::cos(angle) - o.y * std::sin(angle), o.x * std::sin(angle) + o.y * std::cos(angle));
      cv::Point2f direction(1.0 * std::cos(angleDirection), 1.0 * std::sin(angleDirection));
      origin *= scale;

      Tool t;
      t.id = templ.id;
      t.name = templ.name;
      t.position = center + origin;
      t.direction = direction;
      t.scale = scale;
      t.angle = angle;
      t.votesPosition = votes[i][0];
      t.votesScale = votes[i][1];
      t.votesRotation = votes[i][2];
      t.rect = cv::RotatedRect(center, cv::Size2f(templ.edges.cols * scale, templ.edges.rows * scale), positions[i][3]);
      #pragma omp critical
      tools.push_back(t);
    }
  }
  std::sort(tools.begin(), tools.end(), [](const Tool &a, const Tool &b){return a.votesPosition > b.votesPosition;});
}

void Perception::loadTemplate(const std::string &filepath, const int thresholdHough)
{
  std::cout << "loading template: " << filepath << std::endl;

  GHTTemplate t;

  cv::FileStorage fs(dataPath + filepath, cv::FileStorage::READ);
  fs["id"] >> t.id;
  fs["name"] >> t.name;
  fs["center"] >> t.center;
  fs["origin"] >> t.origin;
  fs["direction"] >> t.direction;
  fs["angle"] >> t.angle;
  fs["threshold_low"] >> t.thresholdLow;
  fs["threshold_high"] >> t.thresholdHigh;
  fs["mono"] >> t.mono;
  fs["edges"] >> t.edges;
  fs["dx"] >> t.dx;
  fs["dy"] >> t.dy;
  fs.release();

  int32_t method = cv::GHT_POSITION;
  if(estimateScale)
  {
    method += cv::GHT_SCALE;
  }
  if(estimateRotation)
  {
    method += cv::GHT_ROTATION;
  }

  t.ght = cv::GeneralizedHough::create(method);

  t.ght->set("minDist", 100);
  t.ght->set("levels", 360);
  t.ght->set("dp", 1.0);
  if(estimateScale && estimateRotation)
  {
    t.ght->set("angleThresh", 1000);
    t.ght->set("scaleThresh", 100);
    t.ght->set("posThresh", 15);
    t.ght->set("maxSize", 10000);
    t.ght->set("xi", 90.0);
    t.ght->set("angleEpsilon", 1.0);
  }
  else
  {
    t.ght->set("votesThreshold", thresholdHough);
  }
  if(estimateScale)
  {
    t.ght->set("minScale", 0.8);
    t.ght->set("maxScale", 1.2);
    t.ght->set("scaleStep", 0.05);
  }
  if(estimateRotation)
  {
    t.ght->set("minAngle", 0.0);
    t.ght->set("maxAngle", 360.0);
    t.ght->set("angleStep", 1.0);
  }

  t.ght->setTemplate(t.edges, t.dx, t.dy, t.center);

  templates.push_back(t);
}

void Perception::storeTemplate(const std::string &name, const int32_t id, const cv::Rect &roi, const cv::Point &origin, const cv::Point &direction)
{
  std::ostringstream oss;
  oss << dataPath << std::setfill('0') << std::setw(5) << id << "_" << name << ".yaml";

  std::cout << "template: " << oss.str() << std::endl;

  cv::FileStorage fs(oss.str(), cv::FileStorage::WRITE);
  fs << "id" << id;
  fs << "name" << name;
  fs << "center" << cv::Point2f(roi.width * 0.5, roi.height * 0.5);
  fs << "origin" << origin;
  fs << "direction" << direction;
  fs << "angle" << std::atan2(direction.y, direction.x) * 180.0 / M_PI;
  fs << "threshold_low" << thresholdLow;
  fs << "threshold_high" << thresholdHigh;
  fs << "mono" << mono(roi);
  fs << "edges" << edges(roi);
  fs << "dx" << dx(roi);
  fs << "dy" << dy(roi);
  fs.release();
}


