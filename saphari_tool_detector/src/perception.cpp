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

bool Perception::loadTemplates(const int thresholdHough, const bool checkConfidence)
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

  if(checkConfidence)
  {
    for(size_t i = 0; i < templates.size(); ++i)
    {
      const GHTTemplate &t = templates[i];
      if(t.minVote == t.maxVote)
      {
        std::cerr << "invalid confidence data. Maybe train_confidence was not executed?" << std::endl;
        return false;
      }
    }
  }
  return true;
}

void Perception::trainConfidences()
{
  const size_t size = templates.size();
  std::vector<int32_t> maxPositive(size), minPositive(size), sumPositive(size, 0), maxNegative(size, INT_MIN), minNegative(size, INT_MAX), sumNegative(size, 0);

  loadTemplateImages();

  for(size_t i = 0; i < templates.size(); ++i)
  {
    GHTTemplate &t1 = templates[i];
    size_t count = 0;
    t1.ght->setTemplate(t1.edges, t1.dx, t1.dy, t1.center);

    for(size_t j = 0; j < templates.size(); ++j)
    {
      GHTTemplate &t2 = templates[j];
      int32_t maxV = INT_MIN, minV = INT_MAX, sum = 0;

      for(size_t k = 0; k < t2.files.size(); ++k)
      {
        std::vector<cv::Vec4f> p;
        std::vector<cv::Vec3i> v;
        t1.ght->detect(t2.imagesEdges[k], t2.imagesDx[k], t2.imagesDy[k], p, v);
        const int32_t vote = v.size() > 0 ? v[0][0] : 0;

        sum += vote;
        if(vote > maxV)
        {
          maxV = vote;
        }
        if(vote < minV)
        {
          minV = vote;
        }
      }

      if(i == j)
      {
        maxPositive[i] = maxV;
        minPositive[i] = minV;
        sumPositive[i] = sum;
      }
      else
      {
        if(maxV > maxNegative[i])
        {
          maxNegative[i] = maxV;
        }
        if(minV < minNegative[i])
        {
          minNegative[i] = minV;
        }
        sumNegative[i] += sum;
        count += t2.files.size();
      }
      std::cout << t1.name << " - " << t2.name << ": " << minV << " " << maxV  << " " << sum / (double)t2.files.size() << std::endl;
    }
    t1.ght->release();
    t1.maxVote = sumPositive[i] / (double)t1.files.size();
    t1.minVote = sumNegative[i] / (double)count;
    t1.maxVote = t1.maxVote - t1.minVote;

    std::cout << "positive: " << minPositive[i] << " " << maxPositive[i] << " " << t1.maxVote << std::endl
              << "negative: " << minNegative[i] << " " << maxNegative[i] << " " << t1.minVote << std::endl << std::endl;
  }

  updateTemplates();
}

void Perception::binarize(const cv::Mat &color, cv::Mat &bin, const uint32_t difference)
{
  bin = cv::Mat(color.rows, color.cols, CV_8U);
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

void Perception::detectTools(std::vector<Tool> &tools, const cv::Rect &roi)
{
  tools.clear();
  cv::Mat edges = this->edges(roi), dx = this->dx(roi), dy = this->dy(roi);

  std::chrono::high_resolution_clock::time_point start, end;
  start = std::chrono::high_resolution_clock::now();
  #pragma omp parallel for
  for(size_t i = 0; i < templates.size(); ++i)
  {
    GHTTemplate &templ = templates[i];
    std::vector<cv::Vec4f> positions;
    std::vector<cv::Vec3i> votes;

    std::chrono::high_resolution_clock::time_point s, e;
    s = std::chrono::high_resolution_clock::now();
    templ.ght->setTemplate(templ.edges, templ.dx, templ.dy, templ.center);
    templ.ght->detect(edges, dx, dy, positions, votes);
    templ.ght->release();
    e = std::chrono::high_resolution_clock::now();
    std::cout << FG_CYAN << templ.name << ": " FG_YELLOW << (e - s).count() / 1000000.0 << " ms." NO_COLOR << std::endl;

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

      if(roi.contains(center))
      {
        Tool t;
        t.id = templ.id;
        t.name = templ.name;
        t.position = center + origin;
        t.direction = direction;
        t.scale = scale;
        t.angle = angle;
        t.confidence = (float)(((double)votes[i][0] - templ.minVote) / templ.maxVote);
        t.votesPosition = votes[i][0];
        t.votesScale = votes[i][1];
        t.votesRotation = votes[i][2];
        t.rect = cv::RotatedRect(center, cv::Size2f(templ.edges.cols * scale, templ.edges.rows * scale), positions[i][3]);
        #pragma omp critical
        tools.push_back(t);
      }
    }
  }
  std::sort(tools.begin(), tools.end(), [](const Tool & a, const Tool & b)
  {
    return a.confidence > b.confidence;
  });
  end = std::chrono::high_resolution_clock::now();
  std::cout << FG_BLUE "detection done: " FG_YELLOW << (end - start).count() / 1000000.0 << " ms." NO_COLOR << std::endl;
}

void Perception::storeTemplate(const std::string &name, const int32_t id, const cv::Rect &roi, const cv::Point &origin, const cv::Point &direction, const std::vector<cv::Mat> &images)
{
  std::vector<int> params;
  params.resize(3, 0);
  params[0] = CV_IMWRITE_JPEG_QUALITY;
  params[1] = 100;
  params[3] = 0;

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(3) << id << "_" << name;
  const std::string base = oss.str();
  oss.str("");

  std::string fileTempl = base + ".yaml";
  std::string fileMono  = base + ".jpg";
  std::vector<std::string> files;

  std::cout << "writing image: " << fileMono << std::endl;
  cv::imwrite(dataPath + fileMono, mono(roi), params);

  for(size_t i = 0; i < images.size(); ++i)
  {
    const cv::Mat image = images[i];
    oss << base << "_image_" << std::setw(2) << i << ".jpg";
    files.push_back(oss.str());
    std::cout << "writing training image " << i << ": " << files[i] << std::endl;
    cv::imwrite(dataPath + files[i], image, params);
    oss.str("");
  }

  std::cout << "template: " << fileTempl << std::endl;

  cv::FileStorage fs(dataPath + fileTempl, cv::FileStorage::WRITE);
  fs << "id" << id;
  fs << "name" << name;
  fs << "center" << cv::Point2f(roi.width * 0.5, roi.height * 0.5);
  fs << "origin" << origin;
  fs << "direction" << direction;
  fs << "angle" << std::atan2(direction.y, direction.x) * 180.0 / M_PI;
  fs << "threshold_low" << thresholdLow;
  fs << "threshold_high" << thresholdHigh;
  fs << "min_vote" << 0.0;
  fs << "max_vote" << 0.0;
  fs << "mono" << fileMono;
  fs << "images" << files;
  fs.release();
}

void Perception::loadTemplate(const std::string &filepath, const int thresholdHough)
{
  std::cout << "loading template: " << filepath << std::endl;

  GHTTemplate t;

  t.path = dataPath + filepath;

  cv::FileStorage fs(t.path, cv::FileStorage::READ);
  fs["id"] >> t.id;
  fs["name"] >> t.name;
  fs["center"] >> t.center;
  fs["origin"] >> t.origin;
  fs["direction"] >> t.direction;
  fs["angle"] >> t.angle;
  fs["threshold_low"] >> t.thresholdLow;
  fs["threshold_high"] >> t.thresholdHigh;
  fs["threshold_low"] >> t.thresholdLow;
  fs["threshold_high"] >> t.thresholdHigh;
  fs["min_vote"] >> t.minVote;
  fs["max_vote"] >> t.maxVote;
  fs["mono"] >> t.fileMono;
  fs["images"] >> t.files;
  fs.release();

  cv::Mat mono = cv::imread(dataPath + t.fileMono, CV_LOAD_IMAGE_GRAYSCALE);
  detectEdges(mono, t.edges, t.dx, t.dy, t.thresholdLow, t.thresholdHigh);

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
  t.ght->set("dp", 2.0);
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
    t.ght->set("angleStep", 2.0);
  }

  templates.push_back(t);
}

void Perception::updateTemplates()
{
  for(size_t i = 0; i < templates.size(); ++i)
  {
    GHTTemplate &t = templates[i];
    cv::FileStorage fs(t.path, cv::FileStorage::WRITE);
    fs << "id" << t.id;
    fs << "name" << t.name;
    fs << "center" << t.center;
    fs << "origin" << t.origin;
    fs << "direction" << t.direction;
    fs << "angle" << t.angle;
    fs << "threshold_low" << t.thresholdLow;
    fs << "threshold_high" << t.thresholdHigh;
    fs << "min_vote" << t.minVote;
    fs << "max_vote" << t.maxVote;
    fs << "mono" << t.fileMono;
    fs << "images" << t.files;
    fs.release();
  }
}

void Perception::loadTemplateImages()
{
  for(size_t i = 0; i < templates.size(); ++i)
  {
    GHTTemplate &t = templates[i];
    t.imagesEdges.resize(t.files.size());
    t.imagesDx.resize(t.files.size());
    t.imagesDy.resize(t.files.size());
    for(size_t j = 0; j < t.files.size(); ++j)
    {
      std::cout << "loading image: " << dataPath + t.files[j] << std::endl;
      const cv::Mat image = cv::imread(dataPath + t.files[j], CV_LOAD_IMAGE_GRAYSCALE);
      detectEdges(image, t.imagesEdges[j], t.imagesDx[j], t.imagesDy[j], t.thresholdLow, t.thresholdHigh);
    }
  }
}

