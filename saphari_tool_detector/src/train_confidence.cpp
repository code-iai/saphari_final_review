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

#include <ros/ros.h>

#include "receiver.h"
#include "perception.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "train_confidence", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  ros::NodeHandle priv_nh("~");
  std::string dataPath;
  int32_t thresholdHough;

  priv_nh.param("data_path", dataPath, std::string(DATA_PATH));
  priv_nh.param("threshold_hough", thresholdHough, 50);

  Perception perception;
  perception.setDataPath(dataPath);
  perception.loadTemplates(thresholdHough);
  perception.trainConfidences();

  ros::shutdown();
  return 0;
}
