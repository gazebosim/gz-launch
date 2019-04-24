/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
 *
*/

#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/performer.pb.h>

#include <ignition/common/Util.hh>
#include <ignition/common/Console.hh>
#include "GazeboFactory.hh"

using namespace ignition;
using namespace ignition::launch;

/////////////////////////////////////////////////
GazeboFactory::GazeboFactory()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
bool GazeboFactory::Load(const tinyxml2::XMLElement *_elem)
{
  const tinyxml2::XMLElement *elem;

  msgs::EntityFactory req;

  // Set the sdf field, if an SDF string has been specified.
  tinyxml2::XMLPrinter printer;
  elem = _elem->FirstChildElement("sdf");
  if (elem)
  {
    elem->Accept(&printer);
    req.set_sdf(printer.CStr());
  }

  // Get <name>
  elem = _elem->FirstChildElement("name");
  if (elem)
    req.set_name(elem->GetText());

  // Get <is_performer>, by default we assume a spawned model is a performer
  bool isPerformer = true;
  elem = _elem->FirstChildElement("is_performer");
  if (elem)
  {
    std::string str = elem->GetText();
    isPerformer = str == "1" || common::lowercase(str) == "true";
  }

  // Get <allow_renaming>
  elem = _elem->FirstChildElement("allow_renaming");
  if (elem)
  {
    std::string str = elem->GetText();
    req.set_allow_renaming(str == "1" || common::lowercase(str) == "true");
  }

  // Get the pose
  elem = _elem->FirstChildElement("pose");
  if (elem)
  {
    ignition::math::Pose3d pose;
    std::stringstream stream;
    stream << elem->GetText();
    stream >> pose;
    msgs::Set(req.mutable_pose(), pose);
  }

  // Get a user-defined world name
  elem = _elem->FirstChildElement("world");
  std::string worldName;
  if (elem)
  {
    worldName = elem->GetText();
  }
  // Otherwise get a world name using transport.
  else
  {
    std::vector<std::string> topics;
    std::set<std::string> worlds;

    // Get all the topics
    this->node.TopicList(topics);

    // Find the topics that start with "/world/".
    for (const std::string &topic : topics)
    {
      if (topic.find("/world/") == 0)
      {
        // Get what we would assume is the world name
        std::vector<std::string> split = common::split(topic, "/");
        worlds.insert(split[1]);
      }
    }

    // Error if no world was found.
    if (worlds.empty())
    {
      ignerr << "No simulation worlds were found. Unable to run the factory. "
        << "Is Gazebo running?\n";
      return false;
    }

    // Warning if multiple worlds were found.
    if (worlds.size() > 1)
    {
      ignwarn << "Multiple simulation worlds were found. Using the first, "
        << " which has the name[" << *worlds.begin() << "]\n";
    }

    worldName = *worlds.begin();
  }

  unsigned int timeout = 2000;
  msgs::Boolean rep;
  bool result;

  std::string topic = "/world/";
  topic += worldName + "/create";

  // Send the request.
  bool executed = this->node.Request(topic, req, timeout, rep, result);

  if (executed && result && rep.data())
  {
    igndbg << "Factory service call succeeded.\n";
    if (isPerformer)
    {
      IGN_SLEEP_S(2);
      topic = std::string("/world/") + worldName + "/level/add_performer";
      msgs::StringMsg performerReq;
      performerReq.set_data(req.name());
      // \todo(anyone) Setting the size to 2,2,2 is a hack. Gazebo should
      // calculate the bounding box based on the model information.
      executed = this->node.Request(topic, performerReq, timeout, rep, result);

      // msgs::Performer performerReq;
      // performerReq.set_name(req.name());
      // // \todo(anyone) Setting the size to 2,2,2 is a hack. Gazebo should
      // // calculate the bounding box based on the model information.
      //  msgs::Set(performerReq.mutable_geometry()->mutable_box()->mutable_size(), math::Vector3d(2, 2, 2));
      // executed = this->node.Request(topic, performerReq, timeout, rep, result);
    }
  }
  else
  {
    if (executed)
    {
      if (result && !rep.data())
      {
        ignerr << "Factory service call completed, but returned a false value."
          << "You may have an invalid request. Check the configuration.\n";
      }
      else
      {
        ignerr << "Factory service call failed.\n";
      }
    }
    else
    {
      ignerr << "Factory service call timed out.\n";
    }
  }

  return false;
}
