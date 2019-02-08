/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>
#include "GazeboServer.hh"

using namespace ignition;

/////////////////////////////////////////////////
GazeboServer::GazeboServer()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
GazeboServer::~GazeboServer()
{
  std::cout << "GazeboServer shutdown\n";
}

/////////////////////////////////////////////////
void GazeboServer::Load(const tinyxml2::XMLElement *_elem)
{
  gazebo::ServerConfig serverConfig;
  const tinyxml2::XMLElement *elem;
  elem = _elem->FirstChildElement("world_file");
  if (elem)
    serverConfig.SetSdfFile(elem->GetText());


  elem = _elem->FirstChildElement("plugin");
  while (elem)
  {
    std::string filename = elem->Attribute("filename");
    std::string name = elem->Attribute("name");
    serverConfig.AddPlugin(filename, name);
    elem = elem->NextSiblingElement("plugin");
  }

  this->server.reset(new gazebo::Server(serverConfig));
  this->server->Run(false);

  igndbg << "Loaded GazeboServer plugin with the following parameters:\n";
}
