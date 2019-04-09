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

#include <ignition/common/Console.hh>
#include <sdf/sdf.hh>
#include "GazeboServer.hh"

using namespace ignition;
using namespace ignition::launch;

/////////////////////////////////////////////////
void copyElement(sdf::ElementPtr _sdf, const tinyxml2::XMLElement *_xml)
{
  _sdf->SetName(_xml->Value());
  if (_xml->GetText() != nullptr)
    _sdf->AddValue("string", _xml->GetText(), "1");

  for (const tinyxml2::XMLAttribute *attribute = _xml->FirstAttribute();
       attribute; attribute = attribute->Next())
  {
    _sdf->AddAttribute(attribute->Name(), "string", "", 1, "");
    _sdf->GetAttribute(attribute->Name())->SetFromString(
        attribute->Value());
  }

  // Iterate over all the child elements
  const tinyxml2::XMLElement *elemXml = nullptr;
  for (elemXml = _xml->FirstChildElement(); elemXml;
      elemXml = elemXml->NextSiblingElement())
  {
    sdf::ElementPtr element(new sdf::Element);
    element->SetParent(_sdf);

    copyElement(element, elemXml);
    _sdf->InsertElement(element);
  }
}

/////////////////////////////////////////////////
GazeboServer::GazeboServer()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
bool GazeboServer::Load(const tinyxml2::XMLElement *_elem)
{
  gazebo::ServerConfig serverConfig;
  const tinyxml2::XMLElement *elem;

  // Get the world file
  elem = _elem->FirstChildElement("world_file");
  if (elem)
    serverConfig.SetSdfFile(elem->GetText());

  // Set whether to use levels
  elem = _elem->FirstChildElement("levels");
  if (elem)
  {
    std::string str = elem->GetText();
    serverConfig.SetUseLevels(str == "1" ||
        common::lowercase(str) == "true");
  }

  // Get whether simulation should start paused.
  bool run = false;
  elem = _elem->FirstChildElement("run");
  if (elem)
  {
    std::string str = elem->GetText();
    run = str == "1" || common::lowercase(str) == "true";
  }

  // Process all the plugins.
  for (elem = _elem->FirstChildElement("plugin"); elem;
       elem = elem->NextSiblingElement("plugin"))
  {
    // Get the plugin's name
    const char *nameStr = elem->Attribute("name");
    std::string name = nameStr == nullptr ? "" : nameStr;
    if (name.empty())
    {
      ignerr << "A GazeboServer plugin is missing the name attribute. "
        << "Skipping this plugin.\n";
      continue;
    }

    // Get the plugin's filename
    const char *fileStr = elem->Attribute("filename");
    std::string file = fileStr == nullptr ? "" : fileStr;
    if (file.empty())
    {
      ignerr << "A GazeboServer plugin with name[" << name << "] is "
        << "missing the filename attribute. Skipping this plugin.\n";
      continue;
    }

    // Get the plugin's entity name attachment information.
    const char *entityNameStr = elem->Attribute("entity_name");
    std::string entityName = entityNameStr == nullptr ? "" : entityNameStr;
    if (entityName.empty())
    {
      ignerr << "A GazeboServer plugin with name[" << name << "] and "
        << "filename[" << file << "] is missing the entity_name attribute. "
        << "Skipping this plugin.\n";
      continue;
    }

    // Get the plugin's entity type attachment information.
    const char *entityTypeStr = elem->Attribute("entity_type");
    std::string entityType = entityTypeStr == nullptr ? "" : entityTypeStr;
    if (entityType.empty())
    {
      ignerr << "A GazeboServer plugin with name[" << name << "] and "
        << "filename[" << file << "] is missing the entity_type attribute. "
        << "Skipping this plugin.\n";
      continue;
    }

    // Create an SDF element of the plugin
    sdf::ElementPtr sdf(new sdf::Element);
    copyElement(sdf, elem);

    // Add the plugin to the server config
    serverConfig.AddPlugin({entityName, entityType, file, name, sdf});
  }

  // Create and run the simulation server
  this->server.reset(new gazebo::Server(serverConfig));
  this->server->Run(false, 0, !run);

  igndbg << "Loaded GazeboServer plugin.\n";
  return true;
}
