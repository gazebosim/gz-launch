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
#include <ignition/gazebo/config.hh>
#include <ignition/gui/MainWindow.hh>
#include "ignition/gazebo/gui/TmpIface.hh"

#include "ignition/launch/Events.hh"
#include "GazeboGui.hh"

using namespace ignition;

/////////////////////////////////////////////////
GazeboGui::GazeboGui()
  : ignition::launch::Plugin()
{
}

/////////////////////////////////////////////////
GazeboGui::~GazeboGui()
{
}

/////////////////////////////////////////////////
void GazeboGui::Load(const tinyxml2::XMLElement *_elem)
{
  this->connection = launch::Events::runEvent.Connect(
      std::bind(&GazeboGui::Run, this));

  int argc;
  char **argv = nullptr;

  this->app.reset(new ignition::gui::Application(argc, argv));

  // Load configuration file
  std::string configPath = ignition::common::joinPaths(
      IGNITION_GAZEBO_GUI_CONFIG_PATH, "gui.config");

  if (!app->LoadConfig(configPath))
  {
    ignerr << "Unable to load GazeboGui config file[" << configPath << "]\n";
    return;
  }

  // Customize window
  auto win = this->app->findChild<ignition::gui::MainWindow *>()->QuickWindow();
  win->setProperty("title", "Gazebo");

  // Temporary transport interface
  auto tmp = std::make_unique<ignition::gazebo::TmpIface>();

  // Let QML files use TmpIface' functions and properties
  auto context = new QQmlContext(this->app->Engine()->rootContext());
  context->setContextProperty("TmpIface", tmp.get());

  // Instantiate GazeboDrawer.qml file into a component
  QQmlComponent component(this->app->Engine(), ":/Gazebo/GazeboDrawer.qml");
  auto gzDrawerItem = qobject_cast<QQuickItem *>(component.create(context));
  if (gzDrawerItem)
  {
    // C++ ownership
    QQmlEngine::setObjectOwnership(gzDrawerItem, QQmlEngine::CppOwnership);

    // Add to main window
    auto parentDrawerItem = win->findChild<QQuickItem *>("sideDrawer");
    gzDrawerItem->setParentItem(parentDrawerItem);
    gzDrawerItem->setParent(this->app->Engine());
  }
  else
  {
    ignerr << "Failed to instantiate custom drawer, drawer will be empty"
      << std::endl;
  }

  const tinyxml2::XMLElement *elem = nullptr;
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

    ignition::gui::App()->LoadPlugin(file, elem);
  }

  // This blocks until the window is closed or we receive a SIGINT
  this->app->exec();
  std::cout << "DONE!\n";
  this->app.reset();
  // Run main window.
  igndbg << "Running the GazeboGui plugin.\n";
}

//////////////////////////////////////////////////
void GazeboGui::Run()
{
  this->connection.reset();

  std::cout << "DONE event more!\n";
}
