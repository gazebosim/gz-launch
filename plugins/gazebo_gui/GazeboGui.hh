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
#ifndef IGNITION_LAUNCH_GAZEBOGUI_HH_
#define IGNITION_LAUNCH_GAZEBOGUI_HH_

#include <memory>
#include <ignition/gui/Application.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/launch/Plugin.hh>

namespace ignition
{
  namespace launch
  {
    class GazeboGui : public ignition::launch::Plugin
    {
      public: GazeboGui();
      public: virtual ~GazeboGui();
      public: virtual void Load(
                  const tinyxml2::XMLElement *_elem) override final;

      private: void Run();
      private: std::unique_ptr<ignition::gui::Application> app;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::GazeboGui, ignition::launch::Plugin)

#endif
