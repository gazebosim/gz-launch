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
    /// \brief Runs the Ignition Gazebo GUI.
    ///
    /// # Example usage
    ///
    /// <!-- The GUI wants to be in its own process, so wrap the plugin -->
    /// <executable_wrapper>
    ///   <plugin name="ignition::launch::GazeboGui"
    ///           filename="libignition-launch0-gazebogui.so">
    ///   <!-- Add Ignition GUI plugins here -->
    ///   </plugin>
    /// </executable_wrapper>
    class GazeboGui : public ignition::launch::Plugin
    {
      /// \brief Constructor.
      public: GazeboGui();

      /// \brief Destructor.
      public: virtual ~GazeboGui();

      // Documentation inherited.
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;

      /// \brief Run the GUI
      private: void Run();

      /// \brief Private data pointer
      private: std::unique_ptr<ignition::gui::Application> app;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::GazeboGui, ignition::launch::Plugin)

#endif
