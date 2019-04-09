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
#ifndef IGNITION_LAUNCH_GAZEBOFACTORY_HH_
#define IGNITION_LAUNCH_GAZEBOFACTORY_HH_

#include <memory>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include "ignition/launch/Plugin.hh"

namespace ignition
{
  namespace launch
  {
    /// \brief Spawns entities into simulation.
    ///
    /// # Example usage
    /// <!-- Run the gazebo server with a set of plugins -->
    /// <plugin name="ignition::launch::GazeboServer"
    ///         filename="libignition-launch0-gazebo.so">
    ///   <!-- The SDF file to run -->
    ///   <world_file>diff_drive.sdf</world_file>
    ///
    ///   <!-- The physics system -->
    ///   <plugin entity_name="<%= worldName %>"
    ///           entity_type="world"
    ///           filename="libignition-gazebo-physics-system.so"
    ///           name="ignition::gazebo::systems::v0::Physics">
    ///   </plugin>
    ///
    ///   <!-- Specify any other ignition gazebo plugins here. -->
    /// </plugin>
    class GazeboFactory : public ignition::launch::Plugin
    {
      /// \brief Constructor.
      public: GazeboFactory();

      /// \brief Destructor.
      public: virtual ~GazeboFactory() = default;

      // Documentation inherited.
      public: virtual void Load(
                  const tinyxml2::XMLElement *_elem) override final;

      private: transport::Node node;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::GazeboFactory, ignition::launch::Plugin)

#endif
