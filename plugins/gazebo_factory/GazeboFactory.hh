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
    ///  <plugin name="ignition::launch::GazeboFactory"
    ///          filename="ignition-launch-gazebo-factory">
    ///
    ///   <spawn>
    ///     <!-- Name to give the model -->
    ///     <name>x2</name>
    ///
    ///     <!-- Allow the model to be renamed when the given name is already
    ///          taken -->
    ///     <allow_renaming>true</allow_renaming>
    ///
    ///     <!-- Pose of the model -->
    ///     <pose>1 2 0.5 0 0 0</pose>
    ///
    ///     <!-- SDF snippet that contains information about what should be
    ///          spawned -->
    ///     <sdf version='1.6'>
    ///       <include>
    ///         <uri>https://fuel.ignitionrobotics.org/1.0/openrobotics/models/X2 UGV/1</uri>
    ///         <!-- Publish robot state information -->
    ///         <plugin filename="ignition-gazebo-state-publisher-system"
    ///              name="ignition::gazebo::systems::StatePublisher"></plugin>
    ///       </include>
    ///     </sdf>
    ///   </spawn>
    ///
    ///   <spawn>
    ///   ...
    ///   </spawn>
    /// </plugin>
    class GazeboFactory : public ignition::launch::Plugin
    {
      /// \brief Constructor.
      public: GazeboFactory();

      /// \brief Destructor.
      public: virtual ~GazeboFactory() = default;

      // Documentation inherited.
      public: virtual bool Load(
                  const tinyxml2::XMLElement *_elem) override final;

      /// \brief Process a spawn XML request and generate a new entry
      /// in the worldFactoryMsgs map.
      private: void ProcessSpawn(const tinyxml2::XMLElement *_elem);

      // Transport node.
      private: transport::Node node;

      /// \brief Factory message for each world. Key = world name,
      /// value = entities to spawn.
      private: std::map<std::string, msgs::EntityFactory_V> worldFactoryMsgs;

      /// \brief Performer messages for each world. Key = world name,
      /// value = vector of performer names.
      private: std::map<std::string, std::vector<std::string>> worldPerformers;
    };
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(ignition::launch::GazeboFactory, ignition::launch::Plugin)

#endif
