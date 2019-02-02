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
#ifndef IGNITION_LAUNCH_PLUGIN_HH_
#define IGNITION_LAUNCH_PLUGIN_HH_

#include <map>
#include <string>

#include <ignition/plugin/SpecializedPluginPtr.hh>


namespace ignition
{
  namespace launch
  {
    class Plugin
    {
      public: virtual void Load(std::map<std::string, std::string> _params) = 0;
      public: virtual void Shutdown() = 0;
    };
    using PluginPtr = ignition::plugin::SpecializedPluginPtr<
      ignition::launch::Plugin>;
  }
}
#endif
