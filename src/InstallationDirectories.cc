/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <gz/launch/config.hh>
#include <gz/launch/InstallationDirectories.hh>

#include <gz/common/Filesystem.hh>

namespace gz
{
namespace launch
{
inline namespace GZ_LAUNCH_VERSION_NAMESPACE {

std::string getPluginInstallPath()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_LAUNCH_PLUGIN_RELATIVE_INSTALL_PATH);
}

std::string getInitialConfigPath()
{
  return gz::common::joinPaths(
      getInstallPrefix(), GZ_LAUNCH_INITIAL_CONFIG_RELATIVE_PATH);
}

}
}
}
