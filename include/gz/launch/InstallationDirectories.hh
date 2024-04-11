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

#ifndef GZ_LAUNCH_INSTALLATION_DIRECTORIES_HH_
#define GZ_LAUNCH_INSTALLATION_DIRECTORIES_HH_

#include <string>

#include <gz/launch/config.hh>
#include <gz/launch/Export.hh>

namespace gz
{
  namespace launch
  {
    inline namespace GZ_LAUNCH_VERSION_NAMESPACE {

    /// \brief getInstallPrefix return the install prefix of the library
    /// i.e. CMAKE_INSTALL_PREFIX unless the library has been moved
    GZ_LAUNCH_VISIBLE std::string getInstallPrefix();

    /// \brief getPluginInstallPath return the plugin install path
    GZ_LAUNCH_VISIBLE std::string getPluginInstallPath();

    /// \brief getInitialConfigPath return the initial config path
    GZ_LAUNCH_VISIBLE std::string getInitialConfigPath();

    }
  }
}

#endif
