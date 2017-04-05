/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <gflags/gflags.h>
#include "ignition/launch/Config.hh"
#include "ignition/launch/Manager.hh"

// Gflag command line argument definitions
DEFINE_bool(h, false, "Show help");
DEFINE_string(run, "", "Run an executable");
DEFINE_string(config, "",
    "Run one or more executables from a configuration file.");
DEFINE_int32(verbose, 1,
    "Set the verbose level (0-4). 0 = no output, 1 = adds error messages, "
    "2 adds warning messages, 3 adds info message, and 4 adds debug message");

int main(int _argc, char **_argv)
{
  std::string usage("Ignition runs and manages programs and plugins.");
  usage += " Sample usage:\n ignition -run \"<program_name> <arg1>...<argN>\"";

  gflags::SetUsageMessage(usage);
  gflags::SetVersionString(IGNITION_LAUNCH_VERSION_FULL);

  gflags::ParseCommandLineNonHelpFlags(&_argc, &_argv, true);

  std::vector<gflags::CommandLineFlagInfo> flags;
  gflags::GetAllFlags(&flags);

  bool showHelp = false;
  std::map<std::string, std::string> args;
  for (auto const &flag : flags)
  {
    if (!flag.current_value.empty())
      args[flag.name] = flag.current_value;
    showHelp = showHelp || (flag.name.find("help") != std::string::npos &&
                            flag.current_value == "true");
  }

  // SHow help, if specified
  if (showHelp || FLAGS_h)
  {
    gflags::SetCommandLineOptionWithMode("help", "false",
        gflags::SET_FLAGS_DEFAULT);
    gflags::SetCommandLineOptionWithMode("helpshort", "true",
        gflags::SET_FLAGS_DEFAULT);
  }
  gflags::HandleCommandLineHelpFlags();

  // Create the manager
  ignition::launch::Manager mgr;

  // Run the manager
  return mgr.Run(args);
}
