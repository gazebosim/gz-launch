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

#include <gtest/gtest.h>
#include <cstdio>
#include <cstdlib>

#include <string>

#include <ignition/common/Filesystem.hh>

#include "ignition/launch/test_config.hh"  // NOLINT(build/include)

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#endif

/////////////////////////////////////////////////
std::string customExecStr(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != nullptr)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(CmdLine, Ls)
{
#ifdef _WIN32
  std::string cmd = std::string("set IGN_CONFIG_PATH=") + IGN_CONFIG_PATH +
  " && ign launch " +
#else
  std::string cmd = std::string("IGN_CONFIG_PATH=") + IGN_CONFIG_PATH +
  " ign launch " +
#endif
    ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "config", "ls.ign");

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);
  std::cout << output << std::endl;
  EXPECT_TRUE(output.find("CMakeFiles") != std::string::npos) << output;
  EXPECT_TRUE(output.find("Makefile") != std::string::npos) << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, EchoSelf)
{
  std::string filePath =
    ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "config", "echo.ign");
  std::string cmd = std::string("IGN_CONFIG_PATH=") + IGN_CONFIG_PATH +
    " ign launch " + filePath;

  std::string output = customExecStr(cmd);
  EXPECT_EQ(filePath, output) << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, HelpSelf)
{
  std::string cmd = std::string("IGN_CONFIG_PATH=") + IGN_CONFIG_PATH +
    " ign launch --help";

  std::string output = customExecStr(cmd);
  EXPECT_NE(std::string::npos,
    output.find("Introspect Ignition launch")) << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, EchoErb)
{
  std::string filePath =
    ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "config", "echo.ign");

  std::string cmd = std::string("IGN_CONFIG_PATH=") + IGN_CONFIG_PATH +
    " ign launch " + filePath + " testVar:=erb1234";

  std::string output = customExecStr(cmd);
  EXPECT_EQ("erb1234", output) << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, EchoBadErb)
{
   std::string filePath =
    ignition::common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "config", "echo.ign");

  std::string cmd = std::string("IGN_CONFIG_PATH=") + IGN_CONFIG_PATH +
    " ign launch " + filePath + " badargument";

  std::string output = customExecStr(cmd);
  EXPECT_NE(std::string::npos, output.find("is wrong for erb")) << output;
}
