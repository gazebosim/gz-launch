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
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>

#include <ignition/utilities/ExtraTestMacros.hh>

#include <sys/stat.h>

#include "Manager.hh"

#ifndef _WIN32
static constexpr char kTestScriptPath[] = "/tmp/ign-launch.sh";
#else
std::string kTestScriptPath = std::string(getenv("localappdata")) + "\\Temp\\ign-launch.bat";
#endif

/////////////////////////////////////////////////
bool RemoveTestScript()
{
  // Remove the file if it already exists
  if (ignition::common::isFile(kTestScriptPath))
  {
    if (!ignition::common::removeFile(kTestScriptPath))
    {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
bool WriteTestScript()
{
  if (!RemoveTestScript())
    return false;

  // Write a simple script and mark it executable
  std::ofstream ofs(kTestScriptPath);
#ifndef WIN32
  ofs << R"(#!/usr/bin/env bash
echo $TEST_VAR
touch $TEST_VAR
)";
  chmod(kTestScriptPath, S_IRWXU);
#else
  ofs << R"(echo %TEST_VAR%
type nul > %TEST_VAR%
)";
  _chmod(kTestScriptPath.c_str(), _S_IREAD | _S_IWRITE);
#endif
  return true;
}

// /////////////////////////////////////////////////
// TEST(Ignition_TEST, RunEmptyConfig)
// {
//   ignition::launch::Manager mgr;
//
//   // Expect false because no config file was set.
//   EXPECT_FALSE(mgr.RunConfig(""));
// }
//
// /////////////////////////////////////////////////
// TEST(Ignition_TEST, MissingIgnition)
// {
//   std::string config =
//     "<executable name='gazebo'>"
//     "  <command>ign-gazebo-server</command>"
//     "</executable>";
//
//   ignition::launch::Manager mgr;
//
//   // Stop the manager after a short pause.
//   EXPECT_FALSE(mgr.RunConfig(config));
// }
//
// /////////////////////////////////////////////////
// TEST(Ignition_TEST, RunBadXml)
// {
//   std::string config =
//     "<ignition version='1.0'>"
//     " executable></executable"
//     "</ignition>";
//
//   ignition::launch::Manager mgr;
//
//   // Stop the manager after a short pause.
//   EXPECT_FALSE(mgr.RunConfig(config));
// }
//
/////////////////////////////////////////////////
TEST(Ignition_TEST, RunLs)
{
  std::string cmd;

#ifdef _WIN32
  cmd = "dir";
#else
  cmd = "ls";
#endif

  std::string config =
    "<ignition version='1.0'>"
    "  <executable name='listDirectory'>"
    "    <command>" + cmd + "</command>"
    "  </executable>"
    "  <executable name='listDirectory'>"
    "    <command>dir C:\</command>"
    "  </executable>"
    "</ignition>";

  ignition::launch::Manager mgr;

  // We should be able to run the ls command. This does not check the
  // output.
  EXPECT_TRUE(mgr.RunConfig(config));
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunEnvPre)
{
  // Test that environment is applied regardless of order
  #ifndef _WIN32
  std::string testPath = "/tmp/ign-launch-env-test-pre";
  #else
  std::string testPath =
    std::string(getenv("localappdata")) + "\\Temp\\ign-launch-env-test-pre";
  #endif

  if (ignition::common::isFile(testPath))
  {
    ASSERT_TRUE(ignition::common::removeFile(testPath));
  }

  ASSERT_TRUE(WriteTestScript());

  std::string config = R"(
<ignition version='1.0'>
  <env>
    <name>TEST_VAR</name>
    <value>)" + testPath + R"(</value>
  </env>
  <executable name='touch'>
    <command>)" + kTestScriptPath + R"(</command>
  </executable>
</ignition>
)";

  ignition::launch::Manager mgr;

  EXPECT_TRUE(mgr.RunConfig(config));
  EXPECT_TRUE(ignition::common::isFile(testPath));
  EXPECT_TRUE(ignition::common::removeFile(testPath));
  EXPECT_TRUE(RemoveTestScript());
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunEnvPost)
{
  // Test that environment is applied regardless of order
  #ifndef _WIN32
  std::string testPath = "/tmp/ign-launch-env-test-post";
  #else
  std::string testPath =
    std::string(getenv("localappdata")) + "\\Temp\\ign-launch-env-test-post";
  #endif

  if (ignition::common::isFile(testPath))
  {
    ASSERT_TRUE(ignition::common::removeFile(testPath));
  }

  ASSERT_TRUE(WriteTestScript());

  std::string config = R"(
<ignition version='1.0'>
  <executable name='touch'>
    <command>)" + kTestScriptPath + R"(</command>
  </executable>
  <env>
    <name>TEST_VAR</name>
    <value>)" + testPath + R"(</value>
  </env>
</ignition>
)";

  ignition::launch::Manager mgr;

  EXPECT_TRUE(mgr.RunConfig(config));
  EXPECT_TRUE(ignition::common::isFile(testPath));
  EXPECT_TRUE(ignition::common::removeFile(testPath));
  EXPECT_TRUE(RemoveTestScript());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ignition::common::Console::SetVerbosity(4);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
