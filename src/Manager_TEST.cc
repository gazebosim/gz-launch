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

static const std::string kTestScriptPath = "/tmp/ign-launch.sh";

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
  ofs << R"(#!/usr/bin/env bash
echo $TEST_VAR
touch $TEST_VAR
)";
  chmod(kTestScriptPath.c_str(), S_IRWXU);
  return true;
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunEmptyConfig)
{
  ignition::launch::Manager mgr;

  // Expect false because no config file was set.
  EXPECT_FALSE(mgr.RunConfig(""));
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, MissingIgnition)
{
  std::string config =
    "<executable name='gazebo'>"
    "  <command>ign-gazebo-server</command>"
    "</executable>";

  ignition::launch::Manager mgr;

  // Stop the manager after a short pause.
  EXPECT_FALSE(mgr.RunConfig(config));
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunBadXml)
{
  std::string config =
    "<ignition version='1.0'>"
    " executable></executable"
    "</ignition>";

  ignition::launch::Manager mgr;

  // Stop the manager after a short pause.
  EXPECT_FALSE(mgr.RunConfig(config));
}

/////////////////////////////////////////////////
TEST(Ignition_TEST, RunLs)
{
  std::string cmd;

#ifdef _MSCV_VER
  cmd = "dir";
#else
  cmd = "ls";
#endif

  std::string config =
    "<ignition version='1.0'>"
    "  <executable name='ls'>"
    "    <command>" + cmd + "</command>"
    "  </executable>"
    "</ignition>";

  ignition::launch::Manager mgr;

  // We should be able to run the ls command. This does not check the
  // output.
  EXPECT_TRUE(mgr.RunConfig(config));
}


/////////////////////////////////////////////////
TEST(Ignition_TEST, IGN_UTILS_TEST_DISABLED_ON_WIN32(RunEnvPre))
{
  // Test that environment is applied regardless of order
  std::string testPath = "/tmp/ign-launch-env-test-pre";

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
    <command>/tmp/ign-launch.sh</command>
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
TEST(Ignition_TEST, IGN_UTILS_TEST_DISABLED_ON_WIN32(RunEnvPost))
{
  // Test that environment is applied regardless of order
  std::string testPath = "/tmp/ign-launch-env-test-post";

  if (ignition::common::isFile(testPath))
  {
    ASSERT_TRUE(ignition::common::removeFile(testPath));
  }

  ASSERT_TRUE(WriteTestScript());

  std::string config = R"(
<ignition version='1.0'>
  <executable name='touch'>
    <command>/tmp/ign-launch.sh</command>
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
