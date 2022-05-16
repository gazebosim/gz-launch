#include <gtest/gtest.h>

#include <ignition/launch/Plugin.hh>
#include <ignition/utils/SuppressWarning.hh>

IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION

/////////////////////////////////////////////////
// Make sure the ignition namespace still works
TEST(Deprecated, IgnitionNamespace)
{
  ignition::launch::Plugin plugin;
}

IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
