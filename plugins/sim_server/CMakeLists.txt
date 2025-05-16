set (plugin gz-launch-sim)
set (sources SimServer.cc)

string (TOLOWER ${plugin} plugin_lower)

add_library(${plugin_lower} SHARED ${sources})
target_link_libraries(${plugin_lower}
  PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  gz-sim::gz-sim
  gz-common::gz-common
  gz-transport::core
  gz-plugin::core
)

install (TARGETS ${plugin} DESTINATION ${GZ_LAUNCH_PLUGIN_RELATIVE_INSTALL_PATH})
