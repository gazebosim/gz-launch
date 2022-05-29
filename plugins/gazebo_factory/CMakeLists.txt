set (plugin ignition-launch-gazebo-factory)
set (sources GazeboFactory.cc)

string (TOLOWER ${plugin} plugin_lower)

add_library(${plugin_lower} SHARED ${sources})
target_link_libraries(${plugin_lower}
  PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  ignition-gazebo${IGN_GAZEBO_MAJOR_VER}::ignition-gazebo${IGN_GAZEBO_MAJOR_VER}
  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
  ignition-transport${IGN_TRANSPORT_MAJOR_VER}::core
  ignition-plugin${IGN_PLUGIN_MAJOR_VER}::core
)

install (TARGETS ${plugin} DESTINATION ${GZ_LAUNCH_PLUGIN_INSTALL_PATH})
