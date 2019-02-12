set (plugin GazeboGuiLaunch)
set (sources GazeboGui.cc)

string (TOLOWER ${plugin} plugin_lower)

# ign_find_package(ignition-launch REQUIRED PRIVATE)
# set(IGN_LAUNCH_MAJOR_VER ${ignition-launch_VERSION_MAJOR})

add_library(${plugin_lower} SHARED ${sources})
target_link_libraries(${plugin_lower}
  PRIVATE
  ignition-gazebo${IGN_GAZEBO_MAJOR_VER}::ignition-gazebo${IGN_GAZEBO_MAJOR_VER}
  ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
  ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::core
  sdformat8::sdformat8
  ign-gazebo-gui-lib
)

install (TARGETS ${plugin_lower} DESTINATION ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/)
