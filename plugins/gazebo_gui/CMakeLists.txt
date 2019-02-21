set (plugin ignition-plugins-gazebogui${PROJECT_VERSION_MAJOR})
set (sources GazeboGui.cc)

string (TOLOWER ${plugin} plugin_lower)

add_library(${plugin_lower} SHARED ${sources})
target_link_libraries(${plugin_lower}
  PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  ignition-gazebo${IGN_GAZEBO_MAJOR_VER}::ignition-gazebo${IGN_GAZEBO_MAJOR_VER}
  ignition-gui${IGN_GUI_MAJOR_VER}::ignition-gui${IGN_GUI_MAJOR_VER}
  ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
  ignition-transport${IGN_TRANSPORT_MAJOR_VER}::core
  ignition-plugin${IGN_PLUGIN_MAJOR_VER}::core
  sdformat8::sdformat8
  ign-gazebo-gui-lib
)

install (TARGETS ${plugin} DESTINATION ${IGN_LIB_INSTALL_DIR})
