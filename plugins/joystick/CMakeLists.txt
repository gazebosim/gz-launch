set (plugin Joystick)
set (sources Joystick.cc)

string (TOLOWER ${plugin} plugin_lower)

add_library(${plugin_lower} SHARED ${sources})
target_link_libraries(${plugin_lower}
  PRIVATE
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
    ignition-transport${IGN_TRANSPORT_VER}::core
    ignition-plugin${IGN_PLUGIN_VER}::core
)

install (TARGETS ${plugin_lower} DESTINATION ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/)
