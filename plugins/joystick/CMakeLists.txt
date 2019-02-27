set (plugin ignition-launch-joystick${PROJECT_VERSION_MAJOR})
set (sources Joystick.cc)

add_library(${plugin} SHARED ${sources})
target_link_libraries(${plugin}
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
    ignition-transport${IGN_TRANSPORT_MAJOR_VER}::core
    ignition-plugin${IGN_PLUGIN_MAJOR_VER}::core
)

install (TARGETS ${plugin} DESTINATION ${IGN_LIB_INSTALL_DIR})
