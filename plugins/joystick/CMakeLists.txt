set (plugin gz-launch-joystick)
set (sources Joystick.cc)

add_library(${plugin} SHARED ${sources})
target_link_libraries(${plugin}
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-common::gz-common
    gz-math::gz-math
    gz-transport::core
    gz-plugin::core
)

install (TARGETS ${plugin} DESTINATION ${GZ_LAUNCH_PLUGIN_RELATIVE_INSTALL_PATH})
