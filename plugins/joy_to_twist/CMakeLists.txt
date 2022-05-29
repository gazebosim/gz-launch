set (plugin ignition-launch-joytotwist)
set (sources JoyToTwist.cc)

add_library(${plugin} SHARED ${sources})
target_link_libraries(${plugin}
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
    ignition-math${IGN_MATH_MAJOR_VER}::ignition-math${IGN_MATH_MAJOR_VER}
    ignition-plugin${IGN_PLUGIN_MAJOR_VER}::core
    ignition-transport${IGN_TRANSPORT_MAJOR_VER}::core
)

install (TARGETS ${plugin} DESTINATION ${GZ_LAUNCH_PLUGIN_INSTALL_PATH})
