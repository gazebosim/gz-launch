set (plugin Camera)
set (sources Camera.cc UVCCamera.cc)

string (TOLOWER ${plugin} plugin_lower)
add_library(${plugin_lower} SHARED ${sources})
target_link_libraries(${plugin_lower} 
    PRIVATE
    ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
    ignition-plugin${IGN_PLUGIN_VER}::core
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    )
install (TARGETS ${plugin_lower} DESTINATION ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins/)
