if (websockets_FOUND)
  set (plugin ignition-launch-websocket-server)
  set (sources WebsocketServer.cc)

  add_library(${plugin} SHARED ${sources})

  file (READ combined.proto WEBSOCKETSERVER_MESSAGE_DEFINITIONS)
  configure_file("MessageDefinitions.hh.in"
    "${CMAKE_CURRENT_BINARY_DIR}/MessageDefinitions.hh" @ONLY)
  # Add a dependency on binary source dir so that you can change the
  # combinded.proto file and `make` will rebuild the plugin.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS combined.proto)
  target_include_directories(${plugin} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/>
  )

  target_link_libraries(${plugin}
    PRIVATE
      ${websockets_LIBRARIES}
      ${PROJECT_LIBRARY_TARGET_NAME}
      ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
      ignition-common${IGN_COMMON_MAJOR_VER}::graphics
      ignition-msgs${IGN_MSGS_MAJOR_VER}::core
      ignition-plugin${IGN_PLUGIN_MAJOR_VER}::core
      ignition-transport${IGN_TRANSPORT_MAJOR_VER}::core
  )

  install (TARGETS ${plugin} DESTINATION ${GZ_LAUNCH_PLUGIN_INSTALL_PATH})
endif()
