if (websockets_FOUND)
  set (plugin ignition-launch${PROJECT_VERSION_MAJOR}-websocket-server)
  set (sources WebsocketServer.cc)
  
  add_library(${plugin} SHARED ${sources})
  target_link_libraries(${plugin}
    PRIVATE
      ${websockets_LIBRARIES}
      ${PROJECT_LIBRARY_TARGET_NAME}
      ignition-common${IGN_COMMON_MAJOR_VER}::ignition-common${IGN_COMMON_MAJOR_VER}
      ignition-msgs${IGN_MSGS_MAJOR_VER}::core
      ignition-plugin${IGN_PLUGIN_MAJOR_VER}::core
      ignition-transport${IGN_TRANSPORT_MAJOR_VER}::core
  )
  
  install (TARGETS ${plugin} DESTINATION ${IGN_LAUNCH_PLUGIN_INSTALL_PATH})
endif()
