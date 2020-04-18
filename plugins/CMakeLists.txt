set (plugins
  joy_to_twist
  gazebo_gui
  gazebo_factory
  gazebo_server
  realsense
  websocket_server
)

if (UNIX AND NOT APPLE)
  set(plugins ${plugins}
        joystick)
endif()

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
