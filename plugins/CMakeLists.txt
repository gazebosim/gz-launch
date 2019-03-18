set (plugins
  gazebo_gui
  gazebo_server
)

if (UNIX AND NOT APPLE)
  set(plugins ${plugins}
        joystick
        joy_to_twist)
endif()

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
