set (plugins
  gazebo_gui
  gazebo_server
  joy_to_twist
)

if (UNIX AND NOT APPLE)
  set(plugins ${plugins}
              joystick)
endif()

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
