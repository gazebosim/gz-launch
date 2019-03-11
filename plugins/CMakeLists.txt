set (plugins
  gazebo_gui
  gazebo_server
  joystick
  joy_to_twist
)

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
