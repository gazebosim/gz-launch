set (plugins
  camera
  gazebo
  gazebo_gui
  image_compress
  joystick
  joy_to_twist
)

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
