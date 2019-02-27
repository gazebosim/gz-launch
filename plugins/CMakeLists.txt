set (plugins
  joystick
  joy_to_twist
)

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
