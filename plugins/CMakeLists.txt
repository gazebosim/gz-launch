set (plugins
  joystick
)

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
