set (plugins
  camera
  image_compress
  joystick
)

foreach (plugin ${plugins})
  add_subdirectory(${plugin})
endforeach ()
