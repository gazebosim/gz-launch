file(GLOB files "*.ign")
install(FILES ${files}
  DESTINATION ${IGN_DATA_INSTALL_DIR}/configs)
