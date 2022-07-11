file(GLOB files "*.ign")
install(FILES ${files}
  DESTINATION ${GZ_DATA_INSTALL_DIR}/configs)
