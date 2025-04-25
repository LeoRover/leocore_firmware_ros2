macro(show_sections_size TARGET)
  add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_SIZE} -Ax $<TARGET_FILE:${TARGET}>
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>
  )
endmacro()

macro(generate_hex_bin TARGET)
  add_custom_command(
    TARGET ${TARGET}
    POST_BUILD
    COMMAND
      ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}>
      "$<TARGET_FILE_DIR:${TARGET}>/$<TARGET_FILE_BASE_NAME:${TARGET}>.hex"
  )

  add_custom_command(
    TARGET ${TARGET}
    POST_BUILD
    COMMAND
      ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}>
      "$<TARGET_FILE_DIR:${TARGET}>/$<TARGET_FILE_BASE_NAME:${TARGET}>.bin"
  )
endmacro()
