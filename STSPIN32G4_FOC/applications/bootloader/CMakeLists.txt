include_directories(./)

file(GLOB_RECURSE SOURCES_APP "./*.*")

add_executable(bootloader.elf ${SOURCES_APP})
target_link_libraries(bootloader.elf PRIVATE board_bootloader lwprintf)
stm32_create_hex(bootloader)

