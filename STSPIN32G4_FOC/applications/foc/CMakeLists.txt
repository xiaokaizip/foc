

include_directories(./)

file(GLOB_RECURSE SOURCES_APP "./*.*")

add_executable(foc.elf ${SOURCES_APP})
target_link_libraries(foc.elf PRIVATE board_foc lwprintf)
stm32_create_hex(foc)

