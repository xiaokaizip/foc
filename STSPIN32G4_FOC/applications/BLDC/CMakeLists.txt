

include_directories(./)

file(GLOB_RECURSE SOURCES_APP "./*.*")

add_executable(BLDC.elf ${SOURCES_APP})
target_link_libraries(BLDC.elf PRIVATE board_BLDC lwprintf)
stm32_create_hex(BLDC)

