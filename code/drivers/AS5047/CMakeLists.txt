add_library(AS5047 STATIC
        AS5047.c
)
target_include_directories(AS5047 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(AS5047 PUBLIC lwprintf stm32cubemx)
