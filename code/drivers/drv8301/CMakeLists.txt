add_library(drv8301 STATIC
        drv8301.cpp delay.c
)
target_include_directories(drv8301 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(drv8301 PUBLIC lwprintf stm32cubemx)
