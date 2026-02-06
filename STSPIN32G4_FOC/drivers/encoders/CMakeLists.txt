add_library(mt6825 STATIC
        mt6825.cpp
)
target_include_directories(mt6825 PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(mt6825 PUBLIC board_foc)
