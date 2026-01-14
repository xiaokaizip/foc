add_library(app_BLDC STATIC
        main.cpp

)
target_include_directories(app_BLDC PUBLIC ${CMAKE_CURRENT_LIST_DIR} )

target_link_libraries(app_BLDC PUBLIC lwprintf )

