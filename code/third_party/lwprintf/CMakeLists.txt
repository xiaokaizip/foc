cmake_minimum_required(VERSION 3.24)

# Setup project
project(LwLibPROJECT)

if(NOT PROJECT_IS_TOP_LEVEL)
    add_subdirectory(lwprintf)
else()
    # Set default compile flags for GCC
    if(CMAKE_COMPILER_IS_GNUCXX)
        message(STATUS "GCC detected, adding compile flags")
        set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -Wall -Wextra")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic -Wall -Wextra")
    endif(CMAKE_COMPILER_IS_GNUCXX)

    enable_testing()
    add_executable(${PROJECT_NAME})
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/dev/main.c
        ${CMAKE_CURRENT_LIST_DIR}/lwprintf/src/system/lwprintf_sys_win32.c
        ${CMAKE_CURRENT_LIST_DIR}/tests/test.c
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/dev
    )

    # Add subdir with lwprintf and link to the project
    set(LWPRINTF_OPTS_FILE ${CMAKE_CURRENT_LIST_DIR}/dev/lwprintf_opts.h)
    add_subdirectory(lwprintf)
    target_link_libraries(${PROJECT_NAME} lwprintf)

    # Add compile options to the library, which will propagate options to executable through public link
    target_compile_definitions(lwprintf PUBLIC WIN32 _DEBUG CONSOLE LWPRINTF_DEV)

    # Add test
    add_test(NAME Test COMMAND $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
endif()
