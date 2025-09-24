add_library(app STATIC
        main.cpp ../drivers/drv8301/delay.c position_sensor.cpp application.cpp vofa_plus.c open_loop_controller.cpp math_ops.c FastMath.c velocityPositionLoopController.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(app PUBLIC lwprintf stm32cubemx drv8301 AS5047)
