add_library(app STATIC
        main.cpp ../drivers/drv8301/delay.c position_sensor.cpp application.cpp vofa_plus.c
        loop/open_loop_controller.cpp math/math_ops.c
        loop/velocityPositionLoopController.cpp loop/currentControl.cpp
        math/FastMath.c
        comnunication/serial_modbus.c calibration/currentCalibration.cpp
        calibration/calibration.c

)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR} comnunication calibration)

target_link_libraries(app PUBLIC lwprintf stm32cubemx drv8301 AS5047)
