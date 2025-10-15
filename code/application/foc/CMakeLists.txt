add_library(app_foc STATIC
        main.cpp position_sensor.cpp application.cpp vofa_plus.c
        loop/open_loop_controller.cpp math/math_ops.c
        loop/velocityPositionLoopController.cpp loop/currentControl.cpp
        math/FastMath.c
        comnunication/serial_modbus.c calibration/currentCalibration.cpp
        calibration/calibrationPhase.cpp

)
target_include_directories(app_foc PUBLIC ${CMAKE_CURRENT_LIST_DIR} comnunication calibration math)

target_link_libraries(app_foc PUBLIC lwprintf stm32cubemx drv8301 AS5047)
