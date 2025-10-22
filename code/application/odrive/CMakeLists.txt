add_library(app_odrive STATIC
        main.cpp PositionSensor/PositionSensor.cpp comnunication/serial_modbus.c math/math_ops.c math/FastMath.c
        comnunication/vofa_plus.cpp Calibration/calibration.cpp PreferenceWriter/PrefrenceWriter.cpp
        FlashWriter/FlashWriter.cpp FOC/foc.cpp fsm.cpp comnunication/serial_common.cpp
        comnunication/can/can_common.cpp comnunication/can/can_simple.cpp

)
target_include_directories(app_odrive PUBLIC ${CMAKE_CURRENT_LIST_DIR} Calibration comnunication FlashWriter FOC
        math Config PositionSensor PreferenceWriter)

target_link_libraries(app_odrive PUBLIC lwprintf stm32cubemx drv8301 AS5047)
