stm32_create_cube_lib(board_BLDC ${CMAKE_CURRENT_LIST_DIR}/BLDC STM32G431VBTX_FLASH.ld)
stm32_create_cube_lib(board_foc ${CMAKE_CURRENT_LIST_DIR}/foc STM32G431VBTX_FLASH.ld)

stm32_create_cube_lib(board_bootloader ${CMAKE_CURRENT_LIST_DIR}/bootloader STM32G431VBTX_FLASH.ld)
