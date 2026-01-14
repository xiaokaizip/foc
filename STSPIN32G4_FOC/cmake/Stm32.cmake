# =============================================
MACRO(stm32_create_cube_lib TARGET_NAME BOARD_DIR LINKER)

    if(NOT EXISTS  ${BOARD_DIR}/.mxproject)
        message(FATAL_ERROR ".mxproject not found in  ${BOARD_DIR}")
    endif()
    message(STATUS "Creating static library:  ${TARGET_NAME}")

    file(GLOB_RECURSE ${TARGET_NAME}_SRC "${BOARD_DIR}/Core/*.*" "${BOARD_DIR}/Drivers/*.*")
    add_library(${TARGET_NAME})
    target_sources(${TARGET_NAME} PUBLIC ${${TARGET_NAME}_SRC})

    # 初始化列表
    set(CUBE_DEFINES "")
    set(CUBE_INCLUDE_DIRS "")
    set(CUBE_SOURCES "")

    file(STRINGS  ${BOARD_DIR}/.mxproject LINES)

    set(IN_PREVIOUS_LIB_FILES FALSE)
    set(IN_PREVIOUS_USED_FILES FALSE)

    foreach(LINE ${LINES})
        # 检测节开始
        if(LINE STREQUAL "[PreviousLibFiles]")
            set(IN_PREVIOUS_LIB_FILES TRUE)
            set(IN_PREVIOUS_USED_FILES FALSE)
        elseif(LINE STREQUAL "[PreviousUsedCubeIDEFiles]")
            set(IN_PREVIOUS_LIB_FILES FALSE)
            set(IN_PREVIOUS_USED_FILES TRUE)
        elseif(LINE MATCHES "^\$.*\$ $")
            # 其他节，退出当前上下文
            set(IN_PREVIOUS_LIB_FILES FALSE)
            set(IN_PREVIOUS_USED_FILES FALSE)
        else()
            # 处理 PreviousLibFiles 节中的 LibFiles
            if(IN_PREVIOUS_LIB_FILES AND LINE MATCHES "^\\s*LibFiles=(.*)")
                string(REGEX MATCHALL "[^;]+" SRC_LIST " ${CMAKE_MATCH_1}")
                list(TRANSFORM SRC_LIST PREPEND ${BOARD_DIR}/)
                list(APPEND CUBE_SOURCES ${SRC_LIST})
            endif()

            # 处理 PreviousUsedCubeIDEFiles 节
            if(IN_PREVIOUS_USED_FILES)
                if(LINE MATCHES "^\\s*CDefines=(.*)")
                    string(REPLACE ";" ";" TMP_DEFINES "${CMAKE_MATCH_1}")
                    list(APPEND CUBE_DEFINES ${TMP_DEFINES})
                elseif(LINE MATCHES "^\\s*HeaderPath=(.*)")
                    string(REGEX MATCHALL "[^;]+" INCL_LIST "${CMAKE_MATCH_1}")
                    list(TRANSFORM INCL_LIST PREPEND ${BOARD_DIR}/)
                    list(APPEND CUBE_INCLUDE_DIRS ${INCL_LIST})
                elseif(LINE MATCHES "^\\s*SourceFiles=(.*)")
                    string(REGEX MATCHALL "[^;]+" SRC_LIST "${CMAKE_MATCH_1}")
                    list(TRANSFORM SRC_LIST PREPEND ${BOARD_DIR}/)
                    list(APPEND CUBE_SOURCES ${SRC_LIST})
                endif()
            endif()
        endif()
    endforeach()

    # 去重
    list(REMOVE_DUPLICATES CUBE_DEFINES)
    list(REMOVE_DUPLICATES CUBE_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES CUBE_SOURCES)

    # 只保留 .c 文件
    set(CUBE_SOURCES_C "")
    foreach(SRC  ${CUBE_SOURCES})
        if(SRC MATCHES "\\.c$" AND EXISTS "${SRC}")
            list(APPEND CUBE_SOURCES_C "${SRC}")
        endif()
    endforeach()

    # 应用配置
    target_compile_definitions(${TARGET_NAME} PUBLIC ${CUBE_DEFINES})
    target_include_directories(${TARGET_NAME} PUBLIC ${CUBE_INCLUDE_DIRS})
    target_sources(${TARGET_NAME} PRIVATE ${CUBE_SOURCES_C})

    # 编译与链接选项
    target_link_options(${TARGET_NAME} PUBLIC
            -mcpu=cortex-m4
            -Wl,-Map=${PROJECT_BINARY_DIR}/${TARGET_NAME}.map
            )

    # 链接脚本
    file(GLOB LINKER_SCRIPTS "${BOARD_DIR}/${LINKER}")
    if(LINKER_SCRIPTS)
        target_link_options(${TARGET_NAME} PUBLIC -T ${LINKER_SCRIPTS})
    else()
        message(FATAL_ERROR "Linker script not found: ${BOARD_DIR}/${LINKER}")
    endif()
ENDMACRO()


# =============================================
# 生成 HEX 和 BIN 文件
# =============================================
function(stm32_create_hex NAME)
    set(HEX_FILE ${PROJECT_BINARY_DIR}/${NAME}.hex)
    set(BIN_FILE ${PROJECT_BINARY_DIR}/${NAME}.bin)
    add_custom_command(TARGET ${NAME}.elf POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${NAME}.elf> ${HEX_FILE}
            COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${NAME}.elf> ${BIN_FILE}
            COMMENT " Building ${HEX_FILE} Building ${BIN_FILE}")
endfunction()