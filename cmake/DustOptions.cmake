# Centralized compile/link options for the Dust project.
# Kept as a separate file so subdirectories can stay minimal.

function(dust_configure_options options_target)
    # Language standards (propagate to consumers)
    target_compile_features(${options_target} INTERFACE c_std_11 cxx_std_17)

    target_compile_definitions(${options_target} INTERFACE
        DEBUG
        USE_HAL_DRIVER
        STM32F446xx
    )

    target_compile_options(${options_target} INTERFACE
        -mcpu=cortex-m4 -mthumb -mthumb-interwork
        -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -Wall
        -ffunction-sections -fdata-sections -fno-common -fmessage-length=0
        $<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>
        $<$<COMPILE_LANGUAGE:ASM>:-MMD -MP>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti -fno-exceptions -fno-threadsafe-statics>
        $<$<CONFIG:Release>:-Ofast>
        $<$<CONFIG:RelWithDebInfo>:-Ofast -g>
        $<$<CONFIG:MinSizeRel>:-Os>
        $<$<OR:$<CONFIG:Debug>,$<STREQUAL:${CMAKE_BUILD_TYPE},>>:-Og -g>
    )

    target_link_options(${options_target} INTERFACE
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        $<$<C_COMPILER_ID:GNU>:--specs=nano.specs>
    )

    target_link_libraries(${options_target} INTERFACE
        m
    )

    target_include_directories(${options_target} INTERFACE
        ${CMAKE_SOURCE_DIR}/Board/xtu-board-v2
        ${CMAKE_SOURCE_DIR}/Board/xtu-board-v2/Inc
        ${CMAKE_SOURCE_DIR}/ThirdParty/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/ThirdParty/STM32F4xx_HAL_Driver/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/include
        ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/CMSIS_RTOS_V2
        ${CMAKE_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F
        ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_SOURCE_DIR}/ThirdParty/CMSIS/Include

        ${CMAKE_SOURCE_DIR}/App
        ${CMAKE_SOURCE_DIR}/Topic
        ${CMAKE_SOURCE_DIR}/comm_topics
        ${CMAKE_SOURCE_DIR}/Device
        ${CMAKE_SOURCE_DIR}/Driver
        ${CMAKE_SOURCE_DIR}/Platform
    )
endfunction()
