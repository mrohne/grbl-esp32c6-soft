cmake_minimum_required(VERSION 3.10)
project(grblHAL-sim C)

add_compile_definitions(F_CPU=16000000)
add_compile_definitions(INJECT_BLOCK_BUFFER_SIZE=256)
#add_compile_definitions(SQUARING_ENABLED=1)

if (WIN32)
    add_compile_definitions(PLATFORM_WINDOWS)

    set(platform_SRC
        ${CMAKE_CURRENT_LIST_DIR}/src/platform_windows.h
        ${CMAKE_CURRENT_LIST_DIR}/src/platform_windows.c
    )

    set(platform_LIB
        ws2_32
    )
endif(WIN32)

if(UNIX)
    add_compile_definitions(PLATFORM_LINUX)

    set(platform_SRC
        ${CMAKE_CURRENT_LIST_DIR}/src/platform_linux.h
        ${CMAKE_CURRENT_LIST_DIR}/src/platform_linux.c
    )

    if(APPLE)
        set(platform_LIB
            pthread
        )
    else(APPLE)
        set(platform_LIB
            rt
            pthread
        )
    endif(APPLE)
endif(UNIX)

add_executable(grblHAL_sim 
    ${CMAKE_CURRENT_LIST_DIR}/src/main.c 
    ${CMAKE_CURRENT_LIST_DIR}/src/simulator.c
    ${CMAKE_CURRENT_LIST_DIR}/src/driver.c
    ${CMAKE_CURRENT_LIST_DIR}/src/eeprom.c
    ${CMAKE_CURRENT_LIST_DIR}/src/grbl_eeprom_extensions.c
    ${CMAKE_CURRENT_LIST_DIR}/src/mcu.c
    ${CMAKE_CURRENT_LIST_DIR}/src/serial.c
    ${CMAKE_CURRENT_LIST_DIR}/src/grbl_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/src/planner_inject_accessors.c
    ${platform_SRC}
)

target_link_libraries(grblHAL_sim PRIVATE 
    m
    grbl
    ${platform_LIB}
)

add_executable(grblHAL_validator 
    ${CMAKE_CURRENT_LIST_DIR}/src/eeprom.c
    ${CMAKE_CURRENT_LIST_DIR}/src/grbl_eeprom_extensions.c
    ${CMAKE_CURRENT_LIST_DIR}/src/validator.c 
    ${CMAKE_CURRENT_LIST_DIR}/src/validator_driver.c 
    ${platform_SRC}
)

target_link_libraries(grblHAL_validator PRIVATE 
    m
    grbl
    ${platform_LIB}
)
