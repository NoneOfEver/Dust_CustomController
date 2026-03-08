# Board (xtu-board) CMake configuration.

file(GLOB_RECURSE BOARD_SOURCES CONFIGURE_DEPENDS
    "Src/*.c" "Src/*.cpp" "Src/*.s" "Src/*.S"
    "Port/*.c" "Port/*.cpp" "Port/*.s" "Port/*.S"
    "startup_stm32f446xx.s"
)

list(LENGTH BOARD_SOURCES BOARD_SOURCES_COUNT)
if(BOARD_SOURCES_COUNT EQUAL 0)
    add_library(dust_board_xtu_board_v2 INTERFACE)
    target_link_libraries(dust_board_xtu_board_v2 INTERFACE dust_options)
else()
    add_library(dust_board_xtu_board_v2 OBJECT)
    target_sources(dust_board_xtu_board_v2 PRIVATE ${BOARD_SOURCES})
    target_link_libraries(dust_board_xtu_board_v2 PRIVATE dust_options)
endif()
