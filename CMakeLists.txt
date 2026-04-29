cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(pico_cluster C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

function(configure_pico_target TARGET_NAME SOURCE_FILE)
  add_executable(${TARGET_NAME} ${SOURCE_FILE})

  pico_enable_stdio_usb(${TARGET_NAME} 1)
  pico_add_extra_outputs(${TARGET_NAME})

  target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

  target_link_libraries(${TARGET_NAME}
    pico_stdlib
    hardware_i2c
    pico_i2c_slave
    pico_cyw43_arch_none
  )
endfunction()

configure_pico_target(master master.c)

foreach(IDX RANGE 0 2)
  math(EXPR ADDR "10 + ${IDX}")
  configure_pico_target(slave_${IDX} slave.c)
  target_compile_definitions(slave_${IDX} PRIVATE SLAVE_ADDR=0x${ADDR})
endforeach()
