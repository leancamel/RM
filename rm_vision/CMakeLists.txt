cmake_minimum_required(VERSION 3.0)
project(uart_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


set("${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/src")
include_directories(${COMMON_INCLUDE_DIR})

# find_package(PkgConfig)
# pkg_search_module(GLIB REQUIRED glib-2.0)
# include_directories(${GLIB_INCLUDE_DIRS})


# UartTest （通信查验）
set(UART_PROJECT_NAME "uart_send")
set(UART_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/uart_send.cpp)
add_executable(${UART_PROJECT_NAME} ${UART_PROJECT_SOURCES})
target_include_directories(${UART_PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${UART_PROJECT_NAME} PRIVATE serial)

set(UART_PROJECT_NAME "uart_recv")
set(UART_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/src/uart_recv.cpp)
add_executable(${UART_PROJECT_NAME} ${UART_PROJECT_SOURCES})
target_include_directories(${UART_PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${UART_PROJECT_NAME} PRIVATE serial)
