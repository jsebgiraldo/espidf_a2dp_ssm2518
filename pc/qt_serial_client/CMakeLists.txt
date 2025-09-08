cmake_minimum_required(VERSION 3.16)
project(qt_serial_client LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 COMPONENTS Core SerialPort REQUIRED)

add_executable(qt_serial_client
    main.cpp
)

target_link_libraries(qt_serial_client PRIVATE Qt6::Core Qt6::SerialPort)

install(TARGETS qt_serial_client RUNTIME DESTINATION bin)
