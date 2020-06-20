#cmake_minimum_required(VERSION 3.10.2)

add_library( nativeSerial
        SHARED
        nativeSerial.cpp )

find_library( log-lib  log )

target_link_libraries(
        nativeSerial
        ${log-lib}
        )
