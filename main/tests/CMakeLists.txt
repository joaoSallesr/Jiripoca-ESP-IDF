cmake_minimum_required(VERSION 3.16)
project(UnitTests C)

set(CMAKE_C_STANDARD 99)

if(NOT WIN32)
    link_libraries(m)   # fabsf/lroundf no Linux/macOS
endif()

include_directories(
    .
    unity
    stubs
    readers/accel_sensor_reader
    readers/baro_sensor_reader
    readers/csv_reader
    ..                  # Pasta 'main'
)

set(UNITY_SOURCES
    unity/unity.c
)

add_executable(test_baro
    ${UNITY_SOURCES}
    test_baro.c
    readers/baro_sensor_reader/baro_sensor_reader.c
    readers/csv_reader/csv_reader.c
)

add_executable(test_accel
    ${UNITY_SOURCES}
    test_accel.c
    readers/accel_sensor_reader/accel_sensor_reader.c
    readers/csv_reader/csv_reader.c
)

add_executable(test_gyro
    ${UNITY_SOURCES}
    test_gyro.c
    readers/gyro_sensor_reader/gyro_sensor_reader.c
    readers/csv_reader/csv_reader.c
)

# --- flight_control: o teste usa o arquivo REAL do firmware (main/flight_control.c) ---
# O arquivo faz #include "global.h", que o compilador procuraria primeiro em main/ (o global.h real).
# Para trocar pelos stubs sem manter uma copia manual, o CMake gera uma copia (flight_control_under_test.c) em build/ ao lado de
# um global.h "atalho" que aponta para stubs/global.h. A copia e regenerada a cada configuracao.
set(FC_GEN_DIR ${CMAKE_BINARY_DIR}/flight_control_under_test)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../flight_control.c ${FC_GEN_DIR}/flight_control_under_test.c COPYONLY)
file(WRITE ${FC_GEN_DIR}/global.h "#include \"stubs/global.h\"\n")

add_executable(test_flight_control
    ${UNITY_SOURCES}
    test_flight_control.c
    readers/flight_log_reader/flight_log_reader.c
)
target_include_directories(test_flight_control PRIVATE ${FC_GEN_DIR})
