cmake_minimum_required(VERSION 3.16)
project(UnitTests C)

set(CMAKE_C_STANDARD 99)

include_directories(
    .
    unity
    stubs
    readers/accel_sensor_reader
    readers/baro_sensor_reader
    readers/csv_reader
    readers/flight_log_reader
    ..                  # Pasta 'main'
)

set(UNITY_SOURCES
    unity/unity.c
)

add_executable(test_baro
    ${UNITY_SOURCES}
    test_baro_altitude.c
    readers/baro_sensor_reader/baro_sensor_reader.c
    readers/csv_reader/csv_reader.c
)

add_executable(test_accel
    ${UNITY_SOURCES}
    test_accel.c
    readers/accel_sensor_reader/accel_sensor_reader.c
    readers/csv_reader/csv_reader.c
)

add_executable(test_boost
    ${UNITY_SOURCES}
    test_boost_detection.c
    readers/flight_log_reader/flight_log_reader.c
)