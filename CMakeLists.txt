cmake_minimum_required(VERSION 3.20)
project(FYDP)

set(CMAKE_CXX_STANDARD 14)

include_directories(./include)

add_executable(FYDP tests.cpp
        src/KF.cpp
        src/EKF.cpp
        src/SensorMeasurements.cpp
        src/AircraftState.cpp
        test/TestVector.cpp
        test/TestMatrix.cpp
        )
