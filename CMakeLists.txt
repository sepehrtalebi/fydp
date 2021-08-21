cmake_minimum_required(VERSION 3.20)
project(FYDP)

set(CMAKE_CXX_STANDARD 14)

include_directories(./src)

add_executable(FYDP tests.cpp
        # main_ekf.cpp
        src/Vector3.cpp
        src/Quaternion.cpp
        src/EKF.cpp
        test/TestVector.cpp
        src/SensorMeasurements.cpp
        )
