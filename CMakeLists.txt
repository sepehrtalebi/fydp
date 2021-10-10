cmake_minimum_required(VERSION 3.17)
project(FYDP)

set(CMAKE_CXX_STANDARD 14)

include_directories(./include)
include_directories(./include/differentiation)

add_executable(FYDP tests.cpp
        src/KF.cpp
        src/UKF.cpp
        src/EKF.cpp
        src/SensorMeasurements.cpp
        src/AircraftState.cpp
        src/ControlInputs.cpp
        src/AppliedLoads.cpp
        src/differentiation/Expression.cpp
        src/differentiation/Constant.cpp
        src/differentiation/Zero.cpp
        src/differentiation/One.cpp
        src/differentiation/Nan.cpp
        src/differentiation/Variable.cpp
        src/differentiation/BinaryOperator.cpp
        src/differentiation/UnaryOperator.cpp
        src/differentiation/Sum.cpp
        src/differentiation/Difference.cpp
        src/differentiation/Product.cpp
        src/differentiation/Quotient.cpp
        src/differentiation/Power.cpp
        src/differentiation/Sin.cpp
        src/differentiation/Cos.cpp
        src/differentiation/Log.cpp
        test/TestVector.cpp
        test/TestMatrix.cpp
        test/TestDifferentiation.cpp
        )
