cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ✅ [新增] C++ 标准设置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MechanicalArm_Code_V4)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# ✅ [修改] 顶层启用 CXX，子目录自动继承，无需改子目录文件
enable_language(C CXX ASM)

# ✅ [新增] 嵌入式 C++ 必要编译选项，放在顶层永久生效
add_compile_options(
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Cpp/Src/CppMain.cpp
        Cpp/Inc/CppMain.h
        Cpp/Inc/MotorControl.h
        Cpp/Src/MotorControl.cpp
        Cpp/Src/Forward_kinematics.cpp
        Cpp/Inc/Forward_kinematics.h
        Cpp/Src/NewBee.cpp
        Cpp/Inc/NewBee.h
        Cpp/Inc/RobotTypes.h
        Cpp/Src/RobotState.cpp
        Cpp/Inc/RobotState.h
        Cpp/Inc/Inverse_kinematics.h
        Cpp/Src/Inverse_kinematics.cpp
        Cpp/Inc/Trajectory_Planning.h
        Cpp/Src/Trajectory_Planning.cpp
        Task/Robot_Task.h
        Task/RobotInit_Task.cpp
        Task/Uart_Task.cpp
        Task/PTP_Task.cpp
        Task/C_Hal_Bridge.h
        Task/PAD_Task.cpp)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        # Core/Src/MyClass.cpp
        # Core/Src/app.cpp
        Cpp/Src/CppMain.cpp
        Cpp/Src/MotorControl.cpp
        Cpp/Src/Forward_Kinematics.cpp
        Cpp/Src/Inverse_Kinematics.cpp
        Cpp/Src/RobotState.cpp
        Cpp/Src/NewBee.cpp
        Task/RobotInit_Task.cpp
        Task/Uart_Task.cpp
        Task/PTP_Task.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        Cpp/Inc
        Task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

#添加浮点数支持
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float    # ✅ 启用浮点 printf 支持
)


# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)
