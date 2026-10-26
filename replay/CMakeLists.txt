
cmake_minimum_required(VERSION 3.21)

project(strike_replay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV CONFIG REQUIRED)

add_executable(strike_replay
    main.cpp
    FrameReader.cpp
)

target_link_libraries(strike_replay
    PRIVATE ${OpenCV_LIBS}
)

target_include_directories(strike_replay
    PRIVATE ${OpenCV_INCLUDE_DIRS}
)
