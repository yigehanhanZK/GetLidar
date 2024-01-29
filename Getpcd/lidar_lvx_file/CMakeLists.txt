cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_lvx_sample)
add_executable(${DEMO_NAME} main.cpp lvx_file.cpp)

find_package(OpenCV REQUIRED)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        ${OpenCV_LIBS}
        )
