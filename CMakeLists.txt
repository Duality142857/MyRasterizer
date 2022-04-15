cmake_minimum_required(VERSION 3.0.0)
project(MyRasterizer VERSION 0.1.0)
find_package(OpenCV REQUIRED)
find_package(Qt5Widgets)
find_package(SDL2 REQUIRED)
include_directories(SDL2Test ${SDL2_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)

add_executable(MyRasterizer main.cpp)
# add_executable(sdlapp sdlapp.cpp)
# target_link_libraries(sdlapp ${SDL2_LIBRARIES})
target_link_libraries(MyRasterizer ${OpenCV_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${SDL2_LIBRARIES} -lpthread)