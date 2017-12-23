project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11 -DHAVE_STDDEF_H -D_USE_MATH_DEFINES)


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
    src/behavior.cpp
    src/behavior.h
    src/behavior_planner.cpp
    src/behavior_planner.h
    src/constants.cpp
    src/constants.h
    src/conversion_helpers.cpp
    src/conversion_helpers.h
    src/ego.cpp
    src/ego.h
    src/json.hpp
    src/lane_info.cpp
    src/lane_info.h
    src/main.cpp
    src/path.cpp
    src/path.h
    src/path_planner.cpp
    src/path_planner.h
    src/point.h
    src/sensor_fusion.cpp
    src/sensor_fusion.h
    src/simulator_message_reader.h
    src/spline.h
    src/trajectory_planner.cpp
    src/trajectory_planner.h
    src/vehicle.h
    src/waypoint_map.cpp
    src/waypoint_map.h
)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
include_directories(src/Eigen-3.3/Eigen)
include_directories(../../uWebSockets/src)
include_directories(../../vcpkg/installed/x86-windows/include)
include_directories(../../glfw/include)
link_directories(../../uWebSockets/build_VS/Debug)
link_directories(../../vcpkg/installed/x86-windows/lib)
link_directories(../../glfw/build_VS/src/Debug)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")


add_executable(path_planning 
    ${sources}
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
target_link_libraries(path_planning 
    z 
    ssl 
    uv 
    uWS
)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(path_planning 
    z 
    ssl 
    uv 
    uWS
)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
target_link_libraries(path_planning 
	#ipopt 
	zlib 
	ssleay32 
	libuv 
	uWS
    glfw3 
    opengl32 	
)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
