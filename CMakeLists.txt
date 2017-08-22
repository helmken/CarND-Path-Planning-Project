project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

#add_definitions(-std=c++11 -O3 -DHAVE_STDDEF_H)
add_definitions(-std=c++11 -g -DHAVE_STDDEF_H -D_USE_MATH_DEFINES)


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources
    src/behavior_planner.h
    src/behavior_planner.cpp
    src/conversion_helpers.cpp
    src/conversion_helpers.h
    src/dynamic_object.h
    src/ego.h
    src/json.hpp
    src/main.cpp
    src/path.h
    src/path_generator.cpp
    src/path_generator.h
    src/simulator_message_reader.h
    src/spline.h
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
include_directories(../../uWebSockets/src)
include_directories(../../vcpkg/installed/x86-windows/include)
link_directories(../../uWebSockets/build_VS/Debug)
link_directories(../../vcpkg/installed/x86-windows/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")


add_executable(path_planning 
    ${sources}
)

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
)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
