#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(BinPickingUI)

find_package( PCL REQUIRED )
 
if (PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
endif()

add_library(realsense SHARED STATIC realsense.cpp realsense.h)
set_property(TARGET realsense PROPERTY CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
target_link_libraries(realsense ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(realsense PUBLIC ${PCL_INCLUDE_DIRS} ${DATATYPE_INCLUDE_DIR})
