project(BinPickingUI)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( OpenCV REQUIRED )
find_package(Qt5 REQUIRED Widgets)

add_library(robotCommunication SHARED STATIC robotCommunication.cpp robotCommunication.h)
target_link_libraries(robotCommunication ${PCL_LIBRARIES} ${OpenCV_LIBS} Qt5::Widgets NachiSocket dataType)
target_include_directories(robotCommunication PUBLIC ${PCL_INCLUDE_DIRS} ${NACHISOCKET_INCLUDE_DIR} ${DATATYPE_INCLUDE_DIR})

