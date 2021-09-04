project(BinPickingUI)

set(PPF_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "")

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package( OpenCV REQUIRED )
find_package(Qt5 REQUIRED Widgets)

add_library(PPF SHARED STATIC PPF.cpp PPF.h MyPPFRegistration.cpp MyPPFRegistration.hpp)
target_link_libraries(PPF pclFunction meshSampling HPR ${OpenCV_LIBS} Qt5::Widgets NachiSocket)
target_include_directories(PPF PUBLIC ${PCL_INCLUDE_DIRS} ${PCLFUNCTION_INCLUDE_DIR} ${MESHSAMPLING_INCLUDE_DIR} ${HPR_INCLUDE_DIR} ${NACHISOCKET_INCLUDE_DIR})




