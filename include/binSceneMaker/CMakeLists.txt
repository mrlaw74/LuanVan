cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(BinPickingUI)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

FIND_PACKAGE(BOOST REQUIRED)
FIND_PACKAGE(PCL REQUIRED)
FIND_PACKAGE(OpenMP)
FIND_PACKAGE(CUDA)
find_package(Qt5 REQUIRED Widgets)

if (WIN32)
    find_package(Bullet CONFIG REQUIRED HINTS "bullet3\\share\\bullet3")
elseif(UNIX AND NOT APPLE)
    find_package(Bullet CONFIG REQUIRED HINTS "bullet3_linux\\share\\bullet3")
endif ()

# For bullet lib. You may need to modify here.
SET(__BULLET_INCLUDE_DIRS "${BULLET_INCLUDE_DIRS}; ${BULLET_INCLUDE_DIRS}/bullet/")
SET(__BULLET_LIBRARY_DIRS "${BULLET_INCLUDE_DIRS}/../lib")
SET(__BULLET_LIBRARIES "${BULLET_LIBRARIES};optimized;LinearMath;optimized;BulletDynamics;optimized;BulletCollision;optimized;BulletInverseDynamics;optimized;HACD")


IF(OPENMP_FOUND) 
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()

IF(CUDA_FOUND) 
	SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}; -Xcompiler \"/wd 4819\" -arch=sm_61")
	OPTION(USE_CUDA "Enable CUDA" TRUE)
ELSE()
	OPTION(USE_CUDA "Enable CUDA" FALSE)
ENDIF()

IF(USE_CUDA)
	ADD_DEFINITIONS(-DUSE_GPU)
	MESSAGE(STATUS "CUDA Enabled.")
ENDIF()

OPTION(USE_SIMD "Enable SIMD" FALSE)

IF(USE_SIMD)
	ADD_DEFINITIONS(-DUSE_SIMD)
	MESSAGE(STATUS "SIMD Enabled.")
ENDIF()

IF(USE_SIMD AND USE_CUDA)
	MESSAGE(FATAL_ERROR "You cannot set USE_GPU and USE_SIMD simultaneously.")
ENDIF()

ADD_DEFINITIONS(${PCL_DEFINITIONS})

ADD_LIBRARY(binSceneMaker binSceneMaker.cpp binSceneMaker.h convexDecomposition.cpp convexDecomposition.hpp)
TARGET_LINK_LIBRARIES(binSceneMaker ${PCL_LIBRARIES} ${__BULLET_LIBRARIES} Qt5::Widgets HPR)
TARGET_INCLUDE_DIRECTORIES(binSceneMaker PUBLIC ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${__BULLET_INCLUDE_DIRS} ${HPR_INCLUDE_DIR})

#IF(USE_CUDA)
#	CUDA_ADD_EXECUTABLE(visibleSceneMaker visibleSceneMaker.cpp setting.hpp visible_check.cu visible_check.cpp visible_check.h visible_check.cuh)
#ELSE()
#	ADD_EXECUTABLE(visibleSceneMaker visibleSceneMaker.cpp setting.hpp visible_check.cpp visible_check.h)
#ENDIF()
#TARGET_LINK_LIBRARIES(visibleSceneMaker ${PCL_LIBRARIES})
