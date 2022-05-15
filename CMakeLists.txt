cmake_minimum_required(VERSION 3.0)
project(VIG_FactorGraph)

set( CMAKE_BUILD_TYPE "Debug" )
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native -std=c++14")
#Eigen,OpenCV
find_package(Eigen3 REQUIRED)
find_package( OpenCV 3.0.0 REQUIRED )
find_package(Pangolin REQUIRED)

#GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs
set(GTSAM_UNSTABLE_LIBRARIES gtsam_unstable)

######  include and library directories.
include_directories(
	${EIGEN3_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/include/gtsam_derived
	${PROJECT_SOURCE_DIR}/include/test
	)

add_library(${PROJECT_NAME} SHARED
    src/gtsam_derived/jPreintegratedIMUs.cpp
    src/gtsam_derived/jGPSFactorYawLA.cpp
    src/gtsam_derived/jBatchFLSmoother.cpp

    src/VIGCommon.cpp
	src/Configuration.cpp
	src/Tracker.cpp
	src/Mapper.cpp
	src/Optimizer.cpp
	src/VIGInitializer.cpp
	src/Viewer.cpp
	# src/test/TestVSlam.cpp
	# src/test/TestVIO.cpp
	# src/test/TestSynVIO.cpp
	# src/test/TestSimVIO.cpp

	)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${Pangolin_LIBRARIES}
	# "gtsam"
	# "gtsam_unstable"
	${GTSAM_LIBRARIES}
	${GTSAM_UNSTABLE_LIBRARIES}
	)

#############################
add_executable(VIG_Syn Examples/VIG_Syn.cpp)
target_link_libraries(VIG_Syn ${PROJECT_NAME})

add_executable(VIG_Sim Examples/VIG_Sim.cpp)
target_link_libraries(VIG_Sim ${PROJECT_NAME})

# add_executable(IG4Init Examples/IG4Init.cpp)
# target_link_libraries(IG4Init ${PROJECT_NAME})

add_executable(IG Examples/IG.cpp)
target_link_libraries(IG ${PROJECT_NAME})