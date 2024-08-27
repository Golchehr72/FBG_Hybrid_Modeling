cmake_minimum_required (VERSION 3.10)
project (fbg_hybrid_modeling)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(bigssUtil REQUIRED CONFIG
	NAMES BIGSSUtil
  PATHS "~/bigss/util/build")
include_directories(${BIGSS_UTIL_INCLUDE_DIRS})
link_directories(${BIGSS_UTIL_LIBS})

set (REQUIRED_CISST_LIBRARIES
     cisstMultiTask
     cisstCommon
     cisstOSAbstraction
     )

find_package (cisst 1.1.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})
  #include_directories(${CISST_INCLUDE_DIR})

  find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  cisst_ros_bridge
  cisst
  geometry_msgs
  sensor_msgs
  std_msgs
  )
include_directories(${catkin_INCLUDE_DIRS})


# catkin/ROS paths
cisst_is_catkin_build (fbg_hybrid_modeling_IS_CATKIN_BUILT)
if (fbg_hybrid_modeling_IS_CATKIN_BUILT)
  set (fbg_hybrid_modeling_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/share/fbg_hybrid_modeling/cmake")
  set (LIBRARY_OUTPUT_PATH "${fbg_hybrid_modeling_BINARY_DIR}/lib")
else ()
  set (fbg_hybrid_modeling_CONFIG_FILE_DIR "${fbg_hybrid_modeling_BINARY_DIR}")
endif ()

# create/configure file for find_package 
set (fbg_hybrid_modeling_INCLUDE_DIR
     "${fbg_hybrid_modeling_SOURCE_DIR}/include"
     "${fbg_hybrid_modeling_BINARY_DIR}/include")
set (fbg_hybrid_modeling_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
set (fbg_hybrid_modeling_LIBRARIES fbg_hybrid_modeling)

include_directories (${fbg_hybrid_modeling_INCLUDE_DIR})
set (fbg_hybrid_modeling_HEADER_DIR "${fbg_hybrid_modeling_SOURCE_DIR}/include/fbg_hybrid_modeling")



catkin_package(INCLUDE_DIRS include)



  set (HEADER_FILES 
       ${fbg_hybrid_modeling_HEADER_DIR}/mtsTaskFBGInterrogator.h
      #${fbg_hybrid_modeling_HEADER_DIR}/mtsTaskTest.h
       )

  set (SOURCE_FILES
       src/mtsTaskFBGInterrogator.cpp
       #src/testmtsTaskFBGInterrogator.cpp
       #src/testmtsTaskMaxonMotor.cpp
       )

  add_library (fbg_hybrid_modeling
               ${HEADER_FILES}
               ${SOURCE_FILES}
               )

set_property (TARGET fbg_hybrid_modeling PROPERTY FOLDER "fbg_hybrid_modeling")
set_target_properties(fbg_hybrid_modeling PROPERTIES LINKER_LANGUAGE CXX)
cisst_target_link_libraries (fbg_hybrid_modeling ${REQUIRED_CISST_LIBRARIES})
target_link_libraries(fbg_hybrid_modeling Qt5::Widgets ${BIGSS_UTIL_LIB})


add_executable(testFBGInterrogator src/testmtsTaskFBGInterrogator.cpp)
target_link_libraries(testFBGInterrogator fbg_hybrid_modeling ${catkin_LIBRARIES})

add_executable(motorDriverControl src/testmtsTaskMaxonMotor.cpp)
target_link_libraries(motorDriverControl fbg_hybrid_modeling ${catkin_LIBRARIES})

#add_executable(startMotorDriver code/main_MotorDriver.cpp)
#target_link_libraries(startMotorDriver fbg_hybrid_modeling ${catkin_LIBRARIES})


# Install targets for headers and library
install (DIRECTORY
"${fbg_hybrid_modeling_SOURCE_DIR}/include/"
"${fbg_hybrid_modeling_BINARY_DIR}/include/"
DESTINATION include
PATTERN .svn EXCLUDE)

install (TARGETS fbg_hybrid_modeling
RUNTIME DESTINATION bin
LIBRARY DESTINATION lib
ARCHIVE DESTINATION lib)

install (FILES "${fbg_hybrid_modeling_CMAKE_CONFIG_FILE}"
           DESTINATION "share/fbg_hybrid_modeling")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
