cmake_minimum_required(VERSION 2.8.3)
project( kindyn )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g ")

SET(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rosbag
		std_msgs geometry_msgs
		message_generation
		common_utilities
		roboy_communication_middleware
		roboy_communication_simulation
		tf
		tf_conversions
        robot_state_publisher
		grid_map_ros
		eus_qpoases
		controller_interface
		hardware_interface
		controller_manager
		eigen_conversions
)

catkin_package( CATKIN_DEPENDS roboy_communication_middleware common_utilities
        INCLUDE_DIRS include)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

# libxml++
find_package(LibXML++ REQUIRED)
include_directories(${LibXML++_INCLUDE_DIRS})

# idyntree
find_package(iDynTree 0.11.1 REQUIRED)
include_directories(${iDynTree_INCLUDE_DIRS})

# qpOASES
find_package(qpOASES REQUIRED)
include_directories(${qpOASES_INCLUDE_DIRS})

include_directories(
		include/
        ${catkin_INCLUDE_DIRS}
)
#==============================================================================
# Application
#==============================================================================

add_library(kindyn SHARED
        src/EigenExtension.cpp
        src/robot.cpp
		include/kindyn/EigenExtension.hpp
        include/kindyn/robot.hpp
		src/test_robot.cpp include/kindyn/cable.hpp)
add_dependencies( kindyn roboy_communication_middleware_gencpp )
target_link_libraries(kindyn ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} CableLengthController)

add_library(CableLengthController
		src/controller/cableLengthController.cpp
		src/controller/cardsflow_state_interface.cpp
		src/controller/cardsflow_command_interface.cpp
		)
add_dependencies(CableLengthController roboy_communication_middleware_generate_messages_cpp )
target_link_libraries(CableLengthController ${catkin_LIBRARIES})

add_executable( test_idyntree src/test_idyntree.cpp)
target_link_libraries(test_idyntree ${iDynTree_LIBRARIES} ${catkin_LIBRARIES})

add_executable( test_robot src/test_robot.cpp)
target_link_libraries(test_robot ${iDynTree_LIBRARIES} ${catkin_LIBRARIES} kindyn)
#==============================================================================
# Installation
#==============================================================================
install(TARGETS
		kindyn
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
		test_idyntree
	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
		include/kindyn/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.hpp"
		)
