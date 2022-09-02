cmake_minimum_required(VERSION 2.8.3)
project( kindyn )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -fext-numeric-literals")

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

execute_process(COMMAND grep -o BCM /proc/cpuinfo OUTPUT_VARIABLE IAMRASPBERRYPI3)
if (IAMRASPBERRYPI3)
    message(STATUS "detected raspberry pi, defining EIGEN_DONT_ALIGN_STATICALLY!!!")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_DONT_ALIGN_STATICALLY")
endif ()

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		rosbag
		std_msgs geometry_msgs
		message_generation
		common_utilities
		roboy_middleware_msgs
		roboy_simulation_msgs
		tf
		tf_conversions
        robot_state_publisher
		controller_interface
		hardware_interface
		controller_manager
		eigen_conversions
		effort_controllers
		roboy_control_msgs
)

catkin_package( CATKIN_DEPENDS roboy_middleware_msgs common_utilities
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

# find openmp support
find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
		include/
        ${catkin_INCLUDE_DIRS}
)

#==============================================================================
# Application
#==============================================================================
# CARDSflow hardware interface
add_library(CardsflowHardwareInterface
		src/controller/cardsflow_state_interface.cpp
		src/controller/cardsflow_command_interface.cpp)
add_dependencies(CardsflowHardwareInterface roboy_middleware_msgs_generate_messages_cpp)
target_link_libraries(CardsflowHardwareInterface ${catkin_LIBRARIES})

# Cable Length Controller
add_library(CableLengthController src/controller/cableLengthController.cpp )
add_dependencies(CableLengthController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
target_link_libraries(CableLengthController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# Cable Length Velocity Controller
add_library(CableLengthVelocityController src/controller/cableLengthVelocityController.cpp )
add_dependencies(CableLengthVelocityController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
target_link_libraries(CableLengthVelocityController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# Torque Position Controller
add_library(TorquePositionController src/controller/torquePositionController.cpp )
add_dependencies(TorquePositionController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
target_link_libraries(TorquePositionController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# Force Position Controller
add_library(ForcePositionController src/controller/forcePositionController.cpp )
add_dependencies(ForcePositionController roboy_middleware_msgs_generate_messages_cpp roboy_control_msgs_generate_messages_cpp)
target_link_libraries(ForcePositionController ${catkin_LIBRARIES} CardsflowHardwareInterface)

# kindyn library
add_library(kindyn SHARED
        src/robot.cpp
		src/kinematics.cpp
		src/EigenExtension.cpp
		include/kindyn/EigenExtension.hpp
		include/kindyn/robot.hpp
		include/kindyn/kinematics.hpp
		include/kindyn/cable.hpp)
add_dependencies(kindyn roboy_simulation_msgs_generate_messages_cpp roboy_simulation_msgs_msgs_generate_messages_cpp)
target_link_libraries(kindyn ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} CardsflowHardwareInterface
		CableLengthController TorquePositionController ForcePositionController ${OpenMP_CXX_libraries})

# vrpuppet library
add_library(vrpuppet SHARED
		src/vrpuppet.cpp
		src/EigenExtension.cpp
		include/kindyn/EigenExtension.hpp
		include/kindyn/vrpuppet.hpp
		include/kindyn/cable.hpp)
add_dependencies(vrpuppet roboy_simulation_msgs_generate_messages_cpp roboy_simulation_msgs_msgs_generate_messages_cpp)
target_link_libraries(vrpuppet ${catkin_LIBRARIES} ${iDynTree_LIBRARIES} ${qpOASES_LIBRARIES} CardsflowHardwareInterface
		CableLengthController TorquePositionController ForcePositionController ${OpenMP_CXX_libraries})

# testing idyntree
add_executable( test_idyntree src/utilities/test_idyntree.cpp)
target_link_libraries(test_idyntree ${iDynTree_LIBRARIES} ${catkin_LIBRARIES})

# upper_body
add_executable( upper_body src/robots/upper_body.cpp)
target_link_libraries(upper_body ${iDynTree_LIBRARIES} ${catkin_LIBRARIES} vrpuppet kindyn)

# musc_shoulder
add_executable( musc_shoulder src/robots/musc_shoulder.cpp)
target_link_libraries(musc_shoulder ${iDynTree_LIBRARIES} ${catkin_LIBRARIES} vrpuppet kindyn)

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
		kindyn
		CardsflowHardwareInterface
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
	    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
        package.xml
		CableLengthController_plugin.xml
		TorquePositionController_plugin.xml
		ForcePositionController_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
		include/kindyn/
		include/kindyn/controller
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.hpp"
		)
