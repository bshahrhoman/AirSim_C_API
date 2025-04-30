cmake_minimum_required(VERSION 3.5.0)
project(HelloDrone)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

SetupConsoleBuild()

## Specify additional locations of header files
include_directories(
  ${AIRSIM_ROOT}/HelloDrone
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
)

AddExecutableSource()

find_package(Eigen3 3.3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
			
CommonTargetLink()
target_link_libraries(${PROJECT_NAME} AirLib)
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})