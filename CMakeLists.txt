cmake_minimum_required( VERSION 3.10 )

project( slam_dataloader )

set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -Wall -pthread" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_STANDARD 20 )

# Add slam dataloader.
set( SLAM_DATALOADER_PATH ${PROJECT_SOURCE_DIR}/../Slam_Dataloader )
if( NOT TARGET lib_mixed_data_loader )
    add_subdirectory( ${SLAM_DATALOADER_PATH}/src/mixed_data_loader ${PROJECT_SOURCE_DIR}/build/lib_mixed_data_loader )
endif()

# Create executable target to test slam dataloader.
add_executable( test_mixed_data_loader
    test/test_mixed_data_loader.cpp
)
target_link_libraries( test_mixed_data_loader
    lib_mixed_data_loader
)
