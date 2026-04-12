aux_source_directory( . AUX_SRC_MIXED_DATA_LOADER )

# Add all relative components of slam utility.
set( SLAM_UTILITY_PATH ${PROJECT_SOURCE_DIR}/../Slam_Utility )
if( NOT TARGET lib_slam_utility_basic_type )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/basic_type ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_basic_type )
endif()
if( NOT TARGET lib_slam_utility_math )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/math ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_math )
endif()
if( NOT TARGET lib_slam_utility_operate )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/operate ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_operate )
endif()
if( NOT TARGET lib_slam_utility_log )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/log ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_log )
endif()
if( NOT TARGET lib_slam_utility_memory )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/memory ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_memory )
endif()
if( NOT TARGET lib_slam_utility_tick_tock )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/tick_tock ${PROJECT_SOURCE_DIR}/build/lib_slam_utility_tick_tock )
endif()

# Add all relative components of slam utility data type.
if( NOT TARGET lib_image )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_type/image ${PROJECT_SOURCE_DIR}/build/lib_image )
endif()

# Add all relative components of slam utility data struct.
if( NOT TARGET lib_object_pool )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_struct/object_pool ${PROJECT_SOURCE_DIR}/build/lib_object_pool )
endif()
if( NOT TARGET lib_circular_buffer )
    add_subdirectory( ${SLAM_UTILITY_PATH}/src/data_struct/circular_buffer ${PROJECT_SOURCE_DIR}/build/lib_circular_buffer )
endif()

# Add imu and camera model.
set( SENSOR_MODEL ${PROJECT_SOURCE_DIR}/../Sensor_Model )
if( NOT TARGET lib_imu_model )
	add_subdirectory( ${SENSOR_MODEL}/src/imu ${PROJECT_SOURCE_DIR}/build/lib_imu_model )
endif()
if( NOT TARGET lib_camera_model )
    add_subdirectory( ${SENSOR_MODEL}/src/camera ${PROJECT_SOURCE_DIR}/build/lib_camera_model )
endif()
if( NOT TARGET lib_lidar_model )
    add_subdirectory( ${SENSOR_MODEL}/src/lidar ${PROJECT_SOURCE_DIR}/build/lib_lidar_model )
endif()

# Create library.
add_library( lib_mixed_data_loader ${AUX_SRC_MIXED_DATA_LOADER} )
target_include_directories( lib_mixed_data_loader PUBLIC . )
target_link_libraries( lib_mixed_data_loader
    lib_slam_utility_basic_type
    lib_slam_utility_math
    lib_slam_utility_operate
    lib_slam_utility_log
    lib_slam_utility_memory
    lib_slam_utility_tick_tock

    lib_image
    lib_object_pool
    lib_circular_buffer

    lib_imu_model
    lib_camera_model
    lib_lidar_model
)
