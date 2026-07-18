cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(ov_core REQUIRED)
find_package(ov_init REQUIRED)

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (NOT ENABLE_ROS)
    message(FATAL_ERROR "Build with ROS1.cmake if you don't have ROS.")
endif ()
add_definitions(-DROS_AVAILABLE=2)

# Include our header files
include_directories(
        src
        src/thirdparty/px4
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
list(APPEND ament_libraries
        rclcpp
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
        ov_core
        ov_init
)

##################################################
# Make the shared library
##################################################

list(APPEND LIBRARY_SOURCES
        src/dummy.cpp
        src/sim/Simulator.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/ImuFilter.cpp
        src/core/OnlineAlignmentCandidateFilter.cpp
        src/core/OnlineAlignmentInitializer.cpp
        src/core/p4/factors/Factor_P4Epipolar.cpp
        src/core/p4/factors/Factor_P4FcTrajectory.cpp
        src/core/VioManager.cpp
        src/core/VioManagerHelper.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
        src/update/UpdaterZeroVelocity.cpp
        src/update/UpdaterGroundPlaneRange.cpp
        src/update/UpdaterGroundPlaneFeature.cpp
        src/update/UpdaterGroundPlaneFeatureV1.cpp
        src/update/VisualObservabilityPolicy.cpp
)
list(APPEND LIBRARY_SOURCES src/ros/ROS2Visualizer.cpp src/ros/ROSVisualizerHelper.cpp)
file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_msckf_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
ament_target_dependencies(ov_msckf_lib ${ament_libraries})
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src/ src/thirdparty/px4/)
install(TARGETS ov_msckf_lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
install(DIRECTORY src/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(ov_msckf_lib)

##################################################
# Make binary files!
##################################################

add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
ament_target_dependencies(run_subscribe_msckf ${ament_libraries})
target_link_libraries(run_subscribe_msckf ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_subscribe_msckf DESTINATION lib/${PROJECT_NAME})

add_executable(run_simulation src/run_simulation.cpp)
ament_target_dependencies(run_simulation ${ament_libraries})
target_link_libraries(run_simulation ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_simulation DESTINATION lib/${PROJECT_NAME})

add_executable(run_serial_msckf_ros_free
        src/run_serial_msckf_ros_free.cpp
        src/ros_free/VizDashboard.cpp
        src/ros_free/DiagPrinter.cpp
        src/ros_free/DiagLogger.cpp
)
ament_target_dependencies(run_serial_msckf_ros_free ${ament_libraries})
target_link_libraries(run_serial_msckf_ros_free ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_serial_msckf_ros_free DESTINATION lib/${PROJECT_NAME})

add_executable(test_sim_meas src/test_sim_meas.cpp)
ament_target_dependencies(test_sim_meas ${ament_libraries})
target_link_libraries(test_sim_meas ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_sim_meas DESTINATION lib/${PROJECT_NAME})

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
ament_target_dependencies(test_sim_repeat ${ament_libraries})
target_link_libraries(test_sim_repeat ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_sim_repeat DESTINATION lib/${PROJECT_NAME})

add_executable(test_joseph_update src/test_joseph_update.cpp)
ament_target_dependencies(test_joseph_update ${ament_libraries})
target_link_libraries(test_joseph_update ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_joseph_update DESTINATION lib/${PROJECT_NAME})

add_executable(test_fc_init_loader src/test_fc_init_loader.cpp)
ament_target_dependencies(test_fc_init_loader ${ament_libraries})
target_link_libraries(test_fc_init_loader ${thirdparty_libraries})
install(TARGETS test_fc_init_loader DESTINATION lib/${PROJECT_NAME})

add_executable(test_online_alignment_initializer src/test_online_alignment_initializer.cpp)
ament_target_dependencies(test_online_alignment_initializer ${ament_libraries})
target_link_libraries(test_online_alignment_initializer ov_msckf_lib ${thirdparty_libraries})
add_test(NAME test_online_alignment_initializer COMMAND test_online_alignment_initializer)
install(TARGETS test_online_alignment_initializer DESTINATION lib/${PROJECT_NAME})

add_executable(test_p4_formal_factors src/test_p4_formal_factors.cpp)
ament_target_dependencies(test_p4_formal_factors ${ament_libraries})
target_link_libraries(test_p4_formal_factors ov_msckf_lib ${thirdparty_libraries})
add_test(NAME test_p4_formal_factors COMMAND test_p4_formal_factors)
install(TARGETS test_p4_formal_factors DESTINATION lib/${PROJECT_NAME})

add_executable(test_online_vio_fc_gauge_aligner
  src/test_online_vio_fc_gauge_aligner.cpp)
ament_target_dependencies(test_online_vio_fc_gauge_aligner ${ament_libraries})
target_link_libraries(test_online_vio_fc_gauge_aligner ${thirdparty_libraries})
add_test(NAME test_online_vio_fc_gauge_aligner
  COMMAND test_online_vio_fc_gauge_aligner)
install(TARGETS test_online_vio_fc_gauge_aligner DESTINATION lib/${PROJECT_NAME})

add_executable(test_online_alignment_window_injection
  src/test_online_alignment_window_injection.cpp)
ament_target_dependencies(test_online_alignment_window_injection
  ${ament_libraries})
target_link_libraries(test_online_alignment_window_injection
  ov_msckf_lib ${thirdparty_libraries})
add_test(NAME test_online_alignment_window_injection
  COMMAND test_online_alignment_window_injection)
install(TARGETS test_online_alignment_window_injection
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_online_alignment_candidate_filter src/test_online_alignment_candidate_filter.cpp)
ament_target_dependencies(test_online_alignment_candidate_filter ${ament_libraries})
target_link_libraries(test_online_alignment_candidate_filter ov_msckf_lib ${thirdparty_libraries})
add_test(NAME test_online_alignment_candidate_filter COMMAND test_online_alignment_candidate_filter)
install(TARGETS test_online_alignment_candidate_filter DESTINATION lib/${PROJECT_NAME})

add_executable(test_camera_extrinsic_parser src/test_camera_extrinsic_parser.cpp)
ament_target_dependencies(test_camera_extrinsic_parser ${ament_libraries})
target_link_libraries(test_camera_extrinsic_parser ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_camera_extrinsic_parser DESTINATION lib/${PROJECT_NAME})

add_executable(test_adaptive_stride src/test_adaptive_stride.cpp)
ament_target_dependencies(test_adaptive_stride ${ament_libraries})
target_link_libraries(test_adaptive_stride ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_adaptive_stride DESTINATION lib/${PROJECT_NAME})

add_executable(test_dynamic_turn_roi src/test_dynamic_turn_roi.cpp)
ament_target_dependencies(test_dynamic_turn_roi ${ament_libraries})
target_link_libraries(test_dynamic_turn_roi ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_dynamic_turn_roi DESTINATION lib/${PROJECT_NAME})

add_executable(test_state_sim3_scale_reset src/test_state_sim3_scale_reset.cpp)
ament_target_dependencies(test_state_sim3_scale_reset ${ament_libraries})
target_link_libraries(test_state_sim3_scale_reset ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_state_sim3_scale_reset DESTINATION lib/${PROJECT_NAME})

add_executable(test_agl_scene_scale_ground_estimator
  src/test_agl_scene_scale_ground_estimator.cpp)
ament_target_dependencies(test_agl_scene_scale_ground_estimator ${ament_libraries})
target_link_libraries(test_agl_scene_scale_ground_estimator
  ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_agl_scene_scale_ground_estimator DESTINATION lib/${PROJECT_NAME})

add_executable(test_agl_scene_scale_controller
  src/test_agl_scene_scale_controller.cpp)
ament_target_dependencies(test_agl_scene_scale_controller ${ament_libraries})
target_link_libraries(test_agl_scene_scale_controller
  ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_agl_scene_scale_controller DESTINATION lib/${PROJECT_NAME})

add_executable(test_imu_filter src/test_imu_filter.cpp)
ament_target_dependencies(test_imu_filter ${ament_libraries})
target_link_libraries(test_imu_filter ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_imu_filter DESTINATION lib/${PROJECT_NAME})

add_executable(test_anchor_trust_policy src/test_anchor_trust_policy.cpp)
ament_target_dependencies(test_anchor_trust_policy ${ament_libraries})
target_link_libraries(test_anchor_trust_policy ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_anchor_trust_policy DESTINATION lib/${PROJECT_NAME})

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/test_constrained_yaw_nullspace.cpp")
    add_executable(test_constrained_yaw_nullspace src/test_constrained_yaw_nullspace.cpp)
    ament_target_dependencies(test_constrained_yaw_nullspace ${ament_libraries})
    target_link_libraries(test_constrained_yaw_nullspace ov_msckf_lib ${thirdparty_libraries})
    install(TARGETS test_constrained_yaw_nullspace DESTINATION lib/${PROJECT_NAME})
else ()
    message(WARNING "test_constrained_yaw_nullspace.cpp is absent; its target is not rebuilt")
endif ()

if (BUILD_TESTING)
    add_test(NAME test_fc_init_loader COMMAND test_fc_init_loader)
    add_test(NAME test_camera_extrinsic_parser COMMAND test_camera_extrinsic_parser)
    add_test(NAME test_adaptive_stride COMMAND test_adaptive_stride)
    add_test(NAME test_dynamic_turn_roi COMMAND test_dynamic_turn_roi)
    add_test(NAME test_state_sim3_scale_reset COMMAND test_state_sim3_scale_reset)
    add_test(NAME test_agl_scene_scale_ground_estimator
             COMMAND test_agl_scene_scale_ground_estimator)
    add_test(NAME test_agl_scene_scale_controller
             COMMAND test_agl_scene_scale_controller)
    add_test(NAME test_anchor_trust_policy COMMAND test_anchor_trust_policy)
    add_test(NAME test_imu_filter COMMAND test_imu_filter)
    add_test(NAME test_joseph_update COMMAND test_joseph_update)
endif ()

# Install launch and config directories
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY ../config/ DESTINATION share/${PROJECT_NAME}/config/)

# finally define this as the package
ament_package()
