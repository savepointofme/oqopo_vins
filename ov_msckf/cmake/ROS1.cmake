cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge ov_core ov_init)

# Describe ROS project
if (catkin_FOUND AND ENABLE_ROS)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge ov_core ov_init
            INCLUDE_DIRS src/
            LIBRARIES ov_msckf_lib
    )
else ()
    add_definitions(-DROS_AVAILABLE=0)
    message(WARNING "BUILDING WITHOUT ROS!")
    include(GNUInstallDirs)
    set(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    set(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
    set(CATKIN_GLOBAL_INCLUDE_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/open_vins/")
endif ()


# Include our header files
include_directories(
        src
        src/thirdparty/px4
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
)

# Optional ONNX Runtime for neural feature extractors (XFeat, SuperPoint …)
# Pass -DONNXRUNTIME_DIR=/path/to/onnxruntime-linux-x64-<ver> to cmake to enable.
if(DEFINED ONNXRUNTIME_DIR)
    message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")
    include_directories(${ONNXRUNTIME_DIR}/include)
    list(APPEND thirdparty_libraries ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)
    add_definitions(-DUSE_ONNXRUNTIME=1)
else()
    message(STATUS "ONNX Runtime: not configured (XFeat disabled; pass -DONNXRUNTIME_DIR=...)")
endif()

# If we are not building with ROS then we need to manually link to its headers
# This isn't that elegant of a way, but this at least allows for building without ROS
# If we had a root cmake we could do this: https://stackoverflow.com/a/11217008/7718197
# But since we don't we need to basically build all the cpp / h files explicitly :(
if (NOT catkin_FOUND OR NOT ENABLE_ROS)

    message(STATUS "MANUALLY LINKING TO OV_CORE LIBRARY....")
    file(GLOB_RECURSE OVCORE_LIBRARY_SOURCES "${CMAKE_SOURCE_DIR}/../ov_core/src/*.cpp")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_profile\\.cpp$")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_webcam\\.cpp$")
    list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_tracking\\.cpp$")
    list(APPEND LIBRARY_SOURCES ${OVCORE_LIBRARY_SOURCES})
    include_directories(${CMAKE_SOURCE_DIR}/../ov_core/src/)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/../ov_core/src/
            DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

    message(STATUS "MANUALLY LINKING TO OV_INIT LIBRARY....")
    file(GLOB_RECURSE OVINIT_LIBRARY_SOURCES "${CMAKE_SOURCE_DIR}/../ov_init/src/*.cpp")
    list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*test_dynamic_init\\.cpp$")
    list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*test_dynamic_mle\\.cpp$")
    list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*test_simulation\\.cpp$")
    list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*Simulator\\.cpp$")
    list(APPEND LIBRARY_SOURCES ${OVINIT_LIBRARY_SOURCES})
    include_directories(${CMAKE_SOURCE_DIR}/../ov_init/src/)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/../ov_init/src/
            DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
            FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

endif ()

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
if (catkin_FOUND AND ENABLE_ROS)
    list(APPEND LIBRARY_SOURCES src/ros/ROS1Visualizer.cpp src/ros/ROSVisualizerHelper.cpp)
endif ()
file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_msckf_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src/ src/thirdparty/px4/)
install(TARGETS ov_msckf_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


##################################################
# Make binary files!
##################################################

if (catkin_FOUND AND ENABLE_ROS)

    add_executable(ros1_serial_msckf src/ros1_serial_msckf.cpp)
    target_link_libraries(ros1_serial_msckf ov_msckf_lib ${thirdparty_libraries})
    install(TARGETS ros1_serial_msckf
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
    target_link_libraries(run_subscribe_msckf ov_msckf_lib ${thirdparty_libraries})
    install(TARGETS run_subscribe_msckf
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

endif ()

add_executable(run_simulation src/run_simulation.cpp)
target_link_libraries(run_simulation ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_simulation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ROS-free offline runner + OpenCV dashboard. Always built; self-contained
# w.r.t. ROS so it also compiles under ENABLE_ROS=OFF.
add_executable(run_serial_msckf_ros_free
        src/run_serial_msckf_ros_free.cpp
        src/ros_free/VizDashboard.cpp
        src/ros_free/DiagPrinter.cpp
        src/ros_free/DiagLogger.cpp
)
target_link_libraries(run_serial_msckf_ros_free ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_serial_msckf_ros_free
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_sim_meas src/test_sim_meas.cpp)
target_link_libraries(test_sim_meas ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_sim_meas
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
target_link_libraries(test_sim_repeat ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_sim_repeat
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Gate 1: Joseph-form covariance update unit tests (no ROS, no GPS, no VIO pipeline)
add_executable(test_joseph_update src/test_joseph_update.cpp)
target_link_libraries(test_joseph_update ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_joseph_update
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# FC init CSV time-selection tests (pure header logic).
add_executable(test_fc_init_loader src/test_fc_init_loader.cpp)
target_link_libraries(test_fc_init_loader ${thirdparty_libraries})
install(TARGETS test_fc_init_loader
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_online_alignment_initializer src/test_online_alignment_initializer.cpp)
target_link_libraries(test_online_alignment_initializer ov_msckf_lib ${thirdparty_libraries})
add_test(NAME test_online_alignment_initializer COMMAND test_online_alignment_initializer)

add_executable(test_online_alignment_candidate_filter src/test_online_alignment_candidate_filter.cpp)
target_link_libraries(test_online_alignment_candidate_filter ov_msckf_lib ${thirdparty_libraries})
add_test(NAME test_online_alignment_candidate_filter COMMAND test_online_alignment_candidate_filter)

# Camera/IMU external-YAML extrinsic direction regression tests.
add_executable(test_camera_extrinsic_parser src/test_camera_extrinsic_parser.cpp)
target_link_libraries(test_camera_extrinsic_parser ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_camera_extrinsic_parser
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Gate 2: Constrained-yaw-nullspace rank-1 projection unit tests (pure Eigen, no pipeline).
# Some experiment snapshots contain the target declaration but not its source;
# keep those snapshots configurable while emitting an explicit warning.
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/test_constrained_yaw_nullspace.cpp")
    add_executable(test_constrained_yaw_nullspace src/test_constrained_yaw_nullspace.cpp)
    target_link_libraries(test_constrained_yaw_nullspace ov_msckf_lib ${thirdparty_libraries})
    install(TARGETS test_constrained_yaw_nullspace
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
else ()
    message(WARNING "test_constrained_yaw_nullspace.cpp is absent; its target is not rebuilt")
endif ()

# Pure causal-policy tests for --adaptive-stride-shadow/--adaptive-stride.
add_executable(test_adaptive_stride src/test_adaptive_stride.cpp)
target_link_libraries(test_adaptive_stride ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_adaptive_stride
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Header-only P5 state-machine replay used to recompute shadow timelines from
# existing estimator telemetry without rerunning fixed-stride estimators.
add_executable(p5_shadow_state_replay src/p5_shadow_state_replay.cpp)
target_link_libraries(p5_shadow_state_replay ${thirdparty_libraries})
install(TARGETS p5_shadow_state_replay
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Pure external-anchor source-quality and trust-policy tests.
add_executable(test_anchor_trust_policy src/test_anchor_trust_policy.cpp)
target_link_libraries(test_anchor_trust_policy ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_anchor_trust_policy
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Pure causal IMU filter and timestamp-policy tests.
add_executable(test_imu_filter src/test_imu_filter.cpp)
target_link_libraries(test_imu_filter ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_imu_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (BUILD_TESTING)
    add_test(NAME test_fc_init_loader COMMAND test_fc_init_loader)
    add_test(NAME test_camera_extrinsic_parser COMMAND test_camera_extrinsic_parser)
    add_test(NAME test_adaptive_stride COMMAND test_adaptive_stride)
    add_test(NAME test_anchor_trust_policy COMMAND test_anchor_trust_policy)
    add_test(NAME test_imu_filter COMMAND test_imu_filter)
    add_test(NAME test_joseph_update COMMAND test_joseph_update)
endif ()
