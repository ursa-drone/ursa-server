-- Copyright 2017 LAOSAAC

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "base_link",
    published_frame = "base_link",
    odom_frame = "odom",
    provide_odom_frame = true,
    use_odometry = false,
    use_laser_scan = true,
    use_multi_echo_laser_scan = false,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,
    submap_publish_period_sec = 0.3,
    pose_publish_period_sec = 5e-3,
    trajectory_publish_period_sec = 30e-3,
}

-- Use 2D map builder
MAP_BUILDER.use_trajectory_builder_2d = true

-- Custom parameters
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.max_range = 5.6
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.miss_probability = 0.49

return options