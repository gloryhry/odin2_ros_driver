#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <odin2_ros_driver/OdomPacket.h>
#include <odin2_ros_driver/Telemetry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "core.hpp"
namespace odin_driver {
sensor_msgs::PointCloud2 rawCloud(const sdk::OdinPointCloudPacket&, const std_msgs::Header&);
sensor_msgs::PointCloud2 slamCloud(const sdk::OdinSlamPacket&, const std_msgs::Header&);
sensor_msgs::Imu imuMessage(const sdk::OdinImuSample&, const std_msgs::Header&);
nav_msgs::Odometry odomMessage(const sdk::OdinOdomData&, const std_msgs::Header&,
                               const DeviceSettings&);
odin2_ros_driver::OdomPacket odomPacket(const sdk::OdinOdomPacket&, const std_msgs::Header&,
                                        uint8_t type);
geometry_msgs::TransformStamped tfMessage(const sdk::OdinTfPacket&, const std_msgs::Header&,
                                          const DeviceSettings&);
odin2_ros_driver::Telemetry telemetryMessage(const sdk::OdinTelemetryPacket&,
                                             const std_msgs::Header&);
bool cameraKeyframe(const sdk::OdinCameraPacket&);
}  // namespace odin_driver
