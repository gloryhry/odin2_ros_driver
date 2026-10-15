#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "odin2_ros_driver/conversion.hpp"

// Match the x-nav ODIN consumer's registered field types and padded PCL layout.
struct EIGEN_ALIGN16 OdinConsumerPoint {
  PCL_ADD_POINT4D;
  std::uint8_t intensity;
  std::uint8_t confidence;
  float offset_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(
    OdinConsumerPoint, (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
                           std::uint8_t, confidence, confidence)(float, offset_time, offset_time))

TEST(CloudPcl, OfficialRawLayoutMatchesXNavConsumer) {
  odin_driver::sdk::OdinPointCloudPacket packet;
  odin_driver::sdk::OdinRawPoint<float> good;
  good.x = 1000;
  good.y = -2000;
  good.z = 500;
  good.intensity = 255;
  good.confidence = 201;
  packet.points.assign(49152, good);
  packet.points[1535] = {};
  packet.points[6143].z = std::numeric_limits<float>::infinity();
  const auto message = odin_driver::rawCloud(packet, {});

  // Use PCL's actual field mapper; wire offsets deliberately differ from the
  // compiler-aligned consumer offsets, particularly for the float at byte 14.
  pcl::PCLPointCloud2 wire;
  wire.width = message.width;
  wire.height = message.height;
  wire.is_bigendian = message.is_bigendian;
  wire.point_step = message.point_step;
  wire.row_step = message.row_step;
  wire.is_dense = message.is_dense;
  wire.data = message.data;
  for (const auto& field : message.fields) {
    pcl::PCLPointField mapped;
    mapped.name = field.name;
    mapped.offset = field.offset;
    mapped.datatype = field.datatype;
    mapped.count = field.count;
    wire.fields.push_back(mapped);
  }
  pcl::PointCloud<OdinConsumerPoint> output;
  pcl::fromPCLPointCloud2(wire, output);
  ASSERT_EQ(output.size(), packet.points.size());
  EXPECT_FALSE(output.is_dense);
  const std::vector<std::pair<size_t, float>> scan_times = {
      {0, 0}, {1536, .00304414f}, {6144, .01217656f}, {49151, .09436834f}};
  for (const auto& sample : scan_times) {
    const auto i = sample.first;
    EXPECT_FLOAT_EQ(output[i].x, 1);
    EXPECT_FLOAT_EQ(output[i].y, -2);
    EXPECT_FLOAT_EQ(output[i].z, .5);
    EXPECT_EQ(output[i].intensity, 255);
    EXPECT_EQ(output[i].confidence, 201);
    EXPECT_NEAR(output[i].offset_time, sample.second, 1e-8);
  }
  for (size_t i : {1535u, 6143u}) {
    EXPECT_TRUE(std::isnan(output[i].x));
    EXPECT_TRUE(std::isnan(output[i].y));
    EXPECT_TRUE(std::isnan(output[i].z));
    EXPECT_NEAR(output[i].offset_time, i == 1535 ? 0 : .00913242f, 1e-8);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
