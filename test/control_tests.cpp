#include <dynamic_reconfigure/Reconfigure.h>
#include <gtest/gtest.h>
#include <odin2_ros_driver/GetAdaptive.h>
#include <odin2_ros_driver/GetFilters.h>
#include <odin2_ros_driver/GetMode.h>
#include <odin2_ros_driver/SetAdaptive.h>
#include <odin2_ros_driver/SetFilters.h>
#include <odin2_ros_driver/SetStream.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <thread>

#include "odin2_ros_driver/driver.hpp"

namespace {
using namespace odin_driver;
namespace msg = odin2_ros_driver;
// A device whose physical values persist across command reconnects. Aggregate
// commands deliberately behave like the documented firmware placeholders.
class RecordingSdk : public SdkFacade {
 public:
  std::mutex mutex;
  sdk::OdinAdaptiveFilterParam actual{6000, 2000};
  std::vector<Command> calls;
  unsigned connections = 0, adaptive_reads = 0, adaptive_writes = 0;
  bool fail_restore = false;
  std::function<void(const Command&, Reply&)> inject;
  std::vector<sdk::DiscoveredDevice> discover(uint32_t) override {
    sdk::DiscoveredDevice d;
    d.sn = "CONTROL";
    return {d};
  }
  uint32_t connect(const sdk::DiscoveredDevice&) override {
    std::lock_guard<std::mutex> lock(mutex);
    return ++connections;
  }
  void disconnect(uint32_t) override {}
  void callbacks(uint32_t, PacketCallback) override {}
  Reply start(uint32_t, Channel, const StreamSettings&, uint32_t) override {
    return {};
  }
  Reply close(uint32_t, Channel) override {
    return {};
  }
  void sync(uint32_t, bool, uint32_t) override {}
  Reply execute(uint32_t, const Command& c) override {
    std::lock_guard<std::mutex> lock(mutex);
    calls.push_back(c);
    Reply r;
    if (c.kind == CommandKind::GetAdaptive) {
      ++adaptive_reads;
      r.adaptive = actual;
    } else if (c.kind == CommandKind::SetAdaptive) {
      ++adaptive_writes;
      if (fail_restore && c.adaptive.cdf_percentile == 6000) {
        r.code = sdk::OdinResult::kCommunicationError;
        r.message = "injected restore failure";
      } else
        actual = c.adaptive;
    } else if (c.kind == CommandKind::GetDeviceMode)
      r.mode = 1;
    if (inject)
      inject(c, r);
    return r;
  }
};

class Controls : public testing::Test {
 protected:
  std::shared_ptr<RecordingSdk> sdk = std::make_shared<RecordingSdk>();
  std::unique_ptr<DeviceDriver> driver;
  std::unique_ptr<ros::AsyncSpinner> spinner;
  std::string ns;
  void SetUp() override {
    static unsigned serial = 0;
    ns = "/control_" + std::to_string(++serial);
    auto cfg = defaults();
    cfg.sn = "CONTROL";
    for (auto& p : cfg.streams)
      p.second.enabled = false;
    driver = std::make_unique<DeviceDriver>(ros::NodeHandle(ns), cfg, sdk, [](sdk::OdinDataFormat) {
      return std::vector<decoding::Candidate>{};
    });
    spinner = std::make_unique<ros::AsyncSpinner>(4);
    spinner->start();
    driver->start();
    ASSERT_TRUE(wait([&] {
      msg::GetMode query;
      return ros::service::call(ns + "/get_device_mode", query) && query.response.result.success;
    }));
  }
  void TearDown() override {
    driver->stop();
    driver.reset();
    spinner->stop();
    ros::param::del(ns);
  }
  static bool wait(std::function<bool()> predicate) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    do {
      if (predicate())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < end);
    return false;
  }
  msg::SetAdaptive set(uint16_t percentile = 8000, uint16_t threshold = 4000) {
    msg::SetAdaptive call;
    call.request.cdf_percentile = percentile;
    call.request.outdoor_threshold = threshold;
    EXPECT_TRUE(ros::service::call(ns + "/set_adaptive", call));
    return call;
  }
  unsigned count(CommandKind kind) {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    return std::count_if(sdk->calls.begin(), sdk->calls.end(),
                         [&](const auto& c) { return c.kind == kind; });
  }
};

TEST_F(Controls, AdaptiveUsesDedicatedCommandAndCompatibilityWrapper) {
  auto changed = set();
  ASSERT_TRUE(changed.response.result.success);
  EXPECT_EQ(changed.response.effective_cdf_percentile, 8000);
  EXPECT_EQ(count(CommandKind::SetFilters), 0u);
  EXPECT_EQ(count(CommandKind::GetFilters), 0u);
  EXPECT_EQ(count(CommandKind::SetAdaptive), 1u);
  msg::GetFilters query;
  ASSERT_TRUE(ros::service::call(ns + "/get_filters", query));
  EXPECT_TRUE(query.response.result.success);
  EXPECT_TRUE(query.response.filters.has_adaptive);
  EXPECT_FALSE(query.response.filters.has_confidence);
  EXPECT_FALSE(query.response.filters.has_range);
  EXPECT_EQ(query.response.filters.cdf_percentile, 8000);
}

TEST_F(Controls, UnsupportedMixedFiltersAreRejectedWithoutAnyDeviceWrite) {
  msg::SetFilters call;
  call.request.filters.has_confidence = true;
  call.request.filters.has_adaptive = true;
  call.request.filters.cdf_percentile = 8000;
  ASSERT_TRUE(ros::service::call(ns + "/set_filters", call));
  EXPECT_FALSE(call.response.result.success);
  EXPECT_EQ(call.response.result.sdk_code, static_cast<uint8_t>(sdk::OdinResult::kInvalidArgument));
  EXPECT_EQ(call.response.result.message.find("UNSUPPORTED_FILTER:"), 0u);
  EXPECT_FALSE(call.response.effective.has_adaptive);
  EXPECT_EQ(count(CommandKind::SetFilters), 0u);
  EXPECT_EQ(count(CommandKind::SetAdaptive), 0u);
  call.request.filters.has_confidence = false;
  call.request.filters.has_range = true;
  ASSERT_TRUE(ros::service::call(ns + "/set_filters", call));
  EXPECT_FALSE(call.response.result.success);
  // Repeated unrelated reconnects cannot replay a poisoned empty filter set.
  for (unsigned i = 0; i < 2; ++i) {
    msg::SetStream stream;
    stream.request.config.channel = static_cast<uint8_t>(Channel::kRawPoint);
    stream.request.config.format = static_cast<uint8_t>(sdk::OdinDataFormat::kUnknown);
    stream.request.config.publish_rate = i + 1;
    ASSERT_TRUE(ros::service::call(ns + "/set_stream", stream));
    EXPECT_TRUE(stream.response.result.success) << stream.response.result.message;
  }
  EXPECT_EQ(count(CommandKind::SetFilters), 0u);
  EXPECT_EQ(count(CommandKind::SetAdaptive), 0u);
}

TEST_F(Controls, FailedPreReadDoesNotWrite) {
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->inject = [](const Command& c, Reply& r) {
      if (c.kind == CommandKind::GetAdaptive)
        r.code = sdk::OdinResult::kCommunicationError;
    };
  }
  EXPECT_FALSE(set().response.result.success);
  EXPECT_EQ(count(CommandKind::SetAdaptive), 0u);
  EXPECT_EQ(count(CommandKind::SetFilters), 0u);
}

TEST_F(Controls, TimeoutAfterWriteRestoresAndVerifiesActualValue) {
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->inject = [this](const Command& c, Reply& r) {
      if (c.kind == CommandKind::SetAdaptive && sdk->adaptive_writes == 1)
        r.code = sdk::OdinResult::kTimeout;
    };
  }
  auto call = set();
  EXPECT_FALSE(call.response.result.success);
  EXPECT_NE(call.response.result.message.find("restored"), std::string::npos);
  EXPECT_EQ(call.response.effective_cdf_percentile, 6000);
  EXPECT_EQ(count(CommandKind::SetAdaptive), 2u);
  EXPECT_EQ(count(CommandKind::GetAdaptive), 2u);
}

TEST_F(Controls, ReadbackMismatchRestoresActualInsteadOfDesired) {
  ASSERT_TRUE(set(7000, 3000).response.result.success);
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->actual = {6000, 2000};
    sdk->inject = [this](const Command& c, Reply& r) {
      if (c.kind == CommandKind::GetAdaptive && sdk->adaptive_reads == 4)
        ++r.adaptive.cdf_percentile;
    };
  }
  auto call = set();
  EXPECT_FALSE(call.response.result.success);
  EXPECT_EQ(call.response.effective_cdf_percentile, 6000);
  std::lock_guard<std::mutex> lock(sdk->mutex);
  EXPECT_EQ(sdk->actual.cdf_percentile, 6000);
}

TEST_F(Controls, UnknownRestoreWithoutPriorDesiredBlocksReadyUntilVerifiedOnReconnect) {
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->fail_restore = true;
    sdk->inject = [this](const Command& c, Reply& r) {
      if (c.kind == CommandKind::SetAdaptive && sdk->adaptive_writes == 1)
        r.code = sdk::OdinResult::kTimeout;
    };
  }
  auto call = set();
  EXPECT_FALSE(call.response.result.success);
  EXPECT_EQ(call.response.effective_cdf_percentile, 0);
  EXPECT_NE(call.response.result.message.find("not ready"), std::string::npos);
  EXPECT_TRUE(wait([&] { return count(CommandKind::SetAdaptive) >= 3; }));
  msg::GetMode query;
  ASSERT_TRUE(ros::service::call(ns + "/get_device_mode", query));
  EXPECT_FALSE(query.response.result.success);
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->fail_restore = false;
  }
  ASSERT_TRUE(wait([&] {
    return ros::service::call(ns + "/get_device_mode", query) && query.response.result.success;
  }));
  std::lock_guard<std::mutex> lock(sdk->mutex);
  EXPECT_EQ(sdk->actual.cdf_percentile, 6000);
}

TEST_F(Controls, RestoreReadFailureKeepsPendingActualAheadOfAcceptedDesired) {
  ASSERT_TRUE(set(7000, 3000).response.result.success);
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->actual = {6000, 2000};
    sdk->inject = [this](const Command& c, Reply& r) {
      // First request used reads 1/2. Read 3 captures the actual baseline;
      // all subsequent readbacks fail, including the rollback readback.
      if (c.kind == CommandKind::GetAdaptive && sdk->adaptive_reads >= 4)
        r.code = sdk::OdinResult::kTimeout;
    };
  }
  const auto call = set();
  EXPECT_FALSE(call.response.result.success);
  EXPECT_EQ(call.response.effective_cdf_percentile, 0);
  EXPECT_TRUE(wait([&] { return count(CommandKind::SetAdaptive) >= 4; }));
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->inject = {};
  }
  msg::GetAdaptive query;
  ASSERT_TRUE(wait([&] {
    return ros::service::call(ns + "/get_adaptive", query) && query.response.result.success;
  }));
  EXPECT_EQ(query.response.cdf_percentile, 6000);
  EXPECT_EQ(query.response.outdoor_threshold, 2000);
}

TEST_F(Controls, SuccessfulAdaptiveDynamicBatchUsesDedicatedCommandAndClearsApplyFlag) {
  dynamic_reconfigure::Reconfigure request;
  dynamic_reconfigure::BoolParameter flag;
  flag.name = "apply_adaptive";
  flag.value = true;
  request.request.config.bools.push_back(flag);
  dynamic_reconfigure::IntParameter value;
  value.name = "cdf_percentile";
  value.value = 8500;
  request.request.config.ints.push_back(value);
  ASSERT_TRUE(ros::service::call(ns + "/set_parameters", request));
  bool found = false;
  for (const auto& p : request.response.config.bools)
    if (p.name == flag.name) {
      found = true;
      EXPECT_FALSE(p.value);
    }
  EXPECT_TRUE(found);
  EXPECT_EQ(count(CommandKind::SetAdaptive), 1u);
  EXPECT_EQ(count(CommandKind::SetFilters), 0u);
  msg::GetAdaptive query;
  ASSERT_TRUE(ros::service::call(ns + "/get_adaptive", query));
  EXPECT_EQ(query.response.cdf_percentile, 8500);
}

TEST_F(Controls, MixedDynamicFailureRestoresStreamsAndRetainsPendingAdaptiveRecovery) {
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->fail_restore = true;
    sdk->inject = [this](const Command& c, Reply& r) {
      if (c.kind == CommandKind::SetAdaptive && sdk->adaptive_writes == 1)
        r.code = sdk::OdinResult::kTimeout;
    };
  }
  dynamic_reconfigure::Reconfigure request;
  dynamic_reconfigure::BoolParameter flag;
  flag.name = "apply_adaptive";
  flag.value = true;
  request.request.config.bools.push_back(flag);
  dynamic_reconfigure::DoubleParameter rate;
  rate.name = "raw_point_publish_rate";
  rate.value = 5;
  request.request.config.doubles.push_back(rate);
  ASSERT_TRUE(ros::service::call(ns + "/set_parameters", request));
  for (const auto& p : request.response.config.doubles)
    if (p.name == rate.name)
      EXPECT_EQ(p.value, 0);
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->fail_restore = false;
  }
  msg::GetAdaptive query;
  ASSERT_TRUE(wait([&] {
    return ros::service::call(ns + "/get_adaptive", query) && query.response.result.success;
  }));
  EXPECT_EQ(query.response.cdf_percentile, 6000);
}

TEST_F(Controls, MixedDynamicFailurePreservesActualValueDifferentFromAcceptedDesired) {
  ASSERT_TRUE(set(7000, 3000).response.result.success);
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->actual = {6000, 2000};
    sdk->inject = [](const Command& c, Reply& r) {
      if (c.kind == CommandKind::SetAdaptive && c.adaptive.cdf_percentile == 8000)
        r.code = sdk::OdinResult::kTimeout;
    };
  }
  dynamic_reconfigure::Reconfigure request;
  dynamic_reconfigure::BoolParameter flag;
  flag.name = "apply_adaptive";
  flag.value = true;
  request.request.config.bools.push_back(flag);
  dynamic_reconfigure::IntParameter percentile;
  percentile.name = "cdf_percentile";
  percentile.value = 8000;
  request.request.config.ints.push_back(percentile);
  dynamic_reconfigure::DoubleParameter rate;
  rate.name = "raw_point_publish_rate";
  rate.value = 5;
  request.request.config.doubles.push_back(rate);
  ASSERT_TRUE(ros::service::call(ns + "/set_parameters", request));
  for (const auto& p : request.response.config.doubles)
    if (p.name == rate.name)
      EXPECT_EQ(p.value, 0);
  msg::GetAdaptive query;
  ASSERT_TRUE(ros::service::call(ns + "/get_adaptive", query));
  ASSERT_TRUE(query.response.result.success);
  EXPECT_EQ(query.response.cdf_percentile, 6000);
  EXPECT_EQ(query.response.outdoor_threshold, 2000);
}

TEST_F(Controls, MixedDynamicPreReadFailureLeavesStreamsAndDeviceUntouched) {
  ASSERT_TRUE(set(7000, 3000).response.result.success);
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->inject = [](const Command& c, Reply& r) {
      if (c.kind == CommandKind::GetAdaptive)
        r.code = sdk::OdinResult::kTimeout;
    };
  }
  dynamic_reconfigure::Reconfigure request;
  dynamic_reconfigure::BoolParameter flag;
  flag.name = "apply_adaptive";
  flag.value = true;
  request.request.config.bools.push_back(flag);
  dynamic_reconfigure::DoubleParameter rate;
  rate.name = "raw_point_publish_rate";
  rate.value = 5;
  request.request.config.doubles.push_back(rate);
  ASSERT_TRUE(ros::service::call(ns + "/set_parameters", request));
  for (const auto& p : request.response.config.doubles)
    if (p.name == rate.name)
      EXPECT_EQ(p.value, 0);
  std::lock_guard<std::mutex> lock(sdk->mutex);
  EXPECT_EQ(sdk->connections, 1u);
  EXPECT_EQ(sdk->adaptive_writes, 1u);
}

TEST_F(Controls, StartupDesiredReadbackMustBeVerifiedBeforeReady) {
  driver->stop();
  driver.reset();
  auto cfg = defaults();
  cfg.sn = "CONTROL";
  cfg.initial_adaptive = sdk::OdinAdaptiveFilterParam{7500, 3500};
  for (auto& p : cfg.streams)
    p.second.enabled = false;
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->inject = [](const Command& c, Reply& r) {
      if (c.kind == CommandKind::GetAdaptive)
        ++r.adaptive.outdoor_threshold;
    };
  }
  driver = std::make_unique<DeviceDriver>(ros::NodeHandle(ns), cfg, sdk, [](sdk::OdinDataFormat) {
    return std::vector<decoding::Candidate>{};
  });
  driver->start();
  ASSERT_TRUE(wait([&] { return count(CommandKind::SetAdaptive) >= 1; }));
  msg::GetMode mode;
  ASSERT_TRUE(ros::service::call(ns + "/get_device_mode", mode));
  EXPECT_FALSE(mode.response.result.success);
  {
    std::lock_guard<std::mutex> lock(sdk->mutex);
    sdk->inject = {};
  }
  msg::GetAdaptive query;
  ASSERT_TRUE(wait([&] {
    return ros::service::call(ns + "/get_adaptive", query) && query.response.result.success;
  }));
  EXPECT_EQ(query.response.cdf_percentile, 7500);
  EXPECT_EQ(query.response.outdoor_threshold, 3500);
}

TEST_F(Controls, DeprecatedDynamicFiltersRejectBeforeStreamReconfiguration) {
  dynamic_reconfigure::Reconfigure request;
  dynamic_reconfigure::BoolParameter flag;
  flag.name = "apply_filters";
  flag.value = true;
  request.request.config.bools.push_back(flag);
  dynamic_reconfigure::DoubleParameter rate;
  rate.name = "raw_point_publish_rate";
  rate.value = 5;
  request.request.config.doubles.push_back(rate);
  ASSERT_TRUE(ros::service::call(ns + "/set_parameters", request));
  for (const auto& p : request.response.config.doubles)
    if (p.name == rate.name)
      EXPECT_EQ(p.value, 0);
  for (const auto& name : {"confidence_threshold", "min_depth"}) {
    request.request.config.bools.front().value = false;
    dynamic_reconfigure::IntParameter deprecated;
    deprecated.name = name;
    deprecated.value = 123;
    request.request.config.ints = {deprecated};
    ASSERT_TRUE(ros::service::call(ns + "/set_parameters", request));
    for (const auto& p : request.response.config.doubles)
      if (p.name == rate.name)
        EXPECT_EQ(p.value, 0);
  }
  std::lock_guard<std::mutex> lock(sdk->mutex);
  EXPECT_EQ(sdk->connections, 1u);
}

TEST_F(Controls, UnsupportedYamlRejectedAtStartup) {
  XmlRpc::XmlRpcValue devices;
  devices[0]["name"] = "configured";
  devices[0]["sn"] = "CONTROL";
  devices[0]["filters"]["confidence_threshold"] = 80;
  ros::NodeHandle config(ns + "/config");
  config.setParam("devices", devices);
  EXPECT_THROW(loadConfig(config), std::invalid_argument);
}

TEST_F(Controls, AdaptiveYamlSingleFieldUsesDocumentedDefaultsAndRejectsEmptyObject) {
  XmlRpc::XmlRpcValue devices;
  devices[0]["name"] = "configured";
  devices[0]["sn"] = "CONTROL";
  devices[0]["filters"]["cdf_percentile"] = 7500;
  ros::NodeHandle config(ns + "/config");
  config.setParam("devices", devices);
  const auto loaded = loadConfig(config);
  ASSERT_TRUE(loaded[0].initial_adaptive.has_value());
  EXPECT_EQ(loaded[0].initial_adaptive->cdf_percentile, 7500);
  EXPECT_EQ(loaded[0].initial_adaptive->outdoor_threshold, 3000);
  devices[0]["filters"] = XmlRpc::XmlRpcValue();
  devices[0]["filters"].begin();  // Force an empty struct, as in YAML filters: {}.
  config.setParam("devices", devices);
  EXPECT_THROW(loadConfig(config), std::invalid_argument);
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
