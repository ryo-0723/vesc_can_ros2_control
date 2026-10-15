#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "vesc_can_driver/vesc_transport.hpp"

namespace vesc_can_driver
{
namespace
{
using namespace std::chrono_literals;
struct Probe
{
  std::mutex mutex;
  std::vector<VescCommand> sent;
  std::vector<std::uint8_t> targets;
  std::vector<DriverFault> faults;
  std::atomic<bool> hybrid_running{false};
  std::atomic<int> fault{0};
  std::atomic<int> stopped{0};
};

// An alternative backend using typed values only: no CAN frames or CAN protocol dependency.
class MemoryInterface final : public VescInterface
{
public:
  explicit MemoryInterface(std::shared_ptr<Probe> probe) : probe_(std::move(probe))
  {
  }
  ~MemoryInterface() override
  {
    stop();
  }
  void configure_targets(std::span<const std::uint8_t> ids) override
  {
    probe_->targets.assign(ids.begin(), ids.end());
  }
  void start(FeedbackCallback on_feedback, ErrorCallback) override
  {
    feedback_ = std::move(on_feedback);
    thread_ = std::jthread([this](std::stop_token stop) {
      while (!stop.stop_requested()) {
        for (auto id : probe_->targets) {
          const auto erpm = id == 31 && !probe_->hybrid_running.load() ? 0 : 420;
          feedback_(MotionFeedback{id, erpm, 1.5, 0.5});
          feedback_(ControlFeedback{id, 30.0, 31.0, 1.2, 90.0});
          feedback_(PositionFeedback{id, -42, 24.0});
        }
        if (const auto code = probe_->fault.exchange(0)) {
          feedback_(FaultCode{probe_->targets.front(), static_cast<std::uint8_t>(code)});
        }
        std::this_thread::sleep_for(2ms);
      }
    });
  }
  void stop() noexcept override
  {
    if (thread_.joinable()) {
      thread_.request_stop();
      thread_.join();
      ++probe_->stopped;
    }
    feedback_ = {};
  }
  void send(const VescCommand &command) override
  {
    {
      std::lock_guard<std::mutex> lock(probe_->mutex);
      probe_->sent.push_back(command);
    }
    // Immediate responses verify that transport never holds the state lock across send().
    if (command.type == VescCommandType::FaultRequest && feedback_) {
      feedback_(FaultCode{command.controller_id, 0});
    }
  }

private:
  std::shared_ptr<Probe> probe_;
  FeedbackCallback feedback_;
  std::jthread thread_;
};

class InterfaceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
  template <typename Predicate> static bool wait_for(Predicate predicate)
  {
    const auto until = SteadyClock::now() + 2s;
    while (SteadyClock::now() < until) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }
};

TEST_F(InterfaceTest, AlternativeBackendSupportsFiveModesAndSynchronousFaultReplies)
{
  std::vector<MotorConfig> configs;
  for (auto [id, mode] :
    {std::pair{12, ControlMode::Current}, {11, ControlMode::Velocity}, {21, ControlMode::Position},
      {41, ControlMode::DutyCycle}, {31, ControlMode::CurrentVelocity}}) {
    MotorConfig cfg;
    cfg.name = "motor" + std::to_string(id);
    cfg.motor_id = id;
    cfg.controller_id = id;
    cfg.mode = mode;
    cfg.command_timeout = 2s;
    if (mode == ControlMode::CurrentVelocity) {
      cfg.startup_current_a = 2.0;
      cfg.rpm_threshold = 50.0;
    }
    configs.push_back(cfg);
  }
  auto probe = std::make_shared<Probe>();
  auto node = std::make_shared<rclcpp::Node>("memory_interface_io");
  TransportOptions opts;
  opts.auto_activate = true;
  opts.fault_period = 20ms;
  VescTransport transport(configs, opts, node, std::make_unique<MemoryInterface>(probe));
  transport.start();
  ASSERT_TRUE(transport.set_commands({{0, 1.5}, {1, 2.0}, {2, 0.25}, {3, -0.25}, {4, 2.0}}));
  const auto seen = [&](std::uint8_t id, VescCommandType type, double value) {
    std::lock_guard<std::mutex> lock(probe->mutex);
    return std::any_of(probe->sent.begin(), probe->sent.end(), [&](const auto &command) {
      return command.controller_id == id && command.type == type &&
             std::abs(command.value - value) < 1e-9;
    });
  };
  ASSERT_TRUE(wait_for([&]() {
    return seen(12, VescCommandType::Current, 1.5) && seen(11, VescCommandType::Rpm, 134) &&
           seen(21, VescCommandType::Position, 0.25 * 180.0 / kPi) &&
           seen(41, VescCommandType::Duty, -0.25) && seen(31, VescCommandType::Current, 2.0);
  }));
  probe->hybrid_running = true;
  ASSERT_TRUE(wait_for([&]() { return seen(31, VescCommandType::Rpm, 134); }));
  ASSERT_TRUE(wait_for([&]() { return transport.snapshot().front().fault_code == 0; }));
  const auto states = transport.snapshot();
  EXPECT_NEAR(states[1].vel_rad_s, 2.0 * kPi, 1e-9);
  EXPECT_NEAR(states[1].pos_rad, -2.0 * kPi, 1e-9);
  EXPECT_EQ(probe->targets, (std::vector<std::uint8_t>{12, 11, 21, 41, 31}));
  EXPECT_FALSE(transport.faulted());
  transport.shutdown();
  EXPECT_EQ(probe->stopped.load(), 1);
  for (auto id : probe->targets) {
    EXPECT_TRUE(seen(id, VescCommandType::Current, 0.0));
  }
}

TEST_F(InterfaceTest, UnsolicitedValidatedFaultStopsOnlyItsTransportAndRebootIsTyped)
{
  MotorConfig cfg;
  cfg.name = "motor";
  cfg.motor_id = 1;
  cfg.controller_id = 11;
  cfg.mode = ControlMode::Current;
  cfg.command_timeout = 2s;
  TransportOptions opts;
  opts.auto_activate = true;
  opts.fault_period = 0ms;
  auto first = std::make_shared<Probe>();
  auto second = std::make_shared<Probe>();
  VescTransport a({cfg}, opts, std::make_shared<rclcpp::Node>("alternative_a"),
    std::make_unique<MemoryInterface>(first), [&](const DriverFault &fault) {
      std::lock_guard<std::mutex> lock(first->mutex);
      first->faults.push_back(fault);
    });
  VescTransport b({cfg}, opts, std::make_shared<rclcpp::Node>("alternative_b"),
    std::make_unique<MemoryInterface>(second));
  a.start();
  b.start();
  ASSERT_TRUE(a.set_commands({{0, 1.0}}));
  ASSERT_TRUE(b.set_commands({{0, 2.0}}));
  ASSERT_TRUE(wait_for([&]() { return a.active() && b.active(); }));
  first->fault = 4;
  ASSERT_TRUE(wait_for([&]() { return a.faulted(); }));
  EXPECT_FALSE(a.active());
  EXPECT_TRUE(b.active());
  ASSERT_TRUE(wait_for([&]() {
    std::lock_guard<std::mutex> lock(first->mutex);
    return first->faults.size() == 1;
  }));
  {
    std::lock_guard<std::mutex> lock(first->mutex);
    ASSERT_TRUE(first->faults.front().device_fault.has_value());
    EXPECT_EQ(first->faults.front().device_fault->code, 4);
  }
  ASSERT_TRUE(a.reboot(0));
  ASSERT_TRUE(wait_for([&]() {
    std::lock_guard<std::mutex> lock(first->mutex);
    return std::any_of(first->sent.begin(), first->sent.end(), [](const auto &command) {
      return command.controller_id == 11 && command.type == VescCommandType::Reboot;
    });
  }));
  EXPECT_FALSE(a.snapshot().front().status1_at.has_value());
  EXPECT_FALSE(a.set_commands({{0, 3.0}}));
  a.shutdown();
  const auto until = SteadyClock::now() + 50ms;
  while (SteadyClock::now() < until) {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_TRUE(b.active());
  b.shutdown();
  EXPECT_EQ(first->stopped.load(), 1);
  EXPECT_EQ(second->stopped.load(), 1);
}

TEST(MotorSafety, IndependentMotorChecksAndLatchesTimeoutAndInvalidCommand)
{
  MotorConfig cfg;
  cfg.name = "single";
  cfg.controller_id = 11;
  cfg.mode = ControlMode::Current;
  cfg.command_timeout = 100ms;
  const auto now = TimePoint(1s);
  MotorController motor(cfg);
  ASSERT_TRUE(motor.receive(MotionFeedback{11, 420, 1.5, 0.5}, now));
  ASSERT_TRUE(motor.set_command(1.0, now));
  EXPECT_TRUE(motor.command(now).error.empty());
  const auto expired = motor.command(now + 101ms);
  EXPECT_EQ(expired.error, "command timeout");
  EXPECT_EQ(expired.command.type, VescCommandType::Current);
  EXPECT_EQ(expired.command.controller_id, 11);
  EXPECT_DOUBLE_EQ(expired.command.value, 0.0);
  EXPECT_FALSE(motor.set_command(2.0, now + 102ms));
  motor.reset_command(now + 102ms);
  ASSERT_TRUE(motor.set_command(2.0, now + 102ms));
  EXPECT_FALSE(motor.set_command(kUnknownValue, now + 103ms));
  EXPECT_EQ(motor.command(now + 103ms).error, "invalid command");
  EXPECT_DOUBLE_EQ(motor.command(now + 103ms).command.value, 0.0);
}
TEST(MotorSafety, FreshFeedbackAndHealthyReplyCannotReplayAFaultedCommand)
{
  MotorConfig cfg;
  cfg.name = "single";
  cfg.controller_id = 11;
  cfg.mode = ControlMode::Current;
  cfg.command_timeout = 2s;
  cfg.feedback_timeout = 100ms;
  const auto now = TimePoint(1s);
  MotorController motor(cfg);
  ASSERT_TRUE(motor.receive(MotionFeedback{11, 420, 1.5, 0.5}, now));
  ASSERT_TRUE(motor.set_command(2.0, now));
  EXPECT_EQ(motor.command(now + 101ms).error, "feedback stale or position outside limits");
  ASSERT_TRUE(motor.receive(MotionFeedback{11, 420, 1.5, 0.5}, now + 102ms));
  EXPECT_FALSE(motor.command(now + 102ms).error.empty());
  EXPECT_FALSE(motor.set_command(1.0, now + 102ms));
  motor.reset_command(now + 103ms);
  ASSERT_TRUE(motor.set_command(1.0, now + 103ms));
  ASSERT_TRUE(motor.receive(FaultCode{11, 4}, now + 104ms));
  EXPECT_EQ(motor.command(now + 104ms).error, "VESC fault");
  EXPECT_DOUBLE_EQ(motor.command(now + 104ms).command.value, 0.0);
  ASSERT_TRUE(motor.receive(FaultCode{11, 0}, now + 105ms));
  EXPECT_FALSE(motor.set_command(3.0, now + 105ms));
  motor.reset_command(now + 106ms);
  EXPECT_TRUE(motor.command(now + 106ms).error.empty());
  EXPECT_DOUBLE_EQ(motor.command(now + 106ms).command.value, 0.0);
  ASSERT_TRUE(motor.set_command(3.0, now + 107ms));
  EXPECT_DOUBLE_EQ(motor.command(now + 107ms).command.value, 3.0);
}

}  // namespace
}  // namespace vesc_can_driver
