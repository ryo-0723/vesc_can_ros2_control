#include "vesc_can_driver/vesc_driver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace vesc_can_driver
{
// Mode selection and unit conversion.
ControlMode parse_control_mode(std::string_view mode_name)
{
  if (mode_name == "current") {
    return ControlMode::Current;
  }
  if (mode_name == "velocity") {
    return ControlMode::Velocity;
  }
  if (mode_name == "position") {
    return ControlMode::Position;
  }
  if (mode_name == "current_velocity") {
    return ControlMode::CurrentVelocity;
  }
  throw std::invalid_argument("Unsupported control mode: " + std::string(mode_name));
}

std::string_view command_interface(ControlMode mode)
{
  switch (mode) {
    case ControlMode::Current:
      return "current";
    case ControlMode::Velocity:
    case ControlMode::CurrentVelocity:
      return "velocity";
    case ControlMode::Position:
      return "position";
  }
  throw std::invalid_argument("Unsupported control mode");
}

ControlMode resolve_control_mode(std::string_view mode_name, std::string_view interface_name)
{
  const auto resolved_mode = parse_control_mode(mode_name.empty() ? interface_name : mode_name);
  if (command_interface(resolved_mode) != interface_name) {
    throw std::invalid_argument("Control mode and command interface do not match");
  }
  return resolved_mode;
}

bool is_velocity_mode(ControlMode mode)
{
  return mode == ControlMode::Velocity || mode == ControlMode::CurrentVelocity;
}

double vel_to_erpm(const MotorConfig &cfg, double vel_rad_s)
{
  return vel_rad_s * 60.0 / (2.0 * kPi) * cfg.gear_ratio * cfg.pole_pairs * cfg.direction;
}

double erpm_to_vel(const MotorConfig &cfg, double electrical_rpm)
{
  return electrical_rpm * (2.0 * kPi) / (60.0 * cfg.gear_ratio * cfg.pole_pairs) * cfg.direction;
}

double pos_to_deg(const MotorConfig &cfg, double pos_rad)
{
  double pid_pos_deg = std::fmod(
    (cfg.zero_offset_rad + pos_rad * cfg.gear_ratio * cfg.direction) * 180.0 / kPi, 360.0);
  return pid_pos_deg < 0.0 ? pid_pos_deg + 360.0 : pid_pos_deg;
}

double deg_to_pos(const MotorConfig &cfg, double pid_pos_deg)
{
  return std::remainder(pid_pos_deg * kPi / 180.0 - cfg.zero_offset_rad, 2.0 * kPi) *
         cfg.direction / cfg.gear_ratio;
}

double apply_direction(const MotorConfig &cfg, double current_a)
{
  return current_a * cfg.direction;
}

// Configuration validation.
namespace
{
/// @brief 1 台の設定・時間・モード固有の指令範囲を検査する
void validate_config(const MotorConfig &cfg)
{
  command_interface(cfg.mode);  // Reject invalid enum values as well.
  const std::array numeric_settings = {cfg.pole_pairs, cfg.gear_ratio, cfg.direction,
    cfg.zero_offset_rad, cfg.max_vel_rad_s, cfg.max_current_a, cfg.min_pos_rad, cfg.max_pos_rad,
    cfg.vel_slew_rad_s2, cfg.startup_current_a, cfg.rpm_threshold};
  for (double setting_value : numeric_settings) {
    if (!std::isfinite(setting_value)) {
      throw std::invalid_argument(cfg.name + ": non-finite setting");
    }
  }
  if (cfg.controller_id == 255 || cfg.pole_pairs <= 0.0 ||
      std::floor(cfg.pole_pairs) != cfg.pole_pairs || cfg.gear_ratio <= 0.0 ||
      std::abs(cfg.direction) != 1.0 || cfg.max_vel_rad_s <= 0.0 || cfg.max_current_a <= 0.0 ||
      cfg.vel_slew_rad_s2 <= 0.0 || cfg.startup_current_a < 0.0 || cfg.rpm_threshold < 0.0 ||
      cfg.feedback_timeout.count() <= 0 || cfg.command_timeout.count() <= 0 ||
      cfg.startup_timeout.count() <= 0) {
    throw std::invalid_argument(cfg.name + ": invalid motor limits or timing");
  }
  const double max_erpm = std::abs(vel_to_erpm(cfg, cfg.max_vel_rad_s));
  if (is_velocity_mode(cfg.mode) &&
      (!std::isfinite(max_erpm) || max_erpm > std::numeric_limits<std::int32_t>::max())) {
    throw std::invalid_argument(cfg.name + ": speed limit exceeds VESC int32 ERPM range");
  }
  if (cfg.mode == ControlMode::Position &&
      (cfg.min_pos_rad >= cfg.max_pos_rad || cfg.min_pos_rad < -kPi / cfg.gear_ratio ||
        cfg.max_pos_rad > kPi / cfg.gear_ratio)) {
    throw std::invalid_argument(cfg.name + ": PID position requires a single-turn range");
  }
  if (cfg.max_current_a > std::numeric_limits<std::int32_t>::max() / 1000.0 ||
      cfg.startup_current_a > cfg.max_current_a) {
    throw std::invalid_argument(cfg.name + ": invalid current limit or startup current");
  }
  if (cfg.mode == ControlMode::CurrentVelocity) {
    if (cfg.startup_current_a <= 0.0 || cfg.rpm_threshold <= 0.0) {
      throw std::invalid_argument(
        cfg.name + ": current_velocity requires positive startup current and RPM threshold");
    }
  } else if (cfg.startup_current_a != 0.0) {
    throw std::invalid_argument(
      cfg.name + ": startup current is only supported in current_velocity mode");
  }
}
}  // namespace

void validate_configs(const std::vector<MotorConfig> &configs)
{
  if (configs.empty()) {
    throw std::invalid_argument("At least one motor is required");
  }
  std::set<std::string> names;
  std::set<std::uint8_t> ids;
  for (const auto &cfg : configs) {
    validate_config(cfg);
    if (cfg.name.empty() || !names.insert(cfg.name).second ||
        !ids.insert(cfg.controller_id).second) {
      throw std::invalid_argument("Motor names and controller IDs must be unique");
    }
  }
}

// Shared feedback freshness and command limits.
bool feedback_fresh(
  const std::optional<TimePoint> &received_at, TimePoint now, std::chrono::milliseconds timeout)
{
  return received_at && now >= *received_at && now - *received_at <= timeout;
}

bool command_is_valid(const MotorConfig &cfg, double value)
{
  if (!std::isfinite(value)) {
    return false;
  }
  switch (cfg.mode) {
    case ControlMode::Current:
      return std::abs(value) <= cfg.max_current_a;
    case ControlMode::Velocity:
    case ControlMode::CurrentVelocity:
      return std::abs(value) <= cfg.max_vel_rad_s;
    case ControlMode::Position:
      return value >= cfg.min_pos_rad && value <= cfg.max_pos_rad;
  }
  return false;
}

bool state_ready(const MotorConfig &cfg, const MotorState &state, TimePoint now)
{
  constexpr double kPositionToleranceRad = 1e-3;
  const auto timeout = cfg.feedback_timeout;
  // A known device fault remains unsafe until a healthy reply is received, even if stale.
  if (state.fault_code > 0 || !feedback_fresh(state.status1_at, now, timeout)) {
    return false;
  }
  if (cfg.mode == ControlMode::Position) {
    if (!feedback_fresh(state.status4_at, now, timeout) || !std::isfinite(state.pos_rad) ||
        state.pos_rad < cfg.min_pos_rad - kPositionToleranceRad ||
        state.pos_rad > cfg.max_pos_rad + kPositionToleranceRad) {
      return false;
    }
  }
  return !cfg.require_tachometer || feedback_fresh(state.status5_at, now, timeout);
}

namespace
{
/// @brief 関節方向の電流 [A] を VESC 方向へ変換して指令フレームを作る
can_msgs::msg::Frame make_current_frame(const MotorConfig &cfg, double current_a)
{
  return protocol::make_set_current_frame(cfg.controller_id, apply_direction(cfg, current_a));
}
}  // namespace

// STATUS reception and motor state.
DriverCore::DriverCore(std::vector<MotorConfig> configs)
  : configs_(std::move(configs)), states_(configs_.size()), cmds_(configs_.size())
{
  validate_configs(configs_);
}

bool DriverCore::receive(const can_msgs::msg::Frame &frame, TimePoint now)
{
  const auto controller_id = static_cast<std::uint8_t>(frame.id & protocol::kControllerIdMask);
  const auto config_it = std::find_if(configs_.begin(), configs_.end(),
    [controller_id](const MotorConfig &cfg) { return cfg.controller_id == controller_id; });
  if (config_it == configs_.end()) {
    return false;
  }
  const auto idx = static_cast<std::size_t>(config_it - configs_.begin());
  auto &state = states_[idx];
  protocol::Status1 status_1;
  protocol::Status2 status_2;
  protocol::Status3 status_3;
  protocol::Status4 status_4;
  protocol::Status5 status_5;
  protocol::Status6 status_6;
  if (protocol::decode_status_1(frame, status_1)) {
    state.vel_rad_s = erpm_to_vel(*config_it, status_1.electrical_rpm);
    state.electrical_rpm = status_1.electrical_rpm;
    state.motor_current_a = status_1.motor_current_a;
    state.duty_cycle = status_1.duty_cycle;
    state.status1_at = now;
  } else if (protocol::decode_status_2(frame, status_2)) {
    state.charge_drawn_ah = status_2.charge_drawn_ah;
    state.charge_regen_ah = status_2.charge_regen_ah;
    state.status2_at = now;
  } else if (protocol::decode_status_3(frame, status_3)) {
    state.energy_drawn_wh = status_3.energy_drawn_wh;
    state.energy_regen_wh = status_3.energy_regen_wh;
    state.status3_at = now;
  } else if (protocol::decode_status_4(frame, status_4)) {
    state.temperature_fet_c = status_4.temperature_fet_c;
    state.temperature_motor_c = status_4.temperature_motor_c;
    state.input_current_a = status_4.input_current_a;
    state.pid_pos_deg = status_4.motor_pid_position_deg;
    if (config_it->mode == ControlMode::Position) {
      state.pos_rad = deg_to_pos(*config_it, status_4.motor_pid_position_deg);
    }
    state.status4_at = now;
  } else if (protocol::decode_status_5(frame, status_5)) {
    state.tachometer_counts = status_5.tachometer_counts;
    if (config_it->mode != ControlMode::Position) {
      state.pos_rad = status_5.tachometer_counts * (2.0 * kPi) /
                      (6.0 * config_it->pole_pairs * config_it->gear_ratio) * config_it->direction;
    }
    state.input_voltage_v = status_5.input_voltage_v;
    state.status5_at = now;
  } else if (protocol::decode_status_6(frame, status_6)) {
    state.adc_1_v = status_6.adc_1_v;
    state.adc_2_v = status_6.adc_2_v;
    state.adc_3_v = status_6.adc_3_v;
    state.ppm = status_6.ppm;
    state.status6_at = now;
  } else {
    return false;
  }
  return true;
}

bool DriverCore::receive_fault(const protocol::FaultCode &fault, TimePoint now)
{
  const auto cfg = std::find_if(configs_.begin(), configs_.end(),
    [&fault](const MotorConfig &motor) { return motor.controller_id == fault.controller_id; });
  if (cfg == configs_.end()) {
    return false;
  }
  auto &state = states_[static_cast<std::size_t>(cfg - configs_.begin())];
  state.fault_code = fault.code;
  state.fault_at = now;
  if (fault.code != 0 && !faulted_) {
    stop_on_error(cfg->name + ": VESC fault_code=" + std::to_string(fault.code));
  }
  return true;
}

// Command activation, deadlines and four-mode frame generation.
bool DriverCore::ready(TimePoint now) const
{
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    if (!state_ready(configs_[idx], states_[idx], now)) {
      return false;
    }
  }
  return true;
}

bool DriverCore::activate(TimePoint now)
{
  if (!ready(now)) {
    return false;
  }
  active_ = true;
  faulted_ = false;
  fault_reason_.clear();
  active_at_ = now;
  std::fill(cmds_.begin(), cmds_.end(), Command{});
  return true;
}

void DriverCore::deactivate()
{
  active_ = false;
  std::fill(cmds_.begin(), cmds_.end(), Command{});
}

bool DriverCore::reset(std::size_t motor_idx)
{
  if (motor_idx >= states_.size()) {
    return false;
  }
  deactivate();
  states_[motor_idx] = MotorState{};
  faulted_ = false;
  fault_reason_.clear();
  return true;
}

void DriverCore::stop_on_error(const std::string &reason)
{
  deactivate();
  faulted_ = true;
  fault_reason_ = reason;
}

bool DriverCore::set_command(std::size_t idx, double value, TimePoint now)
{
  if (idx >= configs_.size() || !active_) {
    return false;
  }
  const auto &cfg = configs_[idx];
  if (!command_is_valid(cfg, value)) {
    stop_on_error(cfg.name + ": invalid command");
    return false;
  }
  auto &cmd = cmds_[idx];
  if (cfg.mode == ControlMode::CurrentVelocity && (!cmd.received_at || cmd.target * value <= 0.0)) {
    cmd.rpm_control_active = false;
    cmd.startup_at = now;
  }
  if (!cmd.received_at) {
    cmd.ramp_at = now;
  }
  cmd.target = value;
  cmd.received_at = now;
  return true;
}

void DriverCore::stop_frames(std::vector<can_msgs::msg::Frame> &frames) const
{
  frames.clear();
  for (const auto &cfg : configs_) {
    frames.push_back(make_current_frame(cfg, 0.0));
  }
}

bool DriverCore::check_commands(TimePoint now)
{
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    const auto &cfg = configs_[idx];
    const auto &cmd = cmds_[idx];
    if (!state_ready(cfg, states_[idx], now)) {
      stop_on_error(cfg.name + ": feedback stale or position outside limits");
      return false;
    }
    if (now - cmd.received_at.value_or(active_at_) > cfg.command_timeout) {
      stop_on_error(cfg.name + ": command timeout");
      return false;
    }
  }
  return true;
}

void DriverCore::command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> &frames)
{
  if (!active_ || !check_commands(now)) {
    stop_frames(frames);
    return;
  }
  frames.clear();
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    const auto frame = make_frame(idx, now);
    if (!frame) {
      stop_frames(frames);
      return;
    }
    frames.push_back(*frame);
    cmds_[idx].ramp_at = now;
  }
}

std::optional<can_msgs::msg::Frame> DriverCore::make_frame(std::size_t idx, TimePoint now)
{
  const auto &cfg = configs_[idx];
  auto &cmd = cmds_[idx];
  if (!cmd.received_at || (cfg.mode != ControlMode::Position && cmd.target == 0.0)) {
    cmd.ramp_vel = 0.0;
    cmd.rpm_control_active = false;
    return make_current_frame(cfg, 0.0);
  }
  switch (cfg.mode) {
    case ControlMode::Current:
      return make_current_frame(cfg, cmd.target);
    case ControlMode::Position:
      return protocol::make_set_position_frame(cfg.controller_id, pos_to_deg(cfg, cmd.target));
    case ControlMode::CurrentVelocity: {
      const auto &state = states_[idx];
      const double rpm_per_vel = cfg.gear_ratio * 60.0 / (2.0 * kPi);
      const double measured_rpm = std::abs(state.vel_rad_s) * rpm_per_vel;
      const double target_rpm = std::abs(cmd.target) * rpm_per_vel;
      const bool reached_threshold = cmd.target * state.vel_rad_s > 0.0 &&
                                     measured_rpm >= std::min(target_rpm, cfg.rpm_threshold);
      if (!cmd.rpm_control_active && reached_threshold) {
        cmd.rpm_control_active = true;
        cmd.ramp_vel = std::clamp(state.vel_rad_s, -cfg.max_vel_rad_s, cfg.max_vel_rad_s);
      }
      if (!cmd.rpm_control_active) {
        if (now - cmd.startup_at > cfg.startup_timeout) {
          stop_on_error(cfg.name + ": startup timeout");
          return std::nullopt;
        }
        return make_current_frame(cfg, std::copysign(cfg.startup_current_a, cmd.target));
      }
      // Once startup finishes, use the same velocity ramp as direct velocity mode.
      [[fallthrough]];
    }
    case ControlMode::Velocity: {
      const double dt_s = std::max(0.0, std::chrono::duration<double>(now - cmd.ramp_at).count());
      const double max_step = cfg.vel_slew_rad_s2 * dt_s;
      cmd.ramp_vel += std::clamp(cmd.target - cmd.ramp_vel, -max_step, max_step);
      const auto electrical_rpm =
        static_cast<std::int32_t>(std::llround(vel_to_erpm(cfg, cmd.ramp_vel)));
      return protocol::make_set_rpm_frame(cfg.controller_id, electrical_rpm);
    }
  }
  stop_on_error(cfg.name + ": unsupported control mode");
  return std::nullopt;
}
// ROS CAN transport: executor, latest commands and periodic transmission.
CanTransport::CanTransport(std::vector<MotorConfig> configs, TransportOptions opts,
  rclcpp::Node::SharedPtr can_node, std::function<void(const DriverFault &)> on_fault)
  : core_(std::move(configs)), opts_(std::move(opts)), can_node_(std::move(can_node)),
    on_fault_(std::move(on_fault)), fault_requests_(core_.motor_configs().size()),
    rx_after_(core_.motor_configs().size()), pending_cmds_(core_.motor_configs().size())
{
  if (!can_node_ || opts_.send_period.count() <= 0 || opts_.tx_topic.empty() ||
      opts_.rx_topic.empty() || opts_.tx_topic == opts_.rx_topic) {
    throw std::invalid_argument("Invalid CAN transport node, topics or period");
  }
  if (opts_.fault_period.count() < 0 || opts_.host_id == protocol::kBroadcastControllerId ||
      std::any_of(core_.motor_configs().begin(), core_.motor_configs().end(),
        [this](const MotorConfig &cfg) { return cfg.controller_id == opts_.host_id; })) {
    throw std::invalid_argument("Fault query period must be non-negative and host CAN ID unique");
  }

  cmd_frames_.reserve(core_.motor_configs().size());
  const auto tx_depth = std::max<std::size_t>(50, core_.motor_configs().size() * 4);
  const auto tx_qos = rclcpp::QoS(tx_depth).reliable().durability_volatile();
  tx_pub_ = can_node_->create_publisher<can_msgs::msg::Frame>(opts_.tx_topic, tx_qos);
  rx_sub_ = can_node_->create_subscription<can_msgs::msg::Frame>(opts_.rx_topic,
    rclcpp::SensorDataQoS().keep_last(512),
    [this](can_msgs::msg::Frame::ConstSharedPtr frame) { receive_frame(*frame); });
  send_timer_ = can_node_->create_wall_timer(opts_.send_period, [this]() { send_commands(); });
  if (opts_.fault_period.count() > 0) {
    fault_timer_ = can_node_->create_wall_timer(opts_.fault_period, [this]() { poll_faults(); });
  }
  can_exec_.add_node(can_node_);
}

CanTransport::~CanTransport()
{
  shutdown();
}

void CanTransport::start()
{
  if (shutdown_started_.load()) {
    throw std::logic_error("Transport has been shut down");
  }
  if (exec_thread_.joinable()) {
    throw std::logic_error("Transport already started");
  }
  exec_thread_ = std::thread([this]() {
    try {
      can_exec_.spin();
    } catch (const std::exception &error) {
      stop_on_error(std::string("CAN executor failed: ") + error.what());
      RCLCPP_ERROR(can_node_->get_logger(), "CAN executor failed: %s", error.what());
    }
  });
}

void CanTransport::shutdown() noexcept
{
  if (shutdown_started_.exchange(true)) {
    return;
  }
  stop_requested_.store(true);
  active_.store(false);
  can_exec_.cancel();
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
  try {
    if (fault_requested_.exchange(false)) {
      stop_on_error("Stop requested by hardware/topic command validation");
    }
    if (rclcpp::ok(can_node_->get_node_base_interface()->get_context())) {
      core_.stop_frames(cmd_frames_);
      publish_frames();
    }
  } catch (const std::exception &) {
    // Destruction cannot guarantee network delivery; MCU/VESC watchdogs remain necessary.
  }
  feedback_cv_.notify_all();
}

bool CanTransport::wait_ready(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  const bool condition_met = feedback_cv_.wait_for(state_lock, timeout,
    [this]() { return shutdown_started_.load() || core_.ready(SteadyClock::now()); });
  return condition_met && !shutdown_started_.load();
}

bool CanTransport::activate()
{
  if (shutdown_started_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  const auto now = SteadyClock::now();
  if (reboot_idx_ || std::any_of(rx_after_.begin(), rx_after_.end(),
                       [now](TimePoint after) { return now < after; })) {
    return false;
  }
  std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
  const bool active = core_.activate(now);
  fault_requested_.store(false);
  active_.store(active);
  faulted_.store(core_.faulted());
  stop_requested_.store(!active);
  return active;
}

void CanTransport::deactivate()
{
  stop_requested_.store(true);
  active_.store(false);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  core_.deactivate();
  std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
}

bool CanTransport::reboot(std::size_t motor_idx)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (shutdown_started_.load() || motor_idx >= pending_cmds_.size() || reboot_idx_ ||
      SteadyClock::now() < rx_after_[motor_idx] || (!opts_.auto_activate && active_.load())) {
    return false;
  }
  stop_requested_.store(true);
  active_.store(false);
  core_.deactivate();
  std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
  reboot_idx_ = motor_idx;
  return true;
}

void CanTransport::stop_on_error(const std::string &reason)
{
  stop_requested_.store(true);
  active_.store(false);
  bool notify = false;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!core_.faulted()) {
      core_.stop_on_error(reason);
      notify = true;
    }
    std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
    faulted_.store(true);
  }
  if (notify) {
    notify_fault({reason, std::nullopt});
  }
}

bool CanTransport::try_set_commands(const std::vector<double> &values)
{
  if (values.size() != pending_cmds_.size() || !active_.load() || stop_requested_.load()) {
    return false;
  }
  for (std::size_t motor_idx = 0; motor_idx < values.size(); ++motor_idx) {
    if (!command_is_valid(core_.motor_configs()[motor_idx], values[motor_idx])) {
      request_stop();
      return false;
    }
  }
  std::unique_lock<std::mutex> state_lock(state_mutex_, std::try_to_lock);
  if (!state_lock.owns_lock()) {
    return false;
  }
  const auto received_at = SteadyClock::now();
  for (std::size_t motor_idx = 0; motor_idx < values.size(); ++motor_idx) {
    pending_cmds_[motor_idx] = {values[motor_idx], received_at};
  }
  return true;
}

bool CanTransport::set_command(std::size_t motor_idx, double value)
{
  if (motor_idx >= pending_cmds_.size() || shutdown_started_.load()) {
    return false;
  }
  if (!command_is_valid(core_.motor_configs()[motor_idx], value)) {
    stop_on_error(core_.motor_configs()[motor_idx].name + ": invalid command");
    return false;
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (core_.faulted() || reboot_idx_ || SteadyClock::now() < rx_after_[motor_idx] ||
      (!opts_.auto_activate && (!active_.load() || stop_requested_.load()))) {
    return false;
  }
  pending_cmds_[motor_idx] = {value, SteadyClock::now()};
  return true;
}

bool CanTransport::try_snapshot(std::vector<MotorState> &states)
{
  if (states.size() != core_.motor_configs().size()) {
    return false;
  }
  std::unique_lock<std::mutex> state_lock(state_mutex_, std::try_to_lock);
  if (!state_lock.owns_lock()) {
    return false;
  }
  std::copy(core_.states().begin(), core_.states().end(), states.begin());
  return true;
}

std::vector<MotorState> CanTransport::snapshot()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return core_.states();
}

std::string CanTransport::fault_reason()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return core_.fault_reason();
}

void CanTransport::send_commands()
{
  std::string new_fault;
  std::optional<can_msgs::msg::Frame> reboot_frame;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const auto now = SteadyClock::now();
    if (reboot_idx_) {
      const auto idx = *reboot_idx_;
      reboot_frame =
        protocol::make_reboot_frame(core_.motor_configs()[idx].controller_id, opts_.host_id);
      core_.reset(idx);
      fault_requests_[idx].reset();
      // Discard queued pre-reboot feedback while the VESC restarts.
      rx_after_[idx] = now + std::chrono::milliseconds(500);
      std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
      fault_requested_.store(false);
      reboot_idx_.reset();
    }
    bool fresh_command = false;
    for (std::size_t idx = 0; idx < pending_cmds_.size(); ++idx) {
      auto &cmd = pending_cmds_[idx];
      if (cmd.received_at &&
          !feedback_fresh(cmd.received_at, now, core_.motor_configs()[idx].command_timeout)) {
        cmd.received_at.reset();
      }
      fresh_command = fresh_command || cmd.received_at.has_value();
    }
    if (opts_.auto_activate && !core_.active() && !core_.faulted() && !fault_requested_.load() &&
        fresh_command && core_.activate(now)) {
      stop_requested_.store(false);
    }
    const bool was_faulted = core_.faulted();
    if (stop_requested_.load()) {
      if (fault_requested_.exchange(false)) {
        if (!was_faulted) {
          core_.stop_on_error("Stop requested by hardware/topic command validation");
        }
        std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
      }
    } else {
      for (std::size_t motor_idx = 0; motor_idx < pending_cmds_.size(); ++motor_idx) {
        auto &cmd = pending_cmds_[motor_idx];
        if (cmd.received_at) {
          core_.set_command(motor_idx, cmd.value, *cmd.received_at);
          cmd.received_at.reset();
        }
      }
    }
    core_.command_frames(now, cmd_frames_);
    active_.store(core_.active());
    faulted_.store(core_.faulted());
    if (!was_faulted && core_.faulted()) {
      new_fault = core_.fault_reason();
    }
  }
  publish_frames();
  if (reboot_frame) {
    reboot_frame->header.stamp = can_node_->now();
    tx_pub_->publish(*reboot_frame);
  }
  if (!new_fault.empty()) {
    notify_fault({new_fault, std::nullopt});
  }
}

void CanTransport::poll_faults()
{
  const auto &configs = core_.motor_configs();
  for (std::size_t idx = 0; idx < configs.size(); ++idx) {
    if (SteadyClock::now() < rx_after_[idx]) {
      continue;
    }
    auto frame = protocol::make_fault_request(configs[idx].controller_id, opts_.host_id);
    fault_requests_[idx] = SteadyClock::now();
    frame.header.stamp = can_node_->now();
    tx_pub_->publish(frame);
  }
}

void CanTransport::receive_frame(const can_msgs::msg::Frame &frame)
{
  if (frame.is_error) {
    stop_on_error("CAN error frame received: id=" + std::to_string(frame.id));
    return;
  }
  const auto now = SteadyClock::now();
  protocol::FaultCode device_fault;
  std::optional<DriverFault> event;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (protocol::decode_fault_reply(frame, opts_.host_id, device_fault)) {
      const auto &configs = core_.motor_configs();
      const auto cfg =
        std::find_if(configs.begin(), configs.end(), [&device_fault](const MotorConfig &motor) {
          return motor.controller_id == device_fault.controller_id;
        });
      if (cfg == configs.end()) {
        return;
      }
      const auto idx = static_cast<std::size_t>(cfg - configs.begin());
      if (now < rx_after_[idx] || !feedback_fresh(fault_requests_[idx], now, opts_.fault_period)) {
        return;  // Ignore unsolicited, duplicate or late replies.
      }
      fault_requests_[idx].reset();
      const auto previous_code = core_.states()[idx].fault_code;
      core_.receive_fault(device_fault, now);
      if (device_fault.code != 0) {
        stop_requested_.store(true);
        active_.store(false);
        faulted_.store(true);
        std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
        if (previous_code != device_fault.code) {
          event = DriverFault{
            cfg->name + ": VESC fault_code=" + std::to_string(device_fault.code), device_fault};
        }
      }
    } else {
      const auto &configs = core_.motor_configs();
      const auto cfg =
        std::find_if(configs.begin(), configs.end(), [&frame](const MotorConfig &motor) {
          return motor.controller_id == (frame.id & protocol::kControllerIdMask);
        });
      if (cfg != configs.end() &&
          now < rx_after_[static_cast<std::size_t>(cfg - configs.begin())]) {
        return;
      }
      core_.receive(frame, now);
    }
  }
  feedback_cv_.notify_all();
  if (event) {
    notify_fault(*event);
  }
}

void CanTransport::notify_fault(const DriverFault &fault)
{
  RCLCPP_ERROR(can_node_->get_logger(), "%s", fault.reason.c_str());
  if (!on_fault_) {
    return;
  }
  try {
    on_fault_(fault);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(can_node_->get_logger(), "Fault notification failed: %s", error.what());
  }
}

void CanTransport::publish_frames()
{
  for (auto &frame : cmd_frames_) {
    // A concurrent deactivate must not leave a previously prepared motion command in flight.
    if (stop_requested_.load()) {
      frame = protocol::make_set_current_frame(
        static_cast<std::uint8_t>(frame.id & protocol::kControllerIdMask), 0.0);
    }
    frame.header.stamp = can_node_->now();
    tx_pub_->publish(frame);
  }
}
}  // namespace vesc_can_driver
