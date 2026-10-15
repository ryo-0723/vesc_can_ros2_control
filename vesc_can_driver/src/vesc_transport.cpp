#include "vesc_can_driver/vesc_transport.hpp"

#include <algorithm>
#include <bitset>
#include <stdexcept>
#include <utility>
namespace vesc_can_driver
{
// VESC transport: executor, latest commands and periodic transmission.
VescTransport::VescTransport(std::vector<MotorConfig> configs, TransportOptions opts,
  rclcpp::Node::SharedPtr io_node, std::unique_ptr<VescInterface> interface,
  std::function<void(const DriverFault &)> on_fault)
  : core_(std::move(configs)), opts_(std::move(opts)), io_node_(std::move(io_node)),
    on_fault_(std::move(on_fault)), interface_(std::move(interface)),
    rx_after_(core_.motors().size()), pending_cmds_(core_.motors().size())
{
  if (!io_node_ || opts_.send_period.count() <= 0 || !interface_) {
    throw std::invalid_argument("Invalid VESC transport node, interface or period");
  }
  if (opts_.fault_period.count() < 0) {
    throw std::invalid_argument("Fault query period must be non-negative");
  }
  std::vector<std::uint8_t> ids;
  for (const auto &motor : core_.motors()) {
    ids.push_back(motor.config().controller_id);
  }
  interface_->configure_targets(ids);
  outputs_.reserve(core_.motors().size());
  fault_commands_.reserve(core_.motors().size());
  send_timer_ = io_node_->create_wall_timer(opts_.send_period, [this]() { send_commands(); });
  if (opts_.fault_period.count() > 0) {
    fault_timer_ = io_node_->create_wall_timer(opts_.fault_period, [this]() { poll_faults(); });
  }
  io_exec_.add_node(io_node_);
}

VescTransport::~VescTransport()
{
  shutdown();
}

void VescTransport::start()
{
  if (shutdown_started_.load()) {
    throw std::logic_error("Transport has been shut down");
  }
  if (exec_thread_.joinable()) {
    throw std::logic_error("Transport already started");
  }
  interface_->start([this](const VescFeedback &feedback) { receive_feedback(feedback); },
    [this](const std::string &reason) { stop_on_error(reason); });
  exec_thread_ = std::thread([this]() {
    try {
      io_exec_.spin();
    } catch (const std::exception &error) {
      stop_on_error(std::string("VESC executor failed: ") + error.what());
      RCLCPP_ERROR(io_node_->get_logger(), "VESC executor failed: %s", error.what());
    }
  });
}

void VescTransport::shutdown() noexcept
{
  if (shutdown_started_.exchange(true)) {
    return;
  }
  stop_requested_.store(true);
  active_.store(false);
  io_exec_.cancel();
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
  try {
    if (fault_requested_.exchange(false)) {
      stop_on_error("Stop requested by hardware/topic command validation");
    }
    if (rclcpp::ok(io_node_->get_node_base_interface()->get_context())) {
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        core_.stop_commands(outputs_);
      }
      send_outputs();
    }
  } catch (const std::exception &) {
    // Destruction cannot guarantee network delivery; MCU/VESC watchdogs remain necessary.
  }
  interface_->stop();
  feedback_cv_.notify_all();
}

bool VescTransport::wait_ready(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  const bool condition_met = feedback_cv_.wait_for(state_lock, timeout,
    [this]() { return shutdown_started_.load() || core_.ready(SteadyClock::now()); });
  return condition_met && !shutdown_started_.load();
}

bool VescTransport::activate()
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
  faulted_.store(core_.is_faulted());
  stop_requested_.store(!active);
  return active;
}

void VescTransport::deactivate()
{
  stop_requested_.store(true);
  active_.store(false);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  core_.deactivate();
  std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
}

bool VescTransport::reboot(std::size_t motor_idx)
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

void VescTransport::stop_on_error(const std::string &reason)
{
  stop_requested_.store(true);
  active_.store(false);
  bool notify = false;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (!core_.is_faulted()) {
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

bool VescTransport::try_set_commands(const std::vector<double> &values)
{
  if (values.size() != pending_cmds_.size() || !active_.load() || stop_requested_.load()) {
    return false;
  }
  for (std::size_t motor_idx = 0; motor_idx < values.size(); ++motor_idx) {
    if (!command_is_valid(core_.motors()[motor_idx].config(), values[motor_idx])) {
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

bool VescTransport::set_commands(const std::vector<MotorCommand> &cmds)
{
  if (shutdown_started_.load() || cmds.empty()) {
    return false;
  }
  const auto received_at = SteadyClock::now();
  std::bitset<256> seen;
  for (const auto &cmd : cmds) {
    if (cmd.motor_idx >= pending_cmds_.size()) {
      stop_on_error("Unknown motor index: " + std::to_string(cmd.motor_idx));
      return false;
    }
    const auto &cfg = core_.motors()[cmd.motor_idx].config();
    if (seen.test(cmd.motor_idx)) {
      stop_on_error(cfg.name + ": duplicate motor index");
      return false;
    }
    if (!command_is_valid(cfg, cmd.value)) {
      stop_on_error(cfg.name + ": invalid command value " + std::to_string(cmd.value));
      return false;
    }
    seen.set(cmd.motor_idx);
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  if (shutdown_started_.load() || core_.is_faulted() || reboot_idx_ ||
      (!opts_.auto_activate && (!active_.load() || stop_requested_.load()))) {
    return false;
  }
  const auto now = SteadyClock::now();
  if (std::any_of(cmds.begin(), cmds.end(),
        [&](const MotorCommand &cmd) { return now < rx_after_[cmd.motor_idx]; })) {
    return false;
  }
  for (const auto &cmd : cmds) {
    pending_cmds_[cmd.motor_idx] = {cmd.value, received_at};
  }
  return true;
}

bool VescTransport::try_snapshot(std::vector<MotorState> &states)
{
  if (states.size() != core_.motors().size()) {
    return false;
  }
  std::unique_lock<std::mutex> state_lock(state_mutex_, std::try_to_lock);
  if (!state_lock.owns_lock()) {
    return false;
  }
  std::transform(core_.motors().begin(), core_.motors().end(), states.begin(),
    [](const MotorController &motor) { return motor.state(); });
  return true;
}

std::vector<MotorState> VescTransport::snapshot()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  std::vector<MotorState> states(core_.motors().size());
  std::transform(core_.motors().begin(), core_.motors().end(), states.begin(),
    [](const MotorController &motor) { return motor.state(); });
  return states;
}

std::string VescTransport::fault_reason()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return core_.fault_reason();
}

void VescTransport::send_commands()
{
  std::string new_fault;
  std::optional<VescCommand> reboot_command;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const auto now = SteadyClock::now();
    if (reboot_idx_) {
      const auto idx = *reboot_idx_;
      reboot_command =
        VescCommand{core_.motors()[idx].config().controller_id, VescCommandType::Reboot, 0.0};
      core_.reset(idx);
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
          !feedback_fresh(cmd.received_at, now, core_.motors()[idx].config().command_timeout)) {
        cmd.received_at.reset();
      }
      fresh_command = fresh_command || cmd.received_at.has_value();
    }
    if (opts_.auto_activate && !core_.is_active() && !core_.is_faulted() &&
        !fault_requested_.load() && fresh_command && core_.activate(now)) {
      stop_requested_.store(false);
    }
    const bool was_faulted = core_.is_faulted();
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
    core_.commands(now, outputs_);
    active_.store(core_.is_active());
    faulted_.store(core_.is_faulted());
    if (!was_faulted && core_.is_faulted()) {
      new_fault = core_.fault_reason();
    }
  }
  send_outputs();
  if (reboot_command) {
    interface_->send(*reboot_command);
  }
  if (!new_fault.empty()) {
    notify_fault({new_fault, std::nullopt});
  }
}

void VescTransport::poll_faults()
{
  fault_commands_.clear();
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const auto now = SteadyClock::now();
    const auto &motors = core_.motors();
    for (std::size_t idx = 0; idx < motors.size(); ++idx) {
      if (now < rx_after_[idx]) {
        continue;
      }
      fault_commands_.push_back(
        {motors[idx].config().controller_id, VescCommandType::FaultRequest, 0.0});
    }
  }
  for (const auto &request : fault_commands_) {
    interface_->send(request);
  }
}

void VescTransport::receive_feedback(const VescFeedback &feedback)
{
  const auto now = SteadyClock::now();
  const auto controller_id =
    std::visit([](const auto &value) { return value.controller_id; }, feedback);
  const auto *device_fault = std::get_if<FaultCode>(&feedback);
  std::optional<DriverFault> event;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const auto &motors = core_.motors();
    const auto motor = std::find_if(motors.begin(), motors.end(),
      [controller_id](const auto &motor) { return motor.config().controller_id == controller_id; });
    if (motor == motors.end()) {
      return;
    }
    const auto idx = static_cast<std::size_t>(motor - motors.begin());
    if (now < rx_after_[idx]) {
      return;
    }
    if (device_fault) {
      const auto previous_code = motor->state().fault_code;
      core_.receive_fault(*device_fault, now);
      if (device_fault->code != 0) {
        stop_requested_.store(true);
        active_.store(false);
        faulted_.store(true);
        std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
        if (previous_code != device_fault->code) {
          event = DriverFault{
            motor->config().name + ": VESC fault_code=" + std::to_string(device_fault->code),
            *device_fault};
        }
      }
    } else {
      core_.receive(feedback, now);
    }
  }
  feedback_cv_.notify_all();
  if (event) {
    notify_fault(*event);
  }
}

void VescTransport::notify_fault(const DriverFault &fault)
{
  RCLCPP_ERROR(io_node_->get_logger(), "%s", fault.reason.c_str());
  if (!on_fault_) {
    return;
  }
  try {
    on_fault_(fault);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(io_node_->get_logger(), "Fault notification failed: %s", error.what());
  }
}

void VescTransport::send_outputs()
{
  for (auto command : outputs_) {
    if (stop_requested_.load()) {
      command = {command.controller_id, VescCommandType::Current, 0.0};
    }
    interface_->send(command);
  }
}
}  // namespace vesc_can_driver
