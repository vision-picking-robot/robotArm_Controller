#include <DRFLEx.h>
#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// Exported by dsr_hardware2. Use only with a single Doosan arm in this manager.
extern void * get_drfl(const char * robot_name);

namespace warehouse_gripper
{
namespace
{
using Frame = std::vector<uint8_t>;
using Trigger = std_srvs::srv::Trigger;
constexpr uint8_t slave_id = 1;
constexpr uint8_t torque_register = 0;  // Register 256, low byte
constexpr uint8_t current_register = 19;  // Register 275, low byte
constexpr uint8_t goal_register = 26;  // Register 282, low byte
constexpr uint8_t feedback_register = 29;  // Register 285, low byte

uint16_t crc16(const Frame & frame, size_t size)
{
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; ++i) {
    crc ^= frame[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
  }
  return crc;
}

class FlangeSession
{
public:
  FlangeSession(DRAFramework::CDRFLEx & robot, int port) : robot_(robot), port_(port)
  {
    if (!robot_.flange_serial_open(port_, 57600)) {
      throw std::runtime_error("Could not open flange serial port " + std::to_string(port_));
    }
  }

  ~FlangeSession() { robot_.flange_serial_close(port_); }

  Frame exchange(Frame request, size_t expected_size)
  {
    const auto crc = crc16(request, request.size());
    request.push_back(static_cast<uint8_t>(crc));
    request.push_back(static_cast<uint8_t>(crc >> 8));

    // Discard any late reply from a previous transaction.
    for (int i = 0; i < 8; ++i) {
      const auto stale = robot_.flange_serial_read(0.02f, port_);
      if (!stale || stale->_iSize <= 0) {
        break;
      }
      if (i == 7) {
        throw std::runtime_error("Flange receive buffer did not clear");
      }
    }

    if (!robot_.flange_serial_write(
        static_cast<int>(request.size()), reinterpret_cast<char *>(request.data()), port_))
    {
      throw std::runtime_error("Flange serial write failed");
    }

    Frame reply;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto chunk = robot_.flange_serial_read(0.05f, port_);
      if (!chunk || chunk->_iSize <= 0) {
        continue;
      }
      if (static_cast<size_t>(chunk->_iSize) > sizeof(chunk->_cRxd)) {
        throw std::runtime_error("Invalid flange serial reply size");
      }
      reply.insert(reply.end(), chunk->_cRxd, chunk->_cRxd + chunk->_iSize);
      const bool exception = reply.size() >= 2 && reply[1] == (request[1] | 0x80);
      const size_t frame_size = exception ? 5 : expected_size;
      if (reply.size() < frame_size) {
        continue;
      }
      const auto received_crc = reply[frame_size - 2] | (reply[frame_size - 1] << 8);
      if (reply.size() != frame_size || reply[0] != slave_id ||
        crc16(reply, frame_size - 2) != received_crc)
      {
        throw std::runtime_error("Invalid Modbus RTU reply (size, slave ID or CRC)");
      }
      if (exception) {
        throw std::runtime_error("Modbus exception " + std::to_string(reply[2]));
      }
      if (reply[1] != request[1] ||
        (request[1] == 3 ? reply[2] != frame_size - 5 :
        !std::equal(reply.begin(), reply.begin() + 6, request.begin())))
      {
        throw std::runtime_error("Modbus RTU reply does not match request");
      }
      return reply;
    }
    throw std::runtime_error("Timed out waiting for the gripper Modbus RTU reply");
  }

private:
  DRAFramework::CDRFLEx & robot_;
  int port_;
};

std::string read_status(FlangeSession & serial)
{
  const auto reply = serial.exchange({slave_id, 3, 1, feedback_register, 0, 7}, 19);
  const auto word = [&reply](size_t index) -> uint32_t {
    return (reply[3 + 2 * index] << 8) | reply[4 + 2 * index];
  };
  const auto position = word(5) | (word(6) << 16);
  const auto signed_position = static_cast<int64_t>(position) -
    ((position & 0x80000000u) ? 0x100000000LL : 0);
  const int current = static_cast<int>(word(2)) - ((word(2) & 0x8000) ? 65536 : 0);
  return "position_pulses=" + std::to_string(signed_position) +
    " current=" + std::to_string(current) + " moving=" + std::to_string(word(0));
}
}  // namespace

class GripperController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<int>("flange_port", 1);
    auto_declare<bool>("enable_motion", false);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const auto port = get_node()->get_parameter("flange_port").as_int();
    if (port != 1 && port != 2) {
      RCLCPP_ERROR(get_node()->get_logger(), "flange_port must be 1 or 2");
      return controller_interface::CallbackReturn::ERROR;
    }
    port_ = static_cast<int>(port);
    status_ = make_service("status", -1);
    open_ = make_service("open", 0);
    close_ = make_service("close", 740);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    status_.reset();
    open_.reset();
    close_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return controller_interface::return_type::OK;
  }

private:
  rclcpp::Service<Trigger>::SharedPtr make_service(const std::string & name, int target)
  {
    return get_node()->create_service<Trigger>(
      "~/" + name,
      [this, target](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock || !active_) {
          response->message = "Gripper controller is inactive or busy";
          return;
        }
        try {
          auto robot = static_cast<DRAFramework::CDRFLEx *>(get_drfl(nullptr));
          if (!robot || robot->get_robot_system() != ROBOT_SYSTEM_REAL) {
            throw std::runtime_error("An active real Doosan connection is required");
          }
          if (target >= 0 &&
            (!get_node()->get_parameter("enable_motion").as_bool() ||
            robot->get_robot_state() != STATE_STANDBY))
          {
            throw std::runtime_error("Motion requires enable_motion=true and robot STANDBY");
          }

          FlangeSession serial(*robot, port_);
          if (target < 0) {
            response->message = read_status(serial);
          } else {
            read_status(serial);
            serial.exchange({slave_id, 6, 1, current_register, 1, 144}, 8);  // 400 mA
            serial.exchange({
              slave_id, 16, 1, goal_register, 0, 2, 4,
              static_cast<uint8_t>(target >> 8), static_cast<uint8_t>(target), 0, 0}, 8);
            // Set the new goal before enabling torque to avoid executing an old goal.
            serial.exchange({slave_id, 6, 1, torque_register, 0, 1}, 8);
            response->message = "Target acknowledged; use status to verify movement";
          }
          response->success = true;
        } catch (const std::exception & error) {
          response->message = error.what();
        }
      });
  }

  std::mutex mutex_;
  bool active_ = false;
  int port_ = 1;
  rclcpp::Service<Trigger>::SharedPtr status_, open_, close_;
};
}  // namespace warehouse_gripper

PLUGINLIB_EXPORT_CLASS(warehouse_gripper::GripperController, controller_interface::ControllerInterface)
