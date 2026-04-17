#include "KinovaRobot.h"
#include "ActuatorConfigClientRpc.h"
#include "ActuatorCyclic.pb.h"
#include "ActuatorCyclicClientRpc.h"
#include "Base.pb.h"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cstdint>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <mc_rtc/DataStore.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <stdexcept>

namespace mc_kinova {

KinovaRobot::KinovaRobot(const std::string &name, const std::string &ip_address,
                         const std::string &username = "admin",
                         const std::string &password = "admin",
                         bool debug_enabled)
    : m_name(name), m_ip_address(ip_address), m_port(10000),
      m_port_real_time(10001), m_username(username), m_password(password),
      stop_controller(false), m_debug_enabled(debug_enabled),
      m_control_ready(false), m_command_ready(false) {
  m_router = nullptr;
  m_router_real_time = nullptr;
  m_transport = nullptr;
  m_transport_real_time = nullptr;
  m_session_manager = nullptr;
  m_session_manager_real_time = nullptr;
  m_base = nullptr;
  m_base_cyclic = nullptr;
  m_device_manager = nullptr;
  m_device_config = nullptr;
  m_actuator_config = nullptr;
  gripper_enabled = false;
  if (m_name.find("gripper") != std::string::npos) {
    mc_rtc::log::info("[MC_KORTEX] Gripper enabled for robot: {}", m_name);
    gripper_enabled = true;
  }
  m_state = k_api::BaseCyclic::Feedback();
  m_servoing_mode = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
  m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
  m_control_id = 0;
  m_prev_control_id = -1;
  m_control_mode_id = 0;
  m_prev_control_mode_id = 0;
  m_bypass_command_id = 0;
  m_low_level_interface_type = LowLevelInterfaceType::Base;
  m_torque_control_type = mc_kinova::TorqueControlType::Default;
  m_gripper_motor_command = nullptr;
  m_interconnect_gripper_motor_command = nullptr;
  m_debug_bypass_gripper_command_logged = false;
  m_debug_bypass_gripper_feedback_logged = false;
}

KinovaRobot::~KinovaRobot() {
  clearBypassDeviceClients();

  // Close API session
  m_session_manager->CloseSession();
  m_session_manager_real_time->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  m_router->SetActivationStatus(false);
  m_transport->disconnect();
  m_router_real_time->SetActivationStatus(false);
  m_transport_real_time->disconnect();

  // Destroy the API
  delete m_actuator_config;
  delete m_device_manager;
  delete m_device_config;
  delete m_session_manager_real_time;
  delete m_session_manager;
  delete m_base_cyclic;
  delete m_base;
  delete m_router_real_time;
  delete m_router;
  delete m_transport_real_time;
  delete m_transport;
}

// ==================== Getter ==================== //

std::vector<double> KinovaRobot::getJointPosition() {
  std::vector<double> q(m_actuator_count);
  for (auto actuator : m_state.actuators())
    q[jointIdFromCommandID(actuator.command_id())] = actuator.position();
  return q;
}

std::string KinovaRobot::getName(void) { return m_name; }

bool KinovaRobot::controlReady() const { return m_control_ready.load(); }

// ==================== Setter ==================== //

void KinovaRobot::debugLog(const std::string &msg) {
  if (m_debug_enabled) {
    mc_rtc::log::info("[MC_KORTEX][DEBUG][{}] {}", m_name, msg);
  }
}

std::string KinovaRobot::ipv4ToString(uint32_t ipv4) const {
  return fmt::format("{}.{}.{}.{}", (ipv4 >> 24) & 0xFF, (ipv4 >> 16) & 0xFF,
                     (ipv4 >> 8) & 0xFF, ipv4 & 0xFF);
}

void KinovaRobot::debugDiscoverDevices() {
  if (!m_debug_enabled || m_device_manager == nullptr ||
      m_device_config == nullptr) {
    return;
  }

  try {
    auto devices = m_device_manager->ReadAllDevices();
    debugLog(fmt::format("DeviceManager returned {} devices",
                         devices.device_handle_size()));

    for (int i = 0; i < devices.device_handle_size(); ++i) {
      const auto &handle = devices.device_handle(i);
      debugLog(fmt::format("Device {}: type={} id={} order={}", i,
                           static_cast<int>(handle.device_type()),
                           handle.device_identifier(), handle.order()));

      try {
        auto ipv4 =
            m_device_config->GetIPv4Settings(handle.device_identifier());
        debugLog(fmt::format("Device {} IPv4: address={} mask={} gateway={}", i,
                             ipv4ToString(ipv4.ipv4_address()),
                             ipv4ToString(ipv4.ipv4_subnet_mask()),
                             ipv4ToString(ipv4.ipv4_default_gateway())));
      } catch (const k_api::KDetailedException &ex) {
        debugLog(fmt::format("Device {} IPv4 query failed: {}", i, ex.what()));
      }
    }
  } catch (const k_api::KDetailedException &ex) {
    debugLog(fmt::format("Device discovery failed: {}", ex.what()));
  }
}

void KinovaRobot::clearBypassDeviceClients() {
  for (auto &client : m_bypass_actuator_clients) {
    delete client.actuator_config;
    delete client.actuator_cyclic;
    if (client.router != nullptr) {
      client.router->SetActivationStatus(false);
    }
    if (client.transport != nullptr) {
      client.transport->disconnect();
    }
    delete client.router;
    delete client.transport;
    client.actuator_config = nullptr;
    client.actuator_cyclic = nullptr;
    client.router = nullptr;
    client.transport = nullptr;
  }
  m_bypass_actuator_clients.clear();

  delete m_bypass_interconnect_client.interconnect_cyclic;
  if (m_bypass_interconnect_client.router != nullptr) {
    m_bypass_interconnect_client.router->SetActivationStatus(false);
  }
  if (m_bypass_interconnect_client.transport != nullptr) {
    m_bypass_interconnect_client.transport->disconnect();
  }
  delete m_bypass_interconnect_client.router;
  delete m_bypass_interconnect_client.transport;
  m_bypass_interconnect_client = BypassInterconnectClient();
}

k_api::ActuatorConfig::ActuatorConfigClient *
KinovaRobot::actuatorConfigClient(size_t joint_idx) {
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass &&
      joint_idx < m_bypass_actuator_clients.size()) {
    return m_bypass_actuator_clients[joint_idx].actuator_config;
  }
  return m_actuator_config;
}

k_api::ActuatorCyclic::ActuatorCyclicClient *
KinovaRobot::actuatorCyclicClient(size_t joint_idx) {
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass &&
      joint_idx < m_bypass_actuator_clients.size()) {
    return m_bypass_actuator_clients[joint_idx].actuator_cyclic;
  }
  return nullptr;
}

k_api::InterconnectCyclic::InterconnectCyclicClient *
KinovaRobot::interconnectCyclicClient() {
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    return m_bypass_interconnect_client.interconnect_cyclic;
  }
  return nullptr;
}

void KinovaRobot::setActuatorControlMode(
    size_t joint_idx,
    const k_api::ActuatorConfig::ControlModeInformation &control_mode) {
  auto *client = actuatorConfigClient(joint_idx);
  if (client == nullptr) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] missing actuator config client for joint {}",
        joint_idx + 1);
  }
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    client->SetControlMode(control_mode);
  } else {
    client->SetControlMode(control_mode, joint_idx + 1);
  }
}

void KinovaRobot::setActuatorCommandMode(
    size_t joint_idx,
    const k_api::ActuatorConfig::CommandModeInformation &command_mode) {
  auto *client = actuatorConfigClient(joint_idx);
  if (client == nullptr) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] missing actuator config client for joint {}",
        joint_idx + 1);
  }
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    client->SetCommandMode(command_mode);
  } else {
    client->SetCommandMode(command_mode, joint_idx + 1);
  }
}

void KinovaRobot::setActuatorServoing(
    size_t joint_idx, const k_api::ActuatorConfig::Servoing &servoing) {
  auto *client = actuatorConfigClient(joint_idx);
  if (client == nullptr) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] missing actuator config client for joint {}",
        joint_idx + 1);
  }
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    client->SetServoing(servoing);
  } else {
    client->SetServoing(servoing, joint_idx + 1);
  }
}

void KinovaRobot::clearActuatorFaults(size_t joint_idx) {
  auto *client = actuatorConfigClient(joint_idx);
  if (client == nullptr) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] missing actuator config client for joint {}",
        joint_idx + 1);
  }
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    client->ClearFaults();
  } else {
    client->ClearFaults(joint_idx + 1);
  }
}

bool KinovaRobot::hasValidCommand(const mc_rbdyn::Robot &robot) const {
  auto rjo = robot.refJointOrder();
  if (rjo.size() < static_cast<size_t>(m_actuator_count)) {
    return false;
  }

  for (int i = 0; i < m_actuator_count; ++i) {
    const auto joint_index = robot.jointIndexByName(rjo[i]);
    if (joint_index < 0 ||
        joint_index >= static_cast<int>(m_command.q.size()) ||
        m_command.q[joint_index].empty()) {
      return false;
    }

    if (m_control_mode != k_api::ActuatorConfig::ControlMode::POSITION) {
      if (joint_index >= static_cast<int>(m_command.alphaD.size()) ||
          m_command.alphaD[joint_index].empty() ||
          joint_index >= static_cast<int>(m_command.jointTorque.size()) ||
          m_command.jointTorque[joint_index].empty()) {
        return false;
      }
    }
  }

  return true;
}

void KinovaRobot::createBypassDeviceClients() {
  clearBypassDeviceClients();

  auto error_callback = [](k_api::KError err) {
    mc_rtc::log::error("_________ callback error _________ {}", err.toString());
  };

  std::vector<BypassActuatorClient> discovered_actuators;
  BypassInterconnectClient discovered_interconnect;

  auto devices = m_device_manager->ReadAllDevices();
  for (int i = 0; i < devices.device_handle_size(); ++i) {
    const auto &handle = devices.device_handle(i);

    k_api::DeviceConfig::IPv4Settings ipv4;
    try {
      ipv4 = m_device_config->GetIPv4Settings(handle.device_identifier());
    } catch (const k_api::KDetailedException &) {
      continue;
    }

    const auto ip_address = ipv4ToString(ipv4.ipv4_address());
    const auto device_type = handle.device_type();

    if (device_type == k_api::Common::BIG_ACTUATOR ||
        device_type == k_api::Common::SMALL_ACTUATOR) {
      BypassActuatorClient client;
      client.ip_address = ip_address;
      client.device_id = handle.device_identifier();
      client.order = handle.order();
      discovered_actuators.push_back(client);
    } else if (device_type == k_api::Common::INTERCONNECT) {
      discovered_interconnect.ip_address = ip_address;
      discovered_interconnect.device_id = handle.device_identifier();
      discovered_interconnect.order = handle.order();
    }
  }

  std::sort(
      discovered_actuators.begin(), discovered_actuators.end(),
      [](const BypassActuatorClient &lhs, const BypassActuatorClient &rhs) {
        return lhs.order < rhs.order;
      });

  if (discovered_actuators.size() != static_cast<size_t>(m_actuator_count)) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] expected {} actuator endpoints for {}, found {}",
        m_actuator_count, m_name, discovered_actuators.size());
  }

  for (size_t i = 0; i < discovered_actuators.size(); ++i) {
    auto &client = discovered_actuators[i];
    client.transport = new k_api::TransportClientUdp();
    client.transport->connect(client.ip_address, m_port);
    client.router = new k_api::RouterClient(client.transport, error_callback);
    client.actuator_config =
        new k_api::ActuatorConfig::ActuatorConfigClient(client.router);
    client.actuator_cyclic =
        new k_api::ActuatorCyclic::ActuatorCyclicClient(client.router);
    debugLog(fmt::format(
        "Created direct bypass actuator client {}: id={} order={} ip={}", i + 1,
        client.device_id, client.order, client.ip_address));
  }

  if (gripper_enabled) {
    if (discovered_interconnect.ip_address.empty()) {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[MC_KORTEX] gripper bypass requested for {} but no interconnect "
          "endpoint was discovered",
          m_name);
    }

    discovered_interconnect.transport = new k_api::TransportClientUdp();
    discovered_interconnect.transport->connect(
        discovered_interconnect.ip_address, m_port);
    discovered_interconnect.router = new k_api::RouterClient(
        discovered_interconnect.transport, error_callback);
    discovered_interconnect.interconnect_cyclic =
        new k_api::InterconnectCyclic::InterconnectCyclicClient(
            discovered_interconnect.router);
    debugLog(fmt::format(
        "Created direct bypass interconnect client: id={} order={} ip={}",
        discovered_interconnect.device_id, discovered_interconnect.order,
        discovered_interconnect.ip_address));
  }

  m_bypass_actuator_clients = std::move(discovered_actuators);
  m_bypass_interconnect_client = std::move(discovered_interconnect);
}

bool KinovaRobot::validateServoingMode(k_api::Base::ServoingMode mode) {
  try {
    return m_base->GetServoingMode().servoing_mode() == mode;
  } catch (const k_api::KDetailedException &ex) {
    printException(const_cast<k_api::KDetailedException &>(ex));
    return false;
  }
}

void KinovaRobot::setLowServoingMode() {
  // Ignore if already in low level servoing mode
  // if(m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
  // return;

  auto servoingMode = k_api::Base::ServoingModeInformation();

  switch (m_low_level_interface_type) {
  case LowLevelInterfaceType::Base:
    m_servoing_mode = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
    break;
  case LowLevelInterfaceType::Bypass:
    m_servoing_mode = k_api::Base::ServoingMode::BYPASS_SERVOING;
    break;
  default:
    m_servoing_mode = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
    break;
  }

  servoingMode.set_servoing_mode(m_servoing_mode);
  debugLog(fmt::format("Requesting low-level servoing mode {}",
                       static_cast<int>(m_servoing_mode)));
  m_base->SetServoingMode(servoingMode);
  if (!validateServoingMode(m_servoing_mode)) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] failed to switch {} to servoing mode {}", m_name,
        static_cast<int>(m_servoing_mode));
  }
}

void KinovaRobot::setSingleServoingMode() {
  // Ignore if already in "high" level servoing mode
  // if(m_servoing_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
  // return;

  auto servoingMode = k_api::Base::ServoingModeInformation();

  servoingMode.set_servoing_mode(
      k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  debugLog("Requesting single-level servoing mode");
  m_base->SetServoingMode(servoingMode);
  if (!validateServoingMode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] failed to switch {} to single-level servoing", m_name);
  }
  m_servoing_mode = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
}

void KinovaRobot::setCustomTorque(mc_rtc::Configuration &torque_config) {
  if (torque_config.has("friction_compensation")) {
    if (torque_config("friction_compensation").has("stiction")) {
      m_stiction_values = torque_config("friction_compensation")("stiction");
      if (not(m_stiction_values.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"compensation_values\" key "
            "does not match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    } else {
      m_stiction_values = {3.0, 3.0, 3.0, 3.0, 1.25, 1.25, 1.25};
    }
    if (torque_config("friction_compensation").has("coulomb")) {
      m_friction_values = torque_config("friction_compensation")("coulomb");
      if (not(m_friction_values.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"compensation_values\" key "
            "does not match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    } else {
      m_friction_values = {3.0, 3.0, 3.0, 3.0, 1.25, 1.25, 1.25};
    }
    if (torque_config("friction_compensation").has("viscous")) {
      m_viscous_values = torque_config("friction_compensation")("viscous");
      if (not(m_viscous_values.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"compensation_values\" key "
            "does not match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    } else {
      m_viscous_values = {2.416, 2.416, 2.416, 2.416, 1.1, 1.1, 1.1};
    }

    if (torque_config("friction_compensation").has("velocity_threshold")) {
      m_friction_vel_threshold =
          torque_config("friction_compensation")("velocity_threshold");
    } else {
      m_friction_vel_threshold = 0.01;
    }

    if (torque_config("friction_compensation").has("acceleration_threshold")) {
      m_friction_accel_threshold =
          torque_config("friction_compensation")("acceleration_threshold");
    } else {
      m_friction_accel_threshold = 100;
    }

    if (torque_config.has("lambda")) {
      m_lambda = torque_config("lambda");
    } else {
      m_lambda = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    }
  } else {
    m_friction_vel_threshold = 0.01;
    m_friction_accel_threshold = 100;
    m_friction_values = {3.0, 3.0, 3.0, 3.0, 1.25, 1.25, 1.25};
    m_viscous_values = {2.416, 2.416, 2.416, 2.416, 1.1, 1.1, 1.1};
  }

  if (torque_config.has("integral_term")) {
    if (torque_config("integral_term").has("theta")) {
      m_integral_slow_theta = torque_config("integral_term")("theta");
    } else {
      m_integral_slow_theta = 0.1;
    }

    if (torque_config("integral_term").has("gain")) {
      m_integral_slow_gain = torque_config("integral_term")("gain");
    } else {
      m_integral_slow_gain = 1e-3;
    }
  } else {
    m_integral_slow_theta = 0.1;
    m_integral_slow_gain = 1e-3;
  }

  mc_rtc::log::info(
      "[MC_KORTEX] {} robot is using custom torque control with parameters:",
      m_name);
}

void KinovaRobot::setControlMode(std::string mode) {
  if (mode.compare("Position") == 0) {
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION)
      return;

    // mc_rtc::log::info("[MC_KORTEX] Using position control");
    m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
    m_control_mode_id++;
    return;
  }
  if (mode.compare("Velocity") == 0) {
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::VELOCITY)
      return;

    // mc_rtc::log::info("[MC_KORTEX] Using velocity control");
    m_control_mode = k_api::ActuatorConfig::ControlMode::VELOCITY;
    m_control_mode_id++;
    return;
  }
  if (mode.compare("Torque") == 0) {
    switch (m_torque_control_type) {
    case mc_kinova::TorqueControlType::Default:
    case mc_kinova::TorqueControlType::Feedforward:
    case mc_kinova::TorqueControlType::Custom:
      if (m_control_mode == k_api::ActuatorConfig::ControlMode::CURRENT)
        return;

      // mc_rtc::log::info("[MC_KORTEX] Using torque control");
      initFiltersBuffers();
      m_control_mode = k_api::ActuatorConfig::ControlMode::CURRENT;
      m_control_mode_id++;
      return;
      break;
    }
  }
}

void KinovaRobot::setTorqueMode(std::string mode) {
  if (mode.compare("Default") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Default;
  } else if (mode.compare("Feedforward") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Feedforward;
  } else if (mode.compare("Custom") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Custom;
  } else {
    mc_rtc::log::error("[MC_KORTEX] Unknown torque control type: {}", mode);
  }
}

// ==================== Public functions ==================== //

void KinovaRobot::init(mc_control::MCGlobalController &gc,
                       mc_rtc::Configuration &kortexConfig) {
  mc_rtc::log::info("[MC_KORTEX] Initializing connection to the robot at {}:{}",
                    m_ip_address, m_port);

  auto error_callback = [](k_api::KError err) {
    mc_rtc::log::error("_________ callback error _________ {}", err.toString());
  };

  if (kortexConfig.has("low_level_type")) {
    std::string low_level_type = kortexConfig("low_level_type");
    if (low_level_type.compare("base") == 0) {
      m_low_level_interface_type = LowLevelInterfaceType::Base;
    } else if (low_level_type.compare("bypass") == 0) {
      m_low_level_interface_type = LowLevelInterfaceType::Bypass;
    } else {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[MC_KORTEX] tried to set \"low_level_type\" received incorect "
          "value: \"{}\". Expected values are: [base, bypass]",
          low_level_type);
    }
  }

  // Initiate connection
  m_transport = new k_api::TransportClientTcp();
  m_router = new k_api::RouterClient(m_transport, error_callback);
  m_transport->connect(m_ip_address, m_port);

  m_transport_real_time = new k_api::TransportClientUdp();
  m_router_real_time =
      new k_api::RouterClient(m_transport_real_time, error_callback);
  m_transport_real_time->connect(m_ip_address, m_port_real_time);

  // Set session data connection information
  auto createSessionInfo = k_api::Session::CreateSessionInfo();
  createSessionInfo.set_username(m_username);
  createSessionInfo.set_password(m_password);
  createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
  createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  m_session_manager = new k_api::SessionManager(m_router);
  m_session_manager->CreateSession(createSessionInfo);
  m_session_manager_real_time = new k_api::SessionManager(m_router_real_time);
  m_session_manager_real_time->CreateSession(createSessionInfo);

  // Create services
  m_device_manager = new k_api::DeviceManager::DeviceManagerClient(m_router);
  m_device_config = new k_api::DeviceConfig::DeviceConfigClient(m_router);
  m_base = new k_api::Base::BaseClient(m_router);
  m_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(m_router);
  if (m_low_level_interface_type == LowLevelInterfaceType::Base) {
    m_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(m_router_real_time);
  } else if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    // Direct bypass device clients are created after device discovery.
  } else {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[MC_KORTEX] Incorect low-level interface type was expecting either "
        "LowLevelInterfaceType::Base or LowLevelInterfaceType::Bypass");
  }

  // Read actuators count
  setSingleServoingMode();
  m_actuator_count = m_base->GetActuatorCount().count();
  gripper_idx = 0;
  if (gripper_enabled) {
    gripper_idx = m_actuator_count;
  }
  mc_rtc::log::info("[MC_KORTEX] {} robot has {} actuators", m_name,
                    m_actuator_count + (gripper_enabled ? 1 : 0));
  m_debug_bypass_init_logged.assign(m_actuator_count, false);
  m_debug_bypass_command_logged.assign(m_actuator_count, false);
  m_debug_bypass_feedback_logged.assign(m_actuator_count, false);

  m_filter_command.assign(m_actuator_count, 0.0);
  m_filter_command_w_gain.assign(m_actuator_count, 0.0);
  m_current_command.setZero(m_actuator_count);
  m_current_measurement.setZero(m_actuator_count);
  m_torque_from_current_measurement.setZero(m_actuator_count);
  m_torque_measure_corrected.assign(m_actuator_count, 0.0);
  m_tau_sensor.setZero(m_actuator_count);
  m_torque_error.assign(m_actuator_count, 0.0);
  m_prev_torque_error.assign(m_actuator_count, 0.0);
  m_integral_slow_filter.assign(m_actuator_count, 0.0);
  m_integral_slow_filter_w_gain.assign(m_actuator_count, 0.0);
  m_integral_slow_bound.assign(m_actuator_count, 0.0);
  m_friction_compensation_mode.assign(m_actuator_count, 0.0);
  m_current_friction_compensation.assign(m_actuator_count, 0.0);
  m_jac_transpose_f.assign(m_actuator_count, 0.0);
  m_offsets.assign(m_actuator_count, 0.0);
  tau_fric.setZero(m_actuator_count);
  m_lambda.assign(m_actuator_count, 0.0);
  m_integral_slow_theta = 1.0;
  m_integral_slow_gain = 1e-2;
  m_actuator_feedback_vec.assign(m_actuator_count,
                                 k_api::ActuatorCyclic::Feedback());

  debugDiscoverDevices();

  // Set control mode
  auto controle_mode = k_api::ActuatorConfig::ControlModeInformation();
  controle_mode.set_control_mode(m_control_mode);
  if (m_low_level_interface_type == LowLevelInterfaceType::Base) {
    for (int i = 0; i < m_actuator_count; i++) {
      setActuatorControlMode(i, controle_mode);
    }
  } else {
    debugLog(
        "Deferring direct actuator control-mode setup until bypass startup");
  }

  // Init pose if desired
  if (kortexConfig.has("init_posture")) {
    if (kortexConfig("init_posture").has("posture")) {
      m_init_posture = kortexConfig("init_posture")("posture");
      if (not(m_init_posture.size() == m_actuator_count))
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] for {} robot, value for \"posture\" key does not "
            "match actuators count.\nActuators count = ",
            m_name, m_actuator_count);
    }

    if (kortexConfig("init_posture")("on_startup", false)) {
      moveToInitPosition();
    }
  } else {
    m_init_posture.resize(m_actuator_count);

    auto joints_feedback = m_base->GetMeasuredJointAngles();
    for (size_t i = 0; i < m_actuator_count; i++) {
      auto joint_feedback = joints_feedback.joint_angles(i);
      m_init_posture[joint_feedback.joint_identifier() - 1] =
          joint_feedback.value();
    }
  }

  if (gripper_enabled) {
    gripper_position = 0.0;
    gripper_velocity = 0.0;
    m_base_command.mutable_interconnect()->mutable_command_id()->set_identifier(
        0);
    m_gripper_motor_command = m_base_command.mutable_interconnect()
                                  ->mutable_gripper_command()
                                  ->add_motor_cmd();
    m_gripper_motor_command->set_position(0.0);
    m_gripper_motor_command->set_velocity(0.0);
    m_gripper_motor_command->set_force(100.0);
    m_interconnect_command.mutable_command_id()->set_identifier(0);
    m_interconnect_gripper_motor_command =
        m_interconnect_command.mutable_gripper_command()->add_motor_cmd();
    m_interconnect_gripper_motor_command->set_position(0.0);
    m_interconnect_gripper_motor_command->set_velocity(0.0);
    m_interconnect_gripper_motor_command->set_force(100.0);
    debugLog("Prepared interconnect cyclic gripper command path");
  }

  // Initialize state
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    initializeBypass();
    debugLog("Bypass initialization completed during robot init");
    updateSensors(gc);
  } else {
    updateState();
    updateSensors(gc);
  }

  // Velocity filtering init
  m_use_filtered_velocities = kortexConfig.has("filter_velocity");
  if (m_use_filtered_velocities) {
    m_velocity_filter_ratio = kortexConfig("filter_velocity")("ratio", 0.0);
    mc_rtc::log::info(
        "[MC_KORTEX] Filtering velocities for {} robot with {} ratio", m_name,
        m_velocity_filter_ratio);
  }
  m_filtered_velocities.assign(m_actuator_count, 0.0);

  // Custom torque control init
  if (kortexConfig.has("torque_control")) {
    kortexConfig = kortexConfig("torque_control");
    if (!kortexConfig.has("mode"))
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[MC_KORTEX] For {} robot, \"torque_control\" key found in config "
          "file but \"mode\" key is missing.",
          m_name);

    std::string controle_mode = kortexConfig("mode");
    if (controle_mode.compare("feedforward") == 0) {
      m_torque_control_type = mc_kinova::TorqueControlType::Feedforward;
      mc_rtc::log::info(
          "[MC_KORTEX] Using feedforward only for torque control");
    } else if (controle_mode.compare("custom") == 0) {
      m_torque_control_type = mc_kinova::TorqueControlType::Custom;
      setCustomTorque(kortexConfig);
      mc_rtc::log::info("[MC_KORTEX] Using custom control for torque control");
    } else {
      m_torque_control_type = mc_kinova::TorqueControlType::Default;
      mc_rtc::log::info(
          "[MC_KORTEX] Using Kinova's default control for torque control");
    }
  }

  gc.controller().datastore().make_call(
      "set_kinova_friction_compensation_stiction",
      [this](std::vector<double> v) { m_stiction_values = v; });
  gc.controller().datastore().make_call(
      "set_kinova_friction_compensation_coulomb",
      [this](std::vector<double> v) { m_friction_values = v; });
  gc.controller().datastore().make_call(
      "set_kinova_friction_compensation_viscous",
      [this](std::vector<double> v) { m_viscous_values = v; });
  gc.controller().datastore().make_call(
      "set_kinova_integral_term_gain",
      [this](double g) { m_integral_slow_gain = g; });

  // Initialize Jacobian object
  auto robot = &gc.robots().robot(m_name);

  Eigen::VectorXd tu = rbd::paramToVector(robot->mb(), robot->tu());
  // Initialize each actuator to its current position
  for (int i = 0; i < m_actuator_count; i++) {
    m_integral_slow_bound[i] = 0.05 * tu[i]; // 5% of torque limit
    const double initial_position =
        (m_low_level_interface_type == LowLevelInterfaceType::Bypass)
            ? m_actuator_feedback_vec[i].position()
            : m_state.actuators(i).position();
    m_base_command.add_actuators()->set_position(initial_position);
  }

  addGui(gc);

  mc_rtc::log::success("[MC_KORTEX] Connected succesfuly to robot at {}:{}",
                       m_ip_address, m_port);
}

void KinovaRobot::addLogEntry(mc_control::MCGlobalController &gc) {
  gc.controller().logger().addLogEntry("kortex_LoopPerf",
                                       [&, this]() { return m_dt; });
  if (m_torque_control_type == mc_kinova::TorqueControlType::Feedforward) {
    gc.controller().logger().addLogEntry(
        "kortex_commanded_current", [this]() { return m_current_command; });
    gc.controller().logger().addLogEntry(
        "kortex_current_measurement",
        [this]() { return m_torque_from_current_measurement; });
  }

  if (m_torque_control_type == mc_kinova::TorqueControlType::Custom) {
    gc.controller().logger().addLogEntry(
        "tauInCorrected", [this]() { return m_torque_measure_corrected; });

    gc.controller().logger().addLogEntry(
        "kortex_friction_velocity_threshold",
        [this]() { return m_friction_vel_threshold; });
    gc.controller().logger().addLogEntry(
        "kortex_friction_acceleration_threshold",
        [this]() { return m_friction_accel_threshold; });
    gc.controller().logger().addLogEntry(
        "kortex_friction_coulomb_values",
        [this]() { return m_friction_values; });
    gc.controller().logger().addLogEntry("kortex_friction_viscous_values",
                                         [this]() { return m_viscous_values; });
    gc.controller().logger().addLogEntry("kortex_friction_mode", [this]() {
      return m_friction_compensation_mode;
    });
    gc.controller().logger().addLogEntry("torque friction",
                                         [this]() { return tau_fric; });
    gc.controller().logger().addLogEntry(
        "kortex_friction_current_compensation",
        [this]() { return m_current_friction_compensation; });

    gc.controller().logger().addLogEntry("kortex_torque_error",
                                         [this]() { return m_torque_error; });
    gc.controller().logger().addLogEntry(
        "kortex_integral_value", [this]() { return m_integral_slow_filter; });
    gc.controller().logger().addLogEntry(
        "kortex_integral_gains", [this]() { return m_integral_slow_gain; });
    gc.controller().logger().addLogEntry(
        "kortex_integral_theta", [this]() { return m_integral_slow_theta; });
    gc.controller().logger().addLogEntry("kortex_integral_w_gain", [this]() {
      return m_integral_slow_filter_w_gain;
    });

    gc.controller().logger().addLogEntry("kortex_transfer_function",
                                         [this]() { return m_filter_command; });
    gc.controller().logger().addLogEntry(
        "kortex_transfer_w_gain", [this]() { return m_filter_command_w_gain; });

    gc.controller().logger().addLogEntry(
        "kortex_current_command", [this]() { return m_current_command; });
    gc.controller().logger().addLogEntry(
        "kortex_current_measurement",
        [this]() { return m_torque_from_current_measurement; });

    gc.controller().logger().addLogEntry(
        "kortex_jac_transpose_F", [this]() { return m_jac_transpose_f; });
    gc.controller().logger().addLogEntry("kortex_posture_task_offset",
                                         [this]() { return m_offsets; });
    gc.controller().logger().addLogEntry("kortex_lambda",
                                         [this]() { return m_lambda; });
  }
}

void KinovaRobot::removeLogEntry(mc_control::MCGlobalController &gc) {
  gc.controller().logger().removeLogEntry("kortex_LoopPerf");
  if (m_torque_control_type == mc_kinova::TorqueControlType::Custom) {
    gc.controller().logger().removeLogEntry(
        "kortex_friction_velocity_threshold");
    gc.controller().logger().removeLogEntry(
        "kortex_friction_acceleration_threshold");
    gc.controller().logger().removeLogEntry(
        "kortex_friction_compensation_values");
    gc.controller().logger().removeLogEntry("kortex_friction_mode");
    gc.controller().logger().removeLogEntry("kortex_torque_error");
    gc.controller().logger().removeLogEntry("kortex_integral_term");
    gc.controller().logger().removeLogEntry(
        "kortex_integral_fast_filtered_integral");
    gc.controller().logger().removeLogEntry(
        "kortex_integral_slow_filtered_integral");
    gc.controller().logger().removeLogEntry("kortex_integral_mixed_term");
    gc.controller().logger().removeLogEntry("kortex_integral_gains");
    gc.controller().logger().removeLogEntry("kortex_integral_mu");
    gc.controller().logger().removeLogEntry("kortex_current_output");
    gc.controller().logger().removeLogEntry("kortex_current_measurement");
    gc.controller().logger().removeLogEntry("kortex_jac_transpose_F");
    gc.controller().logger().removeLogEntry("kortex_posture_task_offset");
  }
}

void KinovaRobot::updateState() {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    auto message_id = k_api::ActuatorCyclic::MessageId();
    message_id.set_identifier(0);
    for (int i = 0; i < m_actuator_count; i++) {
      auto *client = actuatorCyclicClient(i);
      if (client == nullptr) {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] missing bypass actuator cyclic client for joint {}",
            i + 1);
      }
      m_actuator_feedback_vec[i] = client->RefreshFeedback(message_id);
    }
    if (gripper_enabled && interconnectCyclicClient() != nullptr) {
      auto interconnect_message_id = k_api::InterconnectCyclic::MessageId();
      interconnect_message_id.set_identifier(0);
      m_interconnect_feedback =
          interconnectCyclicClient()->RefreshFeedback(interconnect_message_id);
    }
  } else {
    m_state = m_base_cyclic->RefreshFeedback();
    for (int i = 0; i < m_actuator_count; i++) {
      m_actuator_feedback_vec[i].set_position(
          m_state.mutable_actuators(i)->position());
      m_actuator_feedback_vec[i].set_velocity(
          m_state.mutable_actuators(i)->velocity());
      m_actuator_feedback_vec[i].set_torque(
          m_state.mutable_actuators(i)->torque());
      m_actuator_feedback_vec[i].set_current_motor(
          m_state.mutable_actuators(i)->current_motor());
    }
  }
}

void KinovaRobot::updateStateBase(const k_api::BaseCyclic::Feedback data) {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  m_state = data;
}

void KinovaRobot::torqueFrictionComputation(
    mc_rbdyn::Robot &robot, const rbd::MultiBodyConfig &command,
    k_api::BaseCyclic::Feedback m_state_local, double joint_idx) {
  auto rjo = robot.refJointOrder();

  double velocity = mc_rtc::constants::toRad(
      m_state_local.mutable_actuators(joint_idx)->velocity());
  double friction_torque = 0.0;
  auto qdd_r = command.alphaD[robot.jointIndexByName(rjo[joint_idx])][0];

  // Friction compensation logic
  if (velocity > m_friction_vel_threshold) {
    friction_torque =
        m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else if (velocity < -m_friction_vel_threshold) {
    friction_torque =
        -m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else {
    if (qdd_r > m_friction_accel_threshold) {
      friction_torque = m_friction_values[joint_idx];
    } else if (qdd_r < -m_friction_accel_threshold) {
      friction_torque = -m_friction_values[joint_idx];
    }
  }
  tau_fric[joint_idx] = friction_torque;
}

void KinovaRobot::updateStateBypass(const k_api::ActuatorCyclic::Feedback data,
                                    size_t joint_idx) {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  if (joint_idx < m_actuator_feedback_vec.size()) {
    m_actuator_feedback_vec[joint_idx] = data;
  }
}

void KinovaRobot::updateBypassGripperState(
    const k_api::InterconnectCyclic::Feedback &data) {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  m_interconnect_feedback = data;
  if (m_debug_enabled && !m_debug_bypass_gripper_feedback_logged &&
      data.has_gripper_feedback() && data.gripper_feedback().motor_size() > 0) {
    debugLog(fmt::format("First bypass gripper feedback: pos={} vel={}",
                         data.gripper_feedback().motor(0).position(),
                         data.gripper_feedback().motor(0).velocity()));
    m_debug_bypass_gripper_feedback_logged = true;
  }
}

void KinovaRobot::initializeBypass() {
  debugLog("Initializing bypass mode");
  setLowServoingMode();
  debugLog("Low-level bypass servoing mode validated");
  debugLog("Creating direct bypass device clients after servoing switch");
  createBypassDeviceClients();

  auto message_id = k_api::ActuatorCyclic::MessageId();
  message_id.set_identifier(0);
  auto command_mode = k_api::ActuatorConfig::CommandModeInformation();
  command_mode.set_command_mode(k_api::ActuatorConfig::CYCLIC);
  auto servoing = k_api::ActuatorConfig::Servoing();
  servoing.set_enabled(true);

  for (int i = 0; i < m_actuator_count; i++) {
    auto *cyclic_client = actuatorCyclicClient(i);
    auto *config_client = actuatorConfigClient(i);
    if (cyclic_client == nullptr || config_client == nullptr) {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[MC_KORTEX] missing bypass client setup for actuator {}", i + 1);
    }

    debugLog(
        fmt::format("Bypass actuator {}: requesting initial feedback", i + 1));
    auto feedback = cyclic_client->RefreshFeedback(message_id);
    updateStateBypass(feedback, i);
    if (m_debug_enabled && !m_debug_bypass_init_logged[i]) {
      debugLog(fmt::format("Bypass actuator {} init feedback: pos={} vel={} "
                           "torque={} current={}",
                           i + 1, feedback.position(), feedback.velocity(),
                           feedback.torque(), feedback.current_motor()));
    }

    if (feedback.fault_bank_a() != 0 || feedback.fault_bank_b() != 0) {
      mc_rtc::log::warning(
          "[MC_KORTEX] actuator {} has pre-existing faults, clearing before "
          "bypass startup",
          i + 1);
    }

    debugLog(fmt::format("Bypass actuator {}: clearing faults", i + 1));
    clearActuatorFaults(i);
    debugLog(fmt::format("Bypass actuator {}: setting command mode to CYCLIC",
                         i + 1));
    setActuatorCommandMode(i, command_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    debugLog(fmt::format(
        "Bypass actuator {}: requesting feedback after command-mode switch",
        i + 1));
    feedback = cyclic_client->RefreshFeedback(message_id);
    updateStateBypass(feedback, i);
    debugLog(fmt::format("Bypass actuator {}: post-command-mode feedback "
                         "status={} faults=({}, {})",
                         i + 1, feedback.status_flags(),
                         feedback.fault_bank_a(), feedback.fault_bank_b()));

    auto command = k_api::ActuatorCyclic::Command();
    command.mutable_command_id()->set_identifier(0);
    command.set_flags(1);
    command.set_position(feedback.position());
    command.set_velocity(0.0);
    command.set_current_motor(feedback.current_motor());
    debugLog(fmt::format(
        "Bypass actuator {}: priming command prepared before servoing enable",
        i + 1));

    debugLog(fmt::format("Bypass actuator {}: enabling servoing", i + 1));
    bool servoing_enabled = false;
    for (int attempt = 0; attempt < 5 && !servoing_enabled; ++attempt) {
      try {
        setActuatorServoing(i, servoing);
        servoing_enabled = true;
      } catch (const k_api::KDetailedException &ex) {
        debugLog(
            fmt::format("Bypass actuator {}: SetServoing attempt {} failed: {}",
                        i + 1, attempt + 1, ex.what()));
        try {
          feedback = cyclic_client->RefreshFeedback(message_id);
          updateStateBypass(feedback, i);
          debugLog(
              fmt::format("Bypass actuator {}: feedback after SetServoing "
                          "failure status={} faults=({}, {}) pos={} vel={}",
                          i + 1, feedback.status_flags(),
                          feedback.fault_bank_a(), feedback.fault_bank_b(),
                          feedback.position(), feedback.velocity()));
        } catch (const k_api::KDetailedException &feedback_ex) {
          debugLog(fmt::format("Bypass actuator {}: feedback refresh after "
                               "SetServoing failure also failed: {}",
                               i + 1, feedback_ex.what()));
        }
        if (attempt == 4) {
          throw;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    debugLog(fmt::format("Bypass actuator {}: sending priming cyclic command",
                         i + 1));
    cyclic_client->Refresh(command);
    if (m_debug_enabled && !m_debug_bypass_init_logged[i]) {
      debugLog(fmt::format("Bypass actuator {} initialized: "
                           "command_mode=CYCLIC servoing=enabled",
                           i + 1));
      m_debug_bypass_init_logged[i] = true;
    }
  }

  m_bypass_command_id = 1;

  if (gripper_enabled && interconnectCyclicClient() != nullptr) {
    auto interconnect_message_id = k_api::InterconnectCyclic::MessageId();
    interconnect_message_id.set_identifier(0);
    debugLog("Bypass interconnect: requesting initial feedback");
    m_interconnect_feedback =
        interconnectCyclicClient()->RefreshFeedback(interconnect_message_id);
    debugLog("Initialized bypass interconnect feedback path");
  }
}

void KinovaRobot::updateGripperCommand(mc_rbdyn::Robot &robot) {
  if (!gripper_enabled) {
    return;
  }

  float gripper_target = robot.gripper("gripper").q()[0] * 100.0;
  float gripper_velocity_target = fabs(gripper_target - gripper_position) * 2.2;
  if (gripper_velocity_target > 100.0) {
    gripper_velocity_target = 100.0;
  }

  if (m_servoing_mode == k_api::Base::ServoingMode::BYPASS_SERVOING &&
      m_interconnect_gripper_motor_command != nullptr) {
    m_interconnect_command.mutable_command_id()->set_identifier(
        m_bypass_command_id);
    m_interconnect_gripper_motor_command->set_position(gripper_target);
    m_interconnect_gripper_motor_command->set_velocity(gripper_velocity_target);
    if (m_debug_enabled && !m_debug_bypass_gripper_command_logged) {
      debugLog(fmt::format(
          "First bypass gripper command: cmd_id={} pos={} vel={}",
          m_bypass_command_id, gripper_target, gripper_velocity_target));
      m_debug_bypass_gripper_command_logged = true;
    }
    return;
  }

  m_base_command.mutable_interconnect()->mutable_command_id()->set_identifier(
      0);
  m_gripper_motor_command->set_position(gripper_target);
  m_gripper_motor_command->set_velocity(gripper_velocity_target);
}

double KinovaRobot::currentTorqueControlLaw(mc_rbdyn::Robot &robot,
                                            const rbd::MultiBodyConfig &command,
                                            double measured_torque,
                                            double velocity, double joint_idx) {

  auto rjo = robot.refJointOrder();

  double torque_constant = (joint_idx > 3) ? 0.076 : 0.11;
  auto filter_input = m_filter_input_buffer[joint_idx];
  auto filter_output = m_filter_output_buffer[joint_idx];
  double friction_torque = 0.0;

  auto qdd_r = command.alphaD[robot.jointIndexByName(rjo[joint_idx])][0];

  double tau_desired =
      command.jointTorque[robot.jointIndexByName(rjo[joint_idx])][0];

  double rotor_inertia =
      robot.mb().joint(robot.jointIndexByName(rjo[joint_idx])).rotorInertia();

  double rotor_inertia_torque = rotor_inertia * GEAR_RATIO * GEAR_RATIO * qdd_r;

  double torque_error = tau_desired + measured_torque - rotor_inertia_torque;

  m_prev_torque_error[joint_idx] = m_torque_error[joint_idx];
  m_torque_error[joint_idx] = torque_error;

  // Friction compensation logic
  if (velocity > m_friction_vel_threshold) {
    m_friction_compensation_mode[joint_idx] = 2;
    friction_torque =
        m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else if (velocity < -m_friction_vel_threshold) {
    m_friction_compensation_mode[joint_idx] = -2;
    friction_torque =
        -m_friction_values[joint_idx] + m_viscous_values[joint_idx] * velocity;
  } else {
    if (qdd_r > m_friction_accel_threshold) {
      m_friction_compensation_mode[joint_idx] = 1;
      friction_torque = m_stiction_values[joint_idx];
    } else if (qdd_r < -m_friction_accel_threshold) {
      m_friction_compensation_mode[joint_idx] = -1;
      friction_torque = -m_stiction_values[joint_idx];
    }
  }

  m_integral_slow_filter[joint_idx] =
      exp(-(1e-3 / m_integral_slow_theta)) *
          (m_integral_slow_filter_w_gain[joint_idx] / m_integral_slow_gain) +
      (1 - exp(-(1e-3 / m_integral_slow_theta))) * torque_error;

  double integral_w_gain =
      m_integral_slow_gain * m_integral_slow_filter[joint_idx];

  m_current_friction_compensation[joint_idx] = friction_torque;

  m_integral_slow_filter_w_gain[joint_idx] =
      std::max(-m_integral_slow_bound[joint_idx],
               std::min(integral_w_gain, m_integral_slow_bound[joint_idx]));

  // Filtered command calculation
  m_filter_command[joint_idx] =
      1.975063 * filter_output[0] - 0.9751799 * filter_output[1] +
      0.02017482 * (torque_error)-0.03697504 * filter_input[0] +
      0.01691718 * filter_input[1];

  // Update filter buffers
  m_filter_input_buffer[joint_idx].push_front(torque_error);
  m_filter_output_buffer[joint_idx].push_front(m_filter_command[joint_idx]);
  m_filter_command_w_gain[joint_idx] = m_filter_command[joint_idx];
  // Current calculation
  double current =
      (m_lambda[joint_idx] * m_filter_command[joint_idx] + tau_desired +
       m_integral_slow_filter_w_gain[joint_idx] + friction_torque) /
      (GEAR_RATIO * torque_constant);

  m_current_command[joint_idx] = current * torque_constant * GEAR_RATIO;

  return current;
}

bool KinovaRobot::sendCommandBase(mc_rbdyn::Robot &robot, bool &running) {
  bool return_value = true;
  k_api::BaseCyclic::Feedback m_state_local;
  rbd::MultiBodyConfig command_local;
  auto control_mode_local = m_control_mode;
  auto torque_control_type_local = m_torque_control_type;
  int control_id_local = 0;
  int control_mode_id_local = 0;
  {
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    m_state_local = m_state;
  }

  {
    std::unique_lock<std::mutex> lock(m_update_control_mutex);
    command_local = m_command;
    control_mode_local = m_control_mode;
    torque_control_type_local = m_torque_control_type;
    control_id_local = m_control_id;
    control_mode_id_local = m_control_mode_id;
  }
  auto rjo = robot.refJointOrder();

  auto base_callback = [&, this](const Kinova::Api::Error &err,
                                 const k_api::BaseCyclic::Feedback data) {
    updateStateBase(data);
    checkBaseFaultBanks(data.base().fault_bank_a(), data.base().fault_bank_b());
    if (err.error_code() != k_api::ErrorCodes::ERROR_NONE) {
      printError(err);
      running = false;
    }
  };

  for (size_t i = 0; i < m_actuator_count; i++) {
    torqueFrictionComputation(robot, command_local, m_state_local, i);
    double kt = (i > 3) ? 0.076 : 0.11;
    if (control_mode_local == k_api::ActuatorConfig::ControlMode::POSITION) {
      m_base_command.mutable_actuators(i)->set_position(radToJointPose(
          i, command_local.q[robot.jointIndexByName(rjo[i])][0]));
      m_base_command.mutable_actuators(i)->set_current_motor(
          m_state_local.mutable_actuators(i)->current_motor());
      continue;
    } else {
      m_base_command.mutable_actuators(i)->set_position(
          m_state_local.mutable_actuators(i)->position());
    }

    auto rjo = robot.refJointOrder();

    auto qdd_r = command_local.alphaD[robot.jointIndexByName(rjo[i])][0];

    double tau_desired =
        command_local.jointTorque[robot.jointIndexByName(rjo[i])][0];

    double rotor_inertia =
        robot.mb().joint(robot.jointIndexByName(rjo[i])).rotorInertia();

    double rotor_inertia_torque =
        rotor_inertia * GEAR_RATIO * GEAR_RATIO * qdd_r;

    m_torque_measure_corrected[i] =
        -m_state_local.mutable_actuators(i)->torque() + rotor_inertia_torque;

    switch (torque_control_type_local) {
    case mc_kinova::TorqueControlType::Default:
      m_base_command.mutable_actuators(i)->set_torque_joint(
          command_local.jointTorque[robot.jointIndexByName(rjo[i])][0] -
          rotor_inertia_torque);
      break;
    case mc_kinova::TorqueControlType::Feedforward:
      m_base_command.mutable_actuators(i)->set_current_motor(
          command_local.jointTorque[robot.jointIndexByName(rjo[i])][0] /
          (GEAR_RATIO * kt));
      m_current_command(i) =
          command_local.jointTorque[robot.jointIndexByName(rjo[i])][0] /
          (GEAR_RATIO * kt);
      break;
    case mc_kinova::TorqueControlType::Custom:
      m_base_command.mutable_actuators(i)->set_current_motor(
          currentTorqueControlLaw(
              robot, command_local,
              m_state_local.mutable_actuators(i)->torque(),
              mc_rtc::constants::toRad(
                  m_state_local.mutable_actuators(i)->velocity()),
              i));
      break;
    default:
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[MC_KORTEX] wrong torque control type when trying to send command");
      break;
    }
    // std::cout << m_base_command.mutable_actuators(i)->position() << " " <<
    // m_state_local.mutable_actuators(i)->position() << " | ";
  }

  if (gripper_enabled) {
    updateGripperCommand(robot);
  }

  // if (m_control_mode != k_api::ActuatorConfig::ControlMode::POSITION)
  // std::cout << std::endl;

  // ========================= Control mode has changed in mc_rtc, change it for
  // the robot ========================= //
  if (control_mode_id_local != m_prev_control_mode_id) {
    auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
    control_mode.set_control_mode(control_mode_local);

    try {
      mc_rtc::log::info("[MC_KORTEX] Changing robot control mode to {} ",
                        control_mode_local);
      for (int i = 0; i < m_actuator_count; i++) {
        setActuatorControlMode(i, control_mode);
      }
      m_prev_control_mode_id = control_mode_id_local;
    } catch (k_api::KDetailedException &ex) {
      printException(ex);
      return_value = false;
      running = false;
    }
  }

  try {
    m_base_cyclic->Refresh_callback(m_base_command, base_callback, 0);
    return_value = true;
  } catch (k_api::KDetailedException &ex) {
    printException(ex);
    return_value = false;
    running = false;
  }

  m_prev_control_id = control_id_local;
  return return_value;
}

bool KinovaRobot::sendCommandBypass(mc_rbdyn::Robot &robot, bool &running) {
  bool return_value = true;
  rbd::MultiBodyConfig command_local;
  auto control_mode_local = m_control_mode;
  auto torque_control_type_local = m_torque_control_type;
  int control_id_local = 0;
  int control_mode_id_local = 0;
  std::vector<k_api::ActuatorCyclic::Feedback> m_actuators_feeback_local_vec(
      m_actuator_count);
  {
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    for (int i = 0; i < m_actuators_feeback_local_vec.size(); i++) {
      m_actuators_feeback_local_vec[i] = m_actuator_feedback_vec[i];
    }
  }

  {
    std::unique_lock<std::mutex> lock(m_update_control_mutex);
    command_local = m_command;
    control_mode_local = m_control_mode;
    torque_control_type_local = m_torque_control_type;
    control_id_local = m_control_id;
    control_mode_id_local = m_control_mode_id;
  }
  auto rjo = robot.refJointOrder();

  // ========================= Control mode has changed in mc_rtc, change it for
  // the robot ========================= //
  if (control_mode_id_local != m_prev_control_mode_id) {
    auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
    control_mode.set_control_mode(control_mode_local);

    try {
      mc_rtc::log::info("[MC_KORTEX] Changing robot control mode to {} ",
                        control_mode_local);
      for (int i = 0; i < m_actuator_count; i++) {
        setActuatorControlMode(i, control_mode);
      }
      m_prev_control_mode_id = control_mode_id_local;
    } catch (k_api::KDetailedException &ex) {
      printException(ex);
      return_value = false;
      running = false;
    }
  }

  for (size_t i = 0; i < m_actuator_count; i++) {
    k_api::ActuatorCyclic::Command command;

    command.mutable_command_id()->set_identifier(m_bypass_command_id);
    command.set_flags(1);

    double kt = (i > 3) ? 0.076 : 0.11;
    if (control_mode_local == k_api::ActuatorConfig::ControlMode::POSITION) {
      command.set_position(radToJointPose(
          i, command_local.q[robot.jointIndexByName(rjo[i])][0]));
      command.set_velocity(0.0);
      command.set_current_motor(
          m_actuators_feeback_local_vec[i].current_motor());
    } else {
      command.set_position(m_actuators_feeback_local_vec[i].position());
      command.set_velocity(m_actuators_feeback_local_vec[i].velocity());

      auto rjo = robot.refJointOrder();

      auto qdd_r = command_local.alphaD[robot.jointIndexByName(rjo[i])][0];

      double tau_desired =
          command_local.jointTorque[robot.jointIndexByName(rjo[i])][0];

      double rotor_inertia =
          robot.mb().joint(robot.jointIndexByName(rjo[i])).rotorInertia();

      double rotor_inertia_torque =
          rotor_inertia * GEAR_RATIO * GEAR_RATIO * qdd_r;

      m_torque_measure_corrected[i] =
          -m_actuators_feeback_local_vec[i].torque() + rotor_inertia_torque;

      switch (torque_control_type_local) {
      case mc_kinova::TorqueControlType::Default:
        command.set_torque_joint(
            command_local.jointTorque[robot.jointIndexByName(rjo[i])][0] -
            rotor_inertia_torque);
        break;
      case mc_kinova::TorqueControlType::Feedforward:
        command.set_current_motor(
            command_local.jointTorque[robot.jointIndexByName(rjo[i])][0] /
            (GEAR_RATIO * kt));
        m_current_command(i) =
            command_local.jointTorque[robot.jointIndexByName(rjo[i])][0] /
            (GEAR_RATIO * kt);
        break;
      case mc_kinova::TorqueControlType::Custom:
        command.set_current_motor(currentTorqueControlLaw(
            robot, command_local, m_actuators_feeback_local_vec[i].torque(),
            mc_rtc::constants::toRad(
                m_actuators_feeback_local_vec[i].velocity()),
            i));
        break;
      default:
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] wrong torque control type when trying to send "
            "command");
        break;
      }
    }

    // std::cout << m_base_command.mutable_actuators(i)->position() << " " <<
    // m_state_local.mutable_actuators(i)->position() << " | ";

    try {
      if (m_debug_enabled && !m_debug_bypass_command_logged[i]) {
        debugLog(fmt::format("First bypass actuator {} command: cmd_id={} "
                             "pos={} vel={} torque_joint={} current_motor={}",
                             i + 1, command.command_id().identifier(),
                             command.position(), command.velocity(),
                             command.torque_joint(), command.current_motor()));
        m_debug_bypass_command_logged[i] = true;
      }
      auto *cyclic_client = actuatorCyclicClient(i);
      if (cyclic_client == nullptr) {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[MC_KORTEX] missing bypass actuator cyclic client for joint {}",
            i + 1);
      }
      auto feedback = cyclic_client->Refresh(command);
      updateStateBypass(feedback, i);
      checkActuatorFaultBanks(feedback.fault_bank_a(), feedback.fault_bank_b(),
                              i);
      if (m_debug_enabled && !m_debug_bypass_feedback_logged[i]) {
        debugLog(fmt::format("First bypass actuator {} cyclic feedback: "
                             "cmd_id={} pos={} vel={} torque={} current={}",
                             i + 1, feedback.feedback_id().identifier(),
                             feedback.position(), feedback.velocity(),
                             feedback.torque(), feedback.current_motor()));
        m_debug_bypass_feedback_logged[i] = true;
      }
      return_value = true;
    } catch (k_api::KDetailedException &ex) {
      printException(ex);
      return_value = false;
      running = false;
    }
  }

  m_bypass_command_id++;

  if (gripper_enabled) {
    updateGripperCommand(robot);
    if (interconnectCyclicClient() != nullptr) {
      try {
        auto feedback =
            interconnectCyclicClient()->Refresh(m_interconnect_command);
        updateBypassGripperState(feedback);
      } catch (k_api::KDetailedException &ex) {
        printException(ex);
        return_value = false;
        running = false;
      }
    }
  }

  m_prev_control_id = control_id_local;
  return return_value;
}

void KinovaRobot::updateSensors(mc_control::MCGlobalController &gc) {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  auto &robot = gc.controller().robots().robot(m_name);
  auto rjo = robot.refJointOrder();

  // Trick for getting continuous joint to take the shortest path to the target
  auto has_posture_task = gc.controller().datastore().has("getPostureTask");
  auto posture_task_pt =
      (has_posture_task)
          ? gc.controller().datastore().call<mc_tasks::PostureTaskPtr>(
                "getPostureTask")
          : nullptr;
  m_offsets = computePostureTaskOffset(robot, posture_task_pt);

  std::vector<double> q(m_actuator_count + (gripper_enabled ? 1 : 0));
  std::vector<double> qdot(m_actuator_count + (gripper_enabled ? 1 : 0));
  std::vector<double> tau(m_actuator_count + (gripper_enabled ? 1 : 0));
  std::map<std::string, sva::ForceVecd> wrenches;
  double fx, fy, fz, cx, cy, cz;
  std::map<std::string, double> current;
  // std::map<std::string,double> temp;

  for (size_t i = 0; i < m_actuator_count; i++) {
    double kt = (i > 3) ? 0.076 : 0.11;
    double actuator_pos = 0.0;
    double actuator_vel = 0.0;
    double actuator_torque = 0.0;
    double actuator_current = 0.0;
    if (m_servoing_mode == k_api::Base::ServoingMode::BYPASS_SERVOING) {
      actuator_pos = m_actuator_feedback_vec[i].position();
      actuator_vel = m_actuator_feedback_vec[i].velocity();
      actuator_torque = m_actuator_feedback_vec[i].torque();
      actuator_current = m_actuator_feedback_vec[i].current_motor();
    } else {
      actuator_pos = m_state.mutable_actuators(i)->position();
      actuator_vel = m_state.mutable_actuators(i)->velocity();
      actuator_torque = m_state.mutable_actuators(i)->torque();
      actuator_current = m_state.mutable_actuators(i)->current_motor();
    }
    q[i] = jointPoseToRad(i, actuator_pos) + m_offsets[i];
    ;
    if (m_use_filtered_velocities) {
      m_filtered_velocities[i] =
          m_velocity_filter_ratio * m_filtered_velocities[i] +
          (1 - m_velocity_filter_ratio) *
              mc_rtc::constants::toRad(actuator_vel);
      qdot[i] = m_filtered_velocities[i];
    } else {
      qdot[i] = mc_rtc::constants::toRad(actuator_vel);
    }
    tau[i] = -actuator_torque;
    m_tau_sensor(i) = tau[i];
    m_current_measurement(i) = actuator_current;
    m_torque_from_current_measurement(i) = actuator_current * kt * GEAR_RATIO;
    current[fmt::format("joint_{}", i + 1)] = m_current_measurement(i);
  }

  if (gripper_enabled) {
    bool has_gripper_feedback = false;
    if (m_servoing_mode == k_api::Base::ServoingMode::BYPASS_SERVOING &&
        interconnectCyclicClient() != nullptr) {
      const auto &inter = m_interconnect_feedback;
      if (inter.has_gripper_feedback() &&
          inter.gripper_feedback().motor_size() > 0) {
        gripper_position = inter.gripper_feedback().motor()[0].position();
        gripper_velocity = inter.gripper_feedback().motor()[0].velocity();
        has_gripper_feedback = true;
      }
    } else {
      m_base_command.mutable_interconnect()
          ->mutable_command_id()
          ->set_identifier(0);
      const auto &inter = m_state.interconnect();
      if (inter.has_gripper_feedback() &&
          inter.gripper_feedback().motor_size() > 0) {
        gripper_position = inter.gripper_feedback().motor()[0].position();
        gripper_velocity = inter.gripper_feedback().motor()[0].velocity();
        has_gripper_feedback = true;
      }
    }

    if (!has_gripper_feedback) {
      mc_rtc::log::warning("[MC_KORTEX] No gripper feedback available");
      gripper_velocity = 0.0;
    } else if (m_debug_enabled &&
               m_servoing_mode == k_api::Base::ServoingMode::BYPASS_SERVOING &&
               !m_debug_bypass_gripper_feedback_logged) {
      debugLog(fmt::format("Bypass gripper state in sensors: pos={} vel={}",
                           gripper_position, gripper_velocity));
      m_debug_bypass_gripper_feedback_logged = true;
    }

    q[gripper_idx] = jointPoseToRad(gripper_idx, gripper_position);
    qdot[gripper_idx] = mc_rtc::constants::toRad(gripper_velocity);
    tau[gripper_idx] = 0.0;
    // m_tau_sensor(gripper_idx) = tau[gripper_idx];
    // m_current_measurement(gripper_idx) =
    // m_state.interconnect().gripper_feedback().motor()[0].current_motor();
    // m_torque_from_current_measurement(gripper_idx) = 0.0;
    // current[rjo[gripper_idx]] = m_current_measurement(gripper_idx);
  }

  gc.setEncoderValues(m_name, q);
  gc.setEncoderVelocities(m_name, qdot);
  gc.setJointTorques(m_name, tau);
  gc.setJointMotorCurrents(m_name, current);
  // gc.setJointMotorTemperatures(m_name,temp);

  // Store the torque friction to the datastore
  if (!gc.controller().datastore().has("torque_fric")) {
    gc.controller().datastore().make<Eigen::VectorXd>("torque_fric", tau_fric);
  } else {
    gc.controller().datastore().assign("torque_fric", tau_fric);
  }
}

void KinovaRobot::updateControl(mc_control::MCGlobalController &controller) {
  std::unique_lock<std::mutex> lock(m_update_control_mutex);
  auto &robot = controller.controller().robots().robot(m_name);
  m_command = robot.mbc();
  m_control_id++;
  const bool command_ready = hasValidCommand(robot);
  if (m_debug_enabled && !m_command_ready.load() && command_ready) {
    debugLog("First structurally valid controller command received");
  }
  m_command_ready = command_ready;
}

std::string KinovaRobot::controlLoopParamToString(
    k_api::ActuatorConfig::LoopSelection &loop_selected, int actuator_idx) {
  k_api::ActuatorConfig::ControlLoopParameters parameters =
      m_actuator_config->GetControlLoopParameters(loop_selected, actuator_idx);
  std::ostringstream ss;
  ss << "kAz = [";
  for (size_t i = 0; i < parameters.kaz_size() - 1; i++)
    ss << parameters.kaz(i) << ",";
  ss << parameters.kaz(parameters.kaz_size()) << "] kBz = [";
  for (size_t i = 0; i < parameters.kbz_size() - 1; i++)
    ss << parameters.kbz(i) << ",";
  ss << parameters.kbz(parameters.kbz_size()) << "]";

  return ss.str();
}

void KinovaRobot::checkBaseFaultBanks(uint32_t fault_bank_a,
                                      uint32_t fault_bank_b) {
  if (fault_bank_a != 0) {
    auto error_list = getBaseFaultList(fault_bank_a);
    std::ostringstream ss;
    ss << "[";
    std::copy(error_list.begin(), error_list.end() - 1,
              std::ostream_iterator<std::string>(ss, ", "));
    ss << error_list.back() << "]";
    mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank A : {}",
                                 ss.str());
  }
  if (fault_bank_b != 0) {
    auto error_list = getBaseFaultList(fault_bank_b);
    std::ostringstream ss;
    ss << "[";
    std::copy(error_list.begin(), error_list.end() - 1,
              std::ostream_iterator<std::string>(ss, ", "));
    ss << error_list.back() << "]";
    mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank B : {}",
                                 ss.str());
  }
}

void KinovaRobot::checkActuatorFaultBanks(uint32_t fault_bank_a,
                                          uint32_t fault_bank_b,
                                          size_t joint_idx) {
  if (fault_bank_a != 0) {
    auto error_list = getActuatorFaultList(fault_bank_a);
    std::ostringstream ss;
    ss << "[";
    std::copy(error_list.begin(), error_list.end() - 1,
              std::ostream_iterator<std::string>(ss, ", "));
    ss << error_list.back() << "]";
    mc_rtc::log::error_and_throw(
        "[MC_KORTEX] Error in actuator {} fault bank A : {}", joint_idx + 1,
        ss.str());
  }
  if (fault_bank_b != 0) {
    auto error_list = getActuatorFaultList(fault_bank_b);
    std::ostringstream ss;
    ss << "[";
    std::copy(error_list.begin(), error_list.end() - 1,
              std::ostream_iterator<std::string>(ss, ", "));
    ss << error_list.back() << "]";
    mc_rtc::log::error_and_throw(
        "[MC_KORTEX] Error in actuator {} fault bank B : {}", joint_idx + 1,
        ss.str());
  }
}

void KinovaRobot::checkActuatorsFaultBanks(
    k_api::BaseCyclic::Feedback feedback) {
  for (size_t i = 0; i < m_actuator_count; i++) {
    if (feedback.mutable_actuators(i)->fault_bank_a() != 0) {
      auto error_list =
          getActuatorFaultList(feedback.mutable_actuators(i)->fault_bank_a());
      std::ostringstream ss;
      ss << "[";
      std::copy(error_list.begin(), error_list.end() - 1,
                std::ostream_iterator<std::string>(ss, ", "));
      ss << error_list.back() << "]";
      mc_rtc::log::error_and_throw(
          "[MC_KORTEX] Error in base fault bank A : {}", ss.str());
    }
    if (feedback.mutable_actuators(i)->fault_bank_b() != 0) {
      auto error_list =
          getActuatorFaultList(feedback.mutable_actuators(i)->fault_bank_b());
      std::ostringstream ss;
      ss << "[";
      std::copy(error_list.begin(), error_list.end() - 1,
                std::ostream_iterator<std::string>(ss, ", "));
      ss << error_list.back() << "]";
      mc_rtc::log::error_and_throw(
          "[MC_KORTEX] Error in base fault bank B : {}", ss.str());
    }
  }
}

std::vector<std::string> KinovaRobot::getBaseFaultList(uint32_t fault_bank) {
  std::vector<string> fault_list;
  if (fault_bank & (uint32_t)0x1)
    fault_list.push_back("FIRMWARE_UPDATE_FAILURE");
  if (fault_bank & (uint32_t)0x2)
    fault_list.push_back("EXTERNAL_COMMUNICATION_ERROR");
  if (fault_bank & (uint32_t)0x4)
    fault_list.push_back("MAXIMUM_AMBIENT_TEMPERATURE");
  if (fault_bank & (uint32_t)0x8)
    fault_list.push_back("MAXIMUM_CORE_TEMPERATURE");
  if (fault_bank & (uint32_t)0x10)
    fault_list.push_back("JOINT_FAULT");
  if (fault_bank & (uint32_t)0x20)
    fault_list.push_back("CYCLIC_DATA_JITTER");
  if (fault_bank & (uint32_t)0x40)
    fault_list.push_back("REACHED_MAXIMUM_EVENT_LOGS");
  if (fault_bank & (uint32_t)0x80)
    fault_list.push_back("NO_KINEMATICS_SUPPORT");
  if (fault_bank & (uint32_t)0x100)
    fault_list.push_back("ABOVE_MAXIMUM_DOF");
  if (fault_bank & (uint32_t)0x200)
    fault_list.push_back("NETWORK_ERROR");
  if (fault_bank & (uint32_t)0x400)
    fault_list.push_back("UNABLE_TO_REACH_POSE");
  if (fault_bank & (uint32_t)0x800)
    fault_list.push_back("JOINT_DETECTION_ERROR");
  if (fault_bank & (uint32_t)0x1000)
    fault_list.push_back("NETWORK_INITIALIZATION_ERROR");
  if (fault_bank & (uint32_t)0x2000)
    fault_list.push_back("MAXIMUM_CURRENT");
  if (fault_bank & (uint32_t)0x4000)
    fault_list.push_back("MAXIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x8000)
    fault_list.push_back("MINIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x10000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY");
  if (fault_bank & (uint32_t)0x20000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY");
  if (fault_bank & (uint32_t)0x40000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION");
  if (fault_bank & (uint32_t)0x80000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION");
  if (fault_bank & (uint32_t)0x100000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE");
  if (fault_bank & (uint32_t)0x200000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE");
  if (fault_bank & (uint32_t)0x400000)
    fault_list.push_back("MAXIMUM_END_EFFECTOR_PAYLOAD");
  if (fault_bank & (uint32_t)0x800000)
    fault_list.push_back("EMERGENCY_STOP_ACTIVATED");
  if (fault_bank & (uint32_t)0x1000000)
    fault_list.push_back("EMERGENCY_LINE_ACTIVATED");
  if (fault_bank & (uint32_t)0x2000000)
    fault_list.push_back("INRUSH_CURRENT_LIMITER_FAULT");
  if (fault_bank & (uint32_t)0x4000000)
    fault_list.push_back("NVRAM_CORRUPTED");
  if (fault_bank & (uint32_t)0x8000000)
    fault_list.push_back("INCOMPATIBLE_FIRMWARE_VERSION");
  if (fault_bank & (uint32_t)0x10000000)
    fault_list.push_back("POWERON_SELF_TEST_FAILURE");
  if (fault_bank & (uint32_t)0x20000000)
    fault_list.push_back("DISCRETE_INPUT_STUCK_ACTIVE");
  if (fault_bank & (uint32_t)0x40000000)
    fault_list.push_back("ARM_INTO_ILLEGAL_POSITION");
  return fault_list;
}

std::vector<std::string>
KinovaRobot::getActuatorFaultList(uint32_t fault_bank) {
  std::vector<string> fault_list;
  if (fault_bank & (uint32_t)0x1)
    fault_list.push_back("FOLLOWING_ERROR");
  if (fault_bank & (uint32_t)0x2)
    fault_list.push_back("MAXIMUM_VELOCITY");
  if (fault_bank & (uint32_t)0x4)
    fault_list.push_back("JOINT_LIMIT_HIGH");
  if (fault_bank & (uint32_t)0x8)
    fault_list.push_back("JOINT_LIMIT_LOW");
  if (fault_bank & (uint32_t)0x10)
    fault_list.push_back("STRAIN_GAUGE_MISMATCH");
  if (fault_bank & (uint32_t)0x20)
    fault_list.push_back("MAXIMUM_TORQUE");
  if (fault_bank & (uint32_t)0x40)
    fault_list.push_back("UNRELIABLE_ABSOLUTE_POSITION");
  if (fault_bank & (uint32_t)0x80)
    fault_list.push_back("MAGNETIC_POSITION");
  if (fault_bank & (uint32_t)0x100)
    fault_list.push_back("HALL_POSITION");
  if (fault_bank & (uint32_t)0x200)
    fault_list.push_back("HALL_SEQUENCE");
  if (fault_bank & (uint32_t)0x400)
    fault_list.push_back("INPUT_ENCODER_HALL_MISMATCH");
  if (fault_bank & (uint32_t)0x800)
    fault_list.push_back("INPUT_ENCODER_INDEX_MISMATCH");
  if (fault_bank & (uint32_t)0x1000)
    fault_list.push_back("INPUT_ENCODER_MAGNETIC_MISMATCH");
  if (fault_bank & (uint32_t)0x2000)
    fault_list.push_back("MAXIMUM_MOTOR_CURRENT");
  if (fault_bank & (uint32_t)0x4000)
    fault_list.push_back("MOTOR_CURRENT_MISMATCH");
  if (fault_bank & (uint32_t)0x8000)
    fault_list.push_back("MAXIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x10000)
    fault_list.push_back("MINIMUM_VOLTAGE");
  if (fault_bank & (uint32_t)0x20000)
    fault_list.push_back("MAXIMUM_MOTOR_TEMPERATURE");
  if (fault_bank & (uint32_t)0x40000)
    fault_list.push_back("MAXIMUM_CORE_TEMPERATURE");
  if (fault_bank & (uint32_t)0x80000)
    fault_list.push_back("NON_VOLATILE_MEMORY_CORRUPTED");
  if (fault_bank & (uint32_t)0x100000)
    fault_list.push_back("MOTOR_DRIVER_FAULT");
  if (fault_bank & (uint32_t)0x200000)
    fault_list.push_back("EMERGENCY_LINE_ASSERTED");
  if (fault_bank & (uint32_t)0x400000)
    fault_list.push_back("COMMUNICATION_TICK_LOST");
  if (fault_bank & (uint32_t)0x800000)
    fault_list.push_back("WATCHDOG_TRIGGERED");
  if (fault_bank & (uint32_t)0x1000000)
    fault_list.push_back("UNRELIABLE_CAPACITIVE_SENSOR");
  if (fault_bank & (uint32_t)0x2000000)
    fault_list.push_back("UNEXPECTED_GEAR_RATIO");
  if (fault_bank & (uint32_t)0x4000000)
    fault_list.push_back("HALL_MAGNETIC_MISMATCH");
  return fault_list;
}

std::vector<std::string>
KinovaRobot::getFeedbackStatusFlag(uint32_t status_flags) {
  std::vector<string> status_flag_list;
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::UNKNOWN_STATUS) {
    status_flag_list.push_back("UNKNOWN_STATUS");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::STABILIZED) {
    status_flag_list.push_back("STABILIZED");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::MOTOR_INDEXED) {
    status_flag_list.push_back("MOTOR_INDEXED");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::MOTOR_INDEXING) {
    status_flag_list.push_back("MOTOR_INDEXING");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::JOINT_INDEXED) {
    status_flag_list.push_back("JOINT_INDEXED");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::JOINT_INDEXING) {
    status_flag_list.push_back("JOINT_INDEXING");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::HIGH_PRECISION) {
    status_flag_list.push_back("HIGH_PRECISION");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::BRAKING) {
    status_flag_list.push_back("BRAKING");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::SERVOING) {
    status_flag_list.push_back("SERVOING");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::MAJOR_FAULT) {
    status_flag_list.push_back("MAJOR_FAULT");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::MINOR_FAULT) {
    status_flag_list.push_back("MINOR_FAULT");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::CALIBRATED_TORQUE) {
    status_flag_list.push_back("CALIBRATED_TORQUE");
  }
  if (status_flags &
      k_api::ActuatorCyclic::StatusFlags::CALIBRATED_MAG_SENSOR) {
    status_flag_list.push_back("CALIBRATED_MAG_SENSOR");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::CALIBRATED_ZERO) {
    status_flag_list.push_back("CALIBRATED_ZERO");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::GPIO_0) {
    status_flag_list.push_back("GPIO_0");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::GPIO_1) {
    status_flag_list.push_back("GPIO_1");
  }
  if (status_flags &
      k_api::ActuatorCyclic::StatusFlags::CS_QUASI_STATIC_CONTACT) {
    status_flag_list.push_back("CS_QUASI_STATIC_CONTACT");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::CS_TRANSIENT_CONTACT) {
    status_flag_list.push_back("CS_TRANSIENT_CONTACT");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::VFD_HALL_SYNC) {
    status_flag_list.push_back("VFD_HALL_SYNC");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::VFD_INDEXED) {
    status_flag_list.push_back("VFD_INDEXED");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::DRIVE_BOARD_READY) {
    status_flag_list.push_back("DRIVE_BOARD_READY");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::CALIBRATED_CURRENT) {
    status_flag_list.push_back("CALIBRATED_CURRENT");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::CALIBRATED_MOTOR) {
    status_flag_list.push_back("CALIBRATED_MOTOR");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::SW0_ACTIVE) {
    status_flag_list.push_back("SW0_ACTIVE");
  }
  if (status_flags & k_api::ActuatorCyclic::StatusFlags::SW1_ACTIVE) {
    status_flag_list.push_back("SW1_ACTIVE");
  }
  return status_flag_list;
}

void KinovaRobot::controlThread(mc_control::MCGlobalController &controller,
                                std::mutex &startM,
                                std::condition_variable &startCV, bool &start,
                                bool &running) {
  m_control_ready = false;
  m_command_ready = false;

  if (m_low_level_interface_type == LowLevelInterfaceType::Bypass) {
    debugLog("Bypass already initialized, waiting for controller start");
  } else {
    setLowServoingMode();
  }
  m_control_ready = true;

  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }
  debugLog("Kinova control thread entering cyclic loop");

  addLogEntry(controller);
  bool has_prev_tick = false;
  int64_t last_tick = 0;
  auto next_tick = std::chrono::steady_clock::now();

  try {

    while (not stop_controller) {
      next_tick += std::chrono::milliseconds(1);
      std::this_thread::sleep_until(next_tick);
      if (stop_controller.load()) {
        break;
      }

      if (!m_command_ready.load()) {
        continue;
      }

      const int64_t now = GetTickUs();
      m_dt = has_prev_tick ? now - last_tick : 0;
      last_tick = now;
      has_prev_tick = true;

      if (m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) {
        sendCommandBase(controller.robots().robot(m_name), running);
      } else if (m_servoing_mode ==
                 k_api::Base::ServoingMode::BYPASS_SERVOING) {
        sendCommandBypass(controller.robots().robot(m_name), running);
      } else {
        mc_rtc::log::info("high level servoing");
      }
    }

    removeLogEntry(controller);

    mc_rtc::log::warning("[MC_KORTEX] {} control loop killed", m_name);
  } catch (k_api::KDetailedException &ex) {
    std::cout << "Kortex error: " << ex.what() << std::endl;
  } catch (std::runtime_error &ex2) {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
  }

  auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
  control_mode.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

  for (int i = 0; i < m_actuator_count; i++) {
    setActuatorControlMode(i, control_mode);
  }

  setSingleServoingMode();
  m_control_ready = false;
}

void KinovaRobot::stopController() { stop_controller = true; }

void KinovaRobot::moveToHomePosition() {
  // Make sure the arm is in Single Level Servoing before executing an Action
  setSingleServoingMode();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Move arm to "Home" position
  mc_rtc::log::info("[MC_KORTEX] Moving the arm to a safe position");
  auto action_type = k_api::Base::RequestedActionType();
  action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
  auto action_list = m_base->ReadAllActions(action_type);
  auto action_handle = k_api::Base::ActionHandle();
  action_handle.set_identifier(0);
  for (auto action : action_list.action_list()) {
    if (action.name() == "Home") {
      action_handle = action.handle();
    }
  }

  if (action_handle.identifier() == 0) {
    mc_rtc::log::warning("[MC_KORTEX] Can't reach safe position, exiting");
  } else {
    bool action_finished = false;
    // Notify of any action topic event
    auto options = k_api::Common::NotificationOptions();
    auto notification_handle = m_base->OnNotificationActionTopic(
        check_for_end_or_abort(action_finished), options);

    m_base->ExecuteActionFromReference(action_handle);

    while (!action_finished) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    m_base->Unsubscribe(notification_handle);
  }
}

void KinovaRobot::moveToInitPosition() {
  // Make sure the arm is in Single Level Servoing before executing an Action
  setSingleServoingMode();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Create trajectory object
  k_api::Base::WaypointList wpts = k_api::Base::WaypointList();

  // Define joint poses
  auto jointPoses = std::vector<std::array<float, 7>>();
  std::array<float, 7> arr;
  for (size_t i = 0; i < min(m_actuator_count, 7); i++)
    arr[i] = radToJointPose(i, m_init_posture[i]);
  jointPoses.push_back(arr);

  // Add initial pose as a waypoint
  k_api::Base::Waypoint *wpt = wpts.add_waypoints();
  wpt->set_name("waypoint_0");
  k_api::Base::AngularWaypoint *ang = wpt->mutable_angular_waypoint();
  for (size_t i = 0; i < m_actuator_count; i++) {
    ang->add_angles(jointPoses.at(0).at(i));
  }

  // Connect to notification action topic
  std::promise<k_api::Base::ActionEvent> finish_promise_cart;
  auto finish_future_cart = finish_promise_cart.get_future();
  auto promise_notification_handle_cart = m_base->OnNotificationActionTopic(
      create_event_listener_by_promise(finish_promise_cart),
      k_api::Common::NotificationOptions());

  k_api::Base::WaypointValidationReport result;
  try {
    // Verify validity of waypoints
    auto validationResult = m_base->ValidateWaypointList(wpts);
    result = validationResult;
  } catch (k_api::KDetailedException &ex) {
    mc_rtc::log::error(
        "[MC_KORTEX] Error on waypoint list to reach initial position");
    printException(ex);
    return;
  }

  // Trajectory error report always exists and we need to make sure no elements
  // are found in order to validate the trajectory
  if (result.trajectory_error_report().trajectory_error_elements_size() == 0) {
    // Execute action
    try {
      mc_rtc::log::info("[MC_KORTEX] Moving the arm to initial position");
    } catch (k_api::KDetailedException &ex) {
      mc_rtc::log::error("[MC_KORTEX] Error when trying to execute trajectory "
                         "to reach initial position");
      printException(ex);
      return;
    }
    // Wait for future value from promise
    const auto ang_status =
        finish_future_cart.wait_for(std::chrono::seconds(100));
    m_base->Unsubscribe(promise_notification_handle_cart);
    if (ang_status != std::future_status::ready) {
      mc_rtc::log::warning("[MC_KORTEX] Timeout when trying to reach initial "
                           "position, try again");
    } else {
      const auto ang_promise_event = finish_future_cart.get();
      mc_rtc::log::success("[MC_KORTEX] Angular waypoint trajectory completed");
    }
  } else {
    mc_rtc::log::error(
        "[MC_KORTEX] Error found in trajectory to initial position");
    mc_rtc::log::error(result.trajectory_error_report().DebugString());
  }
}

void KinovaRobot::printState() {
  std::string serialized_data;
  google::protobuf::util::MessageToJsonString(m_state, &serialized_data);
  std::cout << serialized_data << std::endl;
}

void KinovaRobot::printJointActiveControlLoop(int joint_id) {
  uint32_t control_loop;
  std::vector<std::string> active_loops;
  control_loop =
      m_actuator_config->GetActivatedControlLoop(joint_id).control_loop();
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::JOINT_POSITION)
    active_loops.push_back("JOINT_POSITION");
  if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::JOINT_TORQUE)
    active_loops.push_back("JOINT_TORQUE");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::JOINT_TORQUE_HIGH_VELOCITY)
    active_loops.push_back("JOINT_TORQUE_HIGH_VELOCITY");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::JOINT_VELOCITY)
    active_loops.push_back("JOINT_VELOCITY");
  if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::MOTOR_CURRENT)
    active_loops.push_back("MOTOR_CURRENT");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::MOTOR_POSITION)
    active_loops.push_back("MOTOR_POSITION");
  if (control_loop &
      k_api::ActuatorConfig::ControlLoopSelection::MOTOR_VELOCITY)
    active_loops.push_back("MOTOR_VELOCITY");

  std::ostringstream ss;
  ss << "[";
  std::copy(active_loops.begin(), active_loops.end() - 1,
            std::ostream_iterator<std::string>(ss, ", "));
  ss << active_loops.back() << "]";

  mc_rtc::log::info("[MC_KORTEX][Joint {}] Active control loops: {}", joint_id,
                    ss.str());
}

// ============================== Private methods ==============================
// //

void KinovaRobot::initFiltersBuffers() {
  m_filter_input_buffer.assign(m_actuator_count,
                               boost::circular_buffer<double>(2, 0.0));
  m_filter_output_buffer.assign(m_actuator_count,
                                boost::circular_buffer<double>(2, 0.0));
}

void KinovaRobot::addGui(mc_control::MCGlobalController &gc) {
  gc.controller().gui()->addElement(
      {"Kortex", m_name},
      mc_rtc::gui::Button("Move to initial position", [this]() {
        setSingleServoingMode();
        moveToInitPosition();
        setLowServoingMode();
      }));

  gc.controller().gui()->addElement(
      {"Kortex", m_name},
      mc_rtc::gui::ArrayLabel("PostureTask offsets",
                              gc.controller().robot().refJointOrder(),
                              [this]() { return m_offsets; }));

  if (m_torque_control_type == mc_kinova::TorqueControlType::Custom) {
    gc.controller().gui()->addElement(
        {"Kortex", m_name, "Friction"},
        mc_rtc::gui::ArrayInput(
            "Friction stiction values", gc.controller().robot().refJointOrder(),
            [this]() { return m_stiction_values; },
            [this](const std::vector<double> &v) { m_stiction_values = v; }),
        mc_rtc::gui::ArrayInput(
            "Friction coulomb values", gc.controller().robot().refJointOrder(),
            [this]() { return m_friction_values; },
            [this](const std::vector<double> &v) { m_friction_values = v; }),
        mc_rtc::gui::ArrayInput(
            "Friction viscous values", gc.controller().robot().refJointOrder(),
            [this]() { return m_viscous_values; },
            [this](const std::vector<double> &v) { m_viscous_values = v; }),
        mc_rtc::gui::NumberInput(
            "Friction compensation velocity threshold",
            [this]() { return m_friction_vel_threshold; },
            [this](const double v) { m_friction_vel_threshold = v; }),
        mc_rtc::gui::NumberInput(
            "Friction compensation acceleration threshold",
            [this]() { return m_friction_accel_threshold; },
            [this](const double v) { m_friction_accel_threshold = v; }));

    gc.controller().gui()->addElement(
        {"Kortex", m_name, "Transfer function"},
        mc_rtc::gui::NumberInput(
            "Lambda", [this]() { return m_lambda[0]; },
            [this](const double v) { m_lambda.assign(v, m_actuator_count); }));

    gc.controller().gui()->addElement(
        {"Kortex", m_name, "Integral term"},
        mc_rtc::gui::NumberInput(
            "Integral time constant",
            [this]() { return m_integral_slow_theta; },
            [this](const double v) { m_integral_slow_theta = v; }),
        mc_rtc::gui::NumberInput(
            "Integral gain", [this]() { return m_integral_slow_gain; },
            [this](const double v) { m_integral_slow_gain = v; }),
        mc_rtc::gui::ArrayInput(
            "Integral bound", [this]() { return m_integral_slow_bound; },
            [this](const std::vector<double> v) {
              m_integral_slow_bound = v;
            }));
  }

  if (m_use_filtered_velocities) {
    gc.controller().gui()->addElement(
        {"Kortex", m_name},
        mc_rtc::gui::NumberSlider(
            "Velocity filtering ratio (decay):",
            [this]() { return m_velocity_filter_ratio; },
            [this](const double v) { m_velocity_filter_ratio = v; }, 0.0, 1.0));
  }
}

void KinovaRobot::removeGui(mc_control::MCGlobalController &gc) {
  gc.controller().gui()->removeCategory({"Kortex"});
  mc_rtc::log::success("[MC_KORTEX] Removed GUI");
}

void KinovaRobot::addPlot(mc_control::MCGlobalController &gc) {
  for (int i = 0; i < m_actuator_count; i++) {
    gc.controller().gui()->addPlot(
        fmt::format("Joint {}", i),
        mc_rtc::gui::plot::X("t", [this]() { return t_plot; }),
        mc_rtc::gui::plot::Y(
            "Integral term", [this, i]() { return m_integral_slow_filter[i]; },
            mc_rtc::gui::Color::Red),
        mc_rtc::gui::plot::Y(
            "Transfert function", [this, i]() { return m_filter_command[i]; },
            mc_rtc::gui::Color::Blue));
  }
}

void KinovaRobot::removePlot(mc_control::MCGlobalController &gc) {
  for (int i = 0; i < m_actuator_count; i++) {
    gc.controller().gui()->removePlot(
        fmt::format("Low-level torque joint {}", i));
  }
}

double KinovaRobot::jointPoseToRad(int joint_idx, double deg) {
  return mc_rtc::constants::toRad((deg < 180.0) ? deg : deg - 360);
}

double KinovaRobot::radToJointPose(int joint_idx, double rad) {
  return mc_rtc::constants::toDeg((rad > 0) ? rad : 2 * M_PI + rad);
}

std::vector<double>
KinovaRobot::computePostureTaskOffset(mc_rbdyn::Robot &robot,
                                      mc_tasks::PostureTaskPtr posture_task) {
  std::vector<double> offsets(m_actuator_count, 0.0);

  // Posture task only in damping mode no need for offsets
  if (posture_task == nullptr)
    return offsets;
  if (posture_task->posture().size() == 0)
    return offsets;

  auto rjo = robot.refJointOrder();

  auto target = posture_task->posture();
  // fmt::print("Posture task stiffness = {}\n",posture_task->stiffness());
  for (size_t i = 0; i < m_actuator_count; i++) {
    double target_pose = target[robot.jointIndexByName(rjo[i])][0];
    double q = 0.0;
    if (m_servoing_mode == k_api::Base::ServoingMode::BYPASS_SERVOING) {
      q = jointPoseToRad(i, m_actuator_feedback_vec[i].position());
    } else {
      q = jointPoseToRad(i, m_state.mutable_actuators(i)->position());
    }
    // std::cout << target_pose << " " << q << " | ";
    if (i % 2 == 0) {
      if (q > (target_pose + M_PI))
        offsets[i] = -2 * M_PI;
      else if (q < (target_pose - M_PI))
        offsets[i] = 2 * M_PI;
    }
  }
  // std::cout << "\n";
  return offsets;
}

uint32_t KinovaRobot::jointIdFromCommandID(google::protobuf::uint32 cmd_id) {
  return (cmd_id >> 16) & 0x0000000F;
}

int64_t KinovaRobot::GetTickUs() {
  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);

  return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

void KinovaRobot::printError(const k_api::Error &err) {
  mc_rtc::log::error("[MC_KORTEX] KError error_code: {}", err.error_code());
  mc_rtc::log::error("[MC_KORTEX] KError sub_code: {}", err.error_sub_code());
  mc_rtc::log::error("[MC_KORTEX] KError sub_string: {}",
                     err.error_sub_string());

  // Error codes by themselves are not very verbose if you don't see their
  // corresponding enum value You can use google::protobuf helpers to get the
  // string enum element for every error code and sub-code
  mc_rtc::log::error(
      "[MC_KORTEX] Error code string equivalent: {}",
      k_api::ErrorCodes_Name(k_api::ErrorCodes(err.error_code())));
  mc_rtc::log::error(
      "[MC_KORTEX] Error sub-code string equivalent: {}",
      k_api::SubErrorCodes_Name(k_api::SubErrorCodes(err.error_sub_code())));
}

void KinovaRobot::printException(k_api::KDetailedException &ex) {
  // You can print the error informations and error codes
  auto error_info = ex.getErrorInfo().getError();
  mc_rtc::log::error("[MC_KORTEX] KDetailedException detected : {}", ex.what());

  printError(error_info);
}

std::function<void(k_api::Base::ActionNotification)>
KinovaRobot::check_for_end_or_abort(bool &finished) {
  return [&finished](k_api::Base::ActionNotification notification) {
    mc_rtc::log::info(
        "[MC_KORTEX] EVENT : {}",
        k_api::Base::ActionEvent_Name(notification.action_event()));

    // The action is finished when we receive a END or ABORT event
    switch (notification.action_event()) {
    case k_api::Base::ActionEvent::ACTION_ABORT:
    case k_api::Base::ActionEvent::ACTION_END:
      finished = true;
      break;
    default:
      break;
    }
  };
}

std::function<void(k_api::Base::ActionNotification)>
KinovaRobot::create_event_listener_by_promise(
    std::promise<k_api::Base::ActionEvent> &finish_promise_cart) {
  return [&finish_promise_cart](k_api::Base::ActionNotification notification) {
    const auto action_event = notification.action_event();
    switch (action_event) {
    case k_api::Base::ActionEvent::ACTION_END:
    case k_api::Base::ActionEvent::ACTION_ABORT:
      finish_promise_cart.set_value(action_event);
      break;
    default:
      break;
    }
  };
}

std::string printVec(std::vector<double> vec) {
  std::ostringstream s;
  s << "[";
  std::copy(vec.begin(), vec.end() - 1, std::ostream_iterator<double>(s, ", "));
  s << vec.back() << "]";
  return s.str();
}

} // namespace mc_kinova
