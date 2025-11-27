#include "KinovaRobot.h"

namespace mc_kinova {

KinovaRobot::KinovaRobot(const std::string &name, const std::string &ip_address,
                         const std::string &username = "admin",
                         const std::string &password = "admin")
    : m_name(name), m_ip_address(ip_address), m_port(10000),
      m_port_real_time(10001), m_username(username), m_password(password),
      stop_controller(false) {
  m_router = nullptr;
  m_router_real_time = nullptr;
  m_transport = nullptr;
  m_transport_real_time = nullptr;
  m_session_manager = nullptr;
  m_session_manager_real_time = nullptr;
  m_base = nullptr;
  m_base_cyclic = nullptr;
  m_device_manager = nullptr;
  m_actuator_config = nullptr;

  m_state = k_api::BaseCyclic::Feedback();
  m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
  m_control_mode_id = 0;
  m_prev_control_mode_id = 0;
  m_torque_control_type = mc_kinova::TorqueControlType::Default;
}

KinovaRobot::~KinovaRobot() {
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

// ==================== Setter ==================== //

void KinovaRobot::setLowServoingMode() {
  // Ignore if already in low level servoing mode
  // if(m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
  // return;

  auto servoingMode = k_api::Base::ServoingModeInformation();

  servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  m_base->SetServoingMode(servoingMode);
  m_servoing_mode = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
}

void KinovaRobot::setSingleServoingMode() {
  // Ignore if already in "high" level servoing mode
  // if(m_servoing_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
  // return;

  auto servoingMode = k_api::Base::ServoingModeInformation();

  servoingMode.set_servoing_mode(
      k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  m_base->SetServoingMode(servoingMode);
  m_servoing_mode = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
}

void KinovaRobot::initKalmanFromConfig(mc_rtc::Configuration &config) {
  if (config.has("init")) {
    config("init")("pTerm", initPTerm);
    config("init")("biasTerm", initBiasTerm);
  }
  if (config.has("limit")) {
    if (config("limit").has("pTerm")) {
      config("limit")("pTerm")("max", limitPTermMax);
      config("limit")("pTerm")("min", limitPTermMin);
    }
    if (config("limit").has("biasTerm")) {
      config("limit")("biasTerm")("max", limitBiasTermMax);
      config("limit")("biasTerm")("min", limitBiasTermMin);
    }
  }
  if (config.has("covar")) {
    config("covar")("initPTerm", covarInitPTerm);
    config("covar")("initBiasTerm", covarInitBiasTerm);
    config("covar")("pTerm", covarPTerm);
    config("covar")("biasTerm", covarBiasTerm);
    config("covar")("sensTerm", covarSensTerm);
  }
  stateCovar << covarInitPTerm, 0.0, 0.0, covarInitBiasTerm;
  processCovar << covarPTerm, 0.0, 0.0, covarBiasTerm;
  measureCovar << covarSensTerm;
  filtAlpha.setZero(m_actuator_count);
  filtBeta.setZero(m_actuator_count);
  tau_r.setZero(m_actuator_count);
  tau_r_theo.setZero(m_actuator_count);
  vecKalmanFilt_.resize(7);
  for (size_t i = 0; i < m_actuator_count; i++) {
    filtAlpha(i) = initPTerm;
    filtBeta(i) = initBiasTerm;
    vecKalmanFilt_[i] = new stateObservation::LinearKalmanFilter(2, 1, 0);
    vecKalmanFilt_[i]->setA(Eigen::Matrix2d::Identity());
    vecKalmanFilt_[i]->setB(Eigen::Matrix<double, 2, 0>());
    vecKalmanFilt_[i]->setD(Eigen::Matrix<double, 1, 0>());
    vecKalmanFilt_[i]->setC(
        Eigen::Vector2d(0.0, 1.0).transpose()); // TODO set correct initial C
    vecKalmanFilt_[i]->setState(Eigen::Vector2d(initPTerm, initBiasTerm), 0);
    vecKalmanFilt_[i]->setStateCovariance(stateCovar);
    vecKalmanFilt_[i]->setProcessCovariance(processCovar);
    vecKalmanFilt_[i]->setMeasurementCovariance(measureCovar);
  }
}

void KinovaRobot::setCustomTorque(mc_rtc::Configuration &torque_config) {
  if (torque_config.has("friction_compensation")) {
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
      "[mc_kortex] {} robot is using custom torque control with parameters:",
      m_name);
}

void KinovaRobot::setControlMode(std::string mode) {
  if (mode.compare("Position") == 0) {
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION)
      return;

    // mc_rtc::log::info("[mc_kortex] Using position control");
    m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
    m_control_mode_id++;
    return;
  }
  if (mode.compare("Velocity") == 0) {
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::VELOCITY)
      return;

    // mc_rtc::log::info("[mc_kortex] Using velocity control");
    m_control_mode = k_api::ActuatorConfig::ControlMode::VELOCITY;
    m_control_mode_id++;
    return;
  }
  if (mode.compare("Torque") == 0) {
    switch (m_torque_control_type) {
    case mc_kinova::TorqueControlType::Default:
    case mc_kinova::TorqueControlType::Kalman:
      if (m_control_mode ==
          k_api::ActuatorConfig::ControlMode::TORQUE_HIGH_VELOCITY)
        return;

      // mc_rtc::log::info("[mc_kortex] Using torque control");
      m_control_mode = k_api::ActuatorConfig::ControlMode::TORQUE_HIGH_VELOCITY;
      m_control_mode_id++;
      return;
      break;
    case mc_kinova::TorqueControlType::Feedforward:
    case mc_kinova::TorqueControlType::Custom:
      if (m_control_mode == k_api::ActuatorConfig::ControlMode::CURRENT)
        return;

      // mc_rtc::log::info("[mc_kortex] Using torque control");
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
  } else if (mode.compare("Kalman") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Kalman;
  } else if (mode.compare("Custom") == 0) {
    m_torque_control_type = mc_kinova::TorqueControlType::Custom;
  } else {
    mc_rtc::log::error("[mc_kortex] Unknown torque control type: {}", mode);
  }
}

// ==================== Public functions ==================== //

void KinovaRobot::init(mc_control::MCGlobalController &gc,
                       mc_rtc::Configuration &kortexConfig) {
  mc_rtc::log::info("[mc_kortex] Initializing connection to the robot at {}:{}",
                    m_ip_address, m_port);

  auto error_callback = [](k_api::KError err) {
    mc_rtc::log::error("_________ callback error _________ {}", err.toString());
  };

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
  m_actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(m_router);
  m_base = new k_api::Base::BaseClient(m_router);
  m_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(m_router_real_time);

  // Read actuators count
  setSingleServoingMode();
  m_actuator_count = m_base->GetActuatorCount().count();

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
  initFilt.assign(m_actuator_count, true);
  tau_r.setZero(m_actuator_count);
  m_lambda.assign(m_actuator_count, 0.0);
  m_integral_slow_theta = 1.0;
  m_integral_slow_gain = 1e-2;

  // Set control mode
  auto controle_mode = k_api::ActuatorConfig::ControlModeInformation();
  controle_mode.set_control_mode(m_control_mode);
  for (int i = 0; i < m_actuator_count; i++)
    m_actuator_config->SetControlMode(controle_mode, i + 1);

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

  // Initialize state
  updateState();
  updateSensors(gc);

  // Velocity filtering init
  m_use_filtered_velocities = kortexConfig.has("filter_velocity");
  if (m_use_filtered_velocities) {
    m_velocity_filter_ratio = kortexConfig("filter_velocity")("ratio", 0.0);
    mc_rtc::log::info(
        "[mc_kortex] Filtering velocities for {} robot with {} ratio", m_name,
        m_velocity_filter_ratio);
  }
  m_filtered_velocities.assign(m_actuator_count, 0.0);

  // Custom torque control init
  if (kortexConfig.has("torque_control")) {
    kortexConfig = kortexConfig("torque_control");
    if (!kortexConfig.has("mode"))
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] For {} robot, \"torque_control\" key found in config "
          "file but \"mode\" key is missing.",
          m_name);

    std::string controle_mode = kortexConfig("mode");
    if (controle_mode.compare("feedforward") == 0) {
      m_torque_control_type = mc_kinova::TorqueControlType::Feedforward;
      mc_rtc::log::info(
          "[mc_kortex] Using feedforward only for torque control");
    } else if (controle_mode.compare("kalman") == 0) {
      m_torque_control_type = mc_kinova::TorqueControlType::Kalman;
      initKalmanFromConfig(kortexConfig);
      mc_rtc::log::info("[mc_kortex] Using kalman filter for torque control");
    } else if (controle_mode.compare("custom") == 0) {
      m_torque_control_type = mc_kinova::TorqueControlType::Custom;
      setCustomTorque(kortexConfig);
      mc_rtc::log::info("[mc_kortex] Using custom control for torque control");
    } else {
      m_torque_control_type = mc_kinova::TorqueControlType::Default;
      mc_rtc::log::info(
          "[mc_kortex] Using Kinova's default control for torque control");
    }
  }

  // Initialize Jacobian object
  auto robot = &gc.robots().robot(m_name);
  // m_jac = rbd::Jacobian(robot->mb(), "tool_frame");

  Eigen::VectorXd tu = rbd::paramToVector(robot->mb(), robot->tu());
  // Initialize each actuator to its current position
  for (int i = 0; i < m_actuator_count; i++) {
    m_integral_slow_bound[i] = 0.05 * tu[i]; // 5% of torque limit
    m_base_command.add_actuators()->set_position(
        m_state.actuators(i).position());
  }

  addGui(gc);

  mc_rtc::log::success("[mc_kortex] Connected succesfuly to robot at {}:{}",
                       m_ip_address, m_port);
}

void KinovaRobot::addLogEntry(mc_control::MCGlobalController &gc) {
  gc.controller().logger().addLogEntry("kortex_LoopPerf",
                                       [&, this]() { return m_dt; });

  if (m_torque_control_type == mc_kinova::TorqueControlType::Kalman) {
    gc.controller().logger().addLogEntry("kortex_kalman_alpha",
                                         [this]() { return filtAlpha; });
    gc.controller().logger().addLogEntry("kortex_kalman_beta",
                                         [this]() { return filtBeta; });
    gc.controller().logger().addLogEntry("kortex_kalman_tau_r_real",
                                         [this]() { return tau_r; });
    gc.controller().logger().addLogEntry("kortex_kalman_tau_r_theo",
                                         [this]() { return tau_r_theo; });
    gc.controller().logger().addLogEntry(
        "kortex_torque_from_current",
        [this]() { return m_torque_from_current_measurement; });
    gc.controller().logger().addLogEntry("kortex_torque_error",
                                         [this]() { return m_torque_error; });
  }
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
  gc.controller().logger().removeLogEntry("kortexLoopPerf");

  if (m_torque_control_type == mc_kinova::TorqueControlType::Kalman) {
    gc.controller().logger().removeLogEntry("kortex_kalman_alpha");
    gc.controller().logger().removeLogEntry("kortex_kalman_beta");
  }

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
  m_state = m_base_cyclic->RefreshFeedback();
}

void KinovaRobot::updateState(bool &running) {
  m_base_cyclic->RefreshFeedback_callback(
      [&, this](const Kinova::Api::Error &err,
                const k_api::BaseCyclic::Feedback data) {
        updateState(data);
        checkBaseFaultBanks(data.base().fault_bank_a(),
                            data.base().fault_bank_b());
        // checkActuatorsFaultBanks(data);
        if (err.error_code() != k_api::ErrorCodes::ERROR_NONE) {
          printError(err);
          running = false;
        }
      });
}

void KinovaRobot::updateState(const k_api::BaseCyclic::Feedback data) {
  std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
  m_state = data;
}

double KinovaRobot::computeTorqueWKalman(
    double torque, k_api::BaseCyclic::Feedback m_state_local, int8_t idx) {
  auto &filter = vecKalmanFilt_[idx];
  Eigen::Matrix<double, 1, 1> measure;
  measure(0) = -m_state_local.mutable_actuators(idx)->torque();
  mc_rtc::log::info("Measure pas encore passer");
  filter->pushMeasurement(measure);
  if (initFilt[idx]) {
    mc_rtc::log::info("Init en cour");
    tau_r_theo(idx) = (torque - initBiasTerm) / initPTerm;
    tau_r(idx) = tau_r_theo(idx);
    mc_rtc::log::info("Init tau_r set");
    filter->setC(Eigen::Vector2d(tau_r(idx), 1.0).transpose());
    mc_rtc::log::info("Init C set");
    initFilt[idx] = false;
    mc_rtc::log::info("Init fini");
    return tau_r(idx);
  }
  mc_rtc::log::info("Measure c'est passer");
  auto state = filter->getEstimatedState(filter->getMeasurementTime());
  if (state(0) < limitPTermMin or state(0) > limitPTermMax) {
    state(0) = min(max(state(0), limitPTermMin), limitPTermMax);
  }
  if (state(1) < limitBiasTermMin or state(1) > limitBiasTermMax) {
    state(1) = min(max(state(1), limitBiasTermMin), limitBiasTermMax);
  }
  filtAlpha(idx) = state(0);
  filtBeta(idx) = state(1);
  filter->setCurrentState(state);
  // tau_r(idx) = (torque - state(1)) / state(0);
  tau_r_theo(idx) = (torque - state(1)) / state(0);
  tau_r(idx) = tau_r_theo(idx);
  filter->setC(Eigen::Vector2d(tau_r(idx), 1.0).transpose());
  m_torque_error[idx] = tau_r(idx) - measure(0);

  return tau_r(idx);
}

double KinovaRobot::currentTorqueControlLaw(
    mc_rbdyn::Robot &robot, k_api::BaseCyclic::Feedback m_state_local, double joint_idx) {

  auto rjo = robot.refJointOrder();

  double velocity = mc_rtc::constants::toRad(
      m_state_local.mutable_actuators(joint_idx)->velocity());
  double torque_measured = m_state_local.mutable_actuators(joint_idx)->torque();

  double torque_constant = (joint_idx > 3) ? 0.076 : 0.11;
  auto filter_input = m_filter_input_buffer[joint_idx];
  auto filter_output = m_filter_output_buffer[joint_idx];
  double friction_torque = 0.0;

  auto qdd_r = m_command.alphaD[robot.jointIndexByName(rjo[joint_idx])][0];

  double tau_desired =
      m_command.jointTorque[robot.jointIndexByName(rjo[joint_idx])][0];

  double rotor_inertia =
      robot.mb().joint(robot.jointIndexByName(rjo[joint_idx])).rotorInertia();

  double rotor_inertia_torque = rotor_inertia * GEAR_RATIO * GEAR_RATIO * qdd_r;

  double torque_error =
      tau_desired + torque_measured; // - rotor_inertia_torque;

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
      friction_torque = m_friction_values[joint_idx];
    } else if (qdd_r < -m_friction_accel_threshold) {
      m_friction_compensation_mode[joint_idx] = -1;
      friction_torque = -m_friction_values[joint_idx];
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

bool KinovaRobot::sendCommand(mc_rbdyn::Robot &robot, bool &running) {
  bool return_value = true;
  k_api::BaseCyclic::Feedback m_state_local;
  {
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    m_state_local = m_state;
  }

  std::unique_lock<std::mutex> lock(m_update_control_mutex);
  auto rjo = robot.refJointOrder();

  if (m_control_id == m_prev_control_id)
    return false;

  auto lambda_fct = [&, this](const Kinova::Api::Error &err,
                              const k_api::BaseCyclic::Feedback data) {
    updateState(data);
    checkBaseFaultBanks(data.base().fault_bank_a(), data.base().fault_bank_b());
    // checkActuatorsFaultBanks(data);
    if (err.error_code() != k_api::ErrorCodes::ERROR_NONE) {
      printError(err);
      running = false;
    }
  };

  for (size_t i = 0; i < m_actuator_count; i++) {
    double kt = (i > 3) ? 0.076 : 0.11;
    if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION) {
      m_base_command.mutable_actuators(i)->set_position(
          radToJointPose(i, m_command.q[robot.jointIndexByName(rjo[i])][0]));
      m_base_command.mutable_actuators(i)->set_current_motor(
          m_state_local.mutable_actuators(i)->current_motor());
      continue;
    } else {
      m_base_command.mutable_actuators(i)->set_position(
          m_state_local.mutable_actuators(i)->position());
    }

    auto rjo = robot.refJointOrder();

    auto qdd_r = m_command.alphaD[robot.jointIndexByName(rjo[i])][0];

    double tau_desired =
        m_command.jointTorque[robot.jointIndexByName(rjo[i])][0];

    double rotor_inertia =
        robot.mb().joint(robot.jointIndexByName(rjo[i])).rotorInertia();

    double rotor_inertia_torque =
        rotor_inertia * GEAR_RATIO * GEAR_RATIO * qdd_r;

    m_torque_measure_corrected[i] =
        -m_state_local.mutable_actuators(i)->torque() + rotor_inertia_torque;

    switch (m_torque_control_type) {
    case mc_kinova::TorqueControlType::Default:
      m_base_command.mutable_actuators(i)->set_torque_joint(
          m_command.jointTorque[robot.jointIndexByName(rjo[i])][0] -
          rotor_inertia_torque);
      break;
    case mc_kinova::TorqueControlType::Kalman:
      m_base_command.mutable_actuators(i)->set_torque_joint(
          computeTorqueWKalman(
              m_command.jointTorque[robot.jointIndexByName(rjo[i])][0],
              m_state_local, i));
      break;
    case mc_kinova::TorqueControlType::Feedforward:
      m_base_command.mutable_actuators(i)->set_current_motor(
          m_command.jointTorque[robot.jointIndexByName(rjo[i])][0] /
          (GEAR_RATIO * kt));
      m_current_command(i) =
          m_command.jointTorque[robot.jointIndexByName(rjo[i])][0] /
          (GEAR_RATIO * kt);
      break;
    case mc_kinova::TorqueControlType::Custom:
      m_base_command.mutable_actuators(i)->set_current_motor(
          currentTorqueControlLaw(robot, m_state_local, i));
      break;
    default:
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_kortex] wrong torque control type when trying to send command");
      break;
    }
    // std::cout << m_base_command.mutable_actuators(i)->position() << " " <<
    // m_state_local.mutable_actuators(i)->position() << " | ";
  }
  // if (m_control_mode != k_api::ActuatorConfig::ControlMode::POSITION)
  // std::cout << std::endl;

  // ========================= Control mode has changed in mc_rtc, change it for
  // the robot ========================= //
  if (m_control_mode_id != m_prev_control_mode_id) {
    auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
    control_mode.set_control_mode(m_control_mode);

    try {
      mc_rtc::log::info("[mc_kortex] Changing robot control mode to {} ",
                        m_control_mode);
      for (int i = 0; i < m_actuator_count; i++) {
        // printJointActiveControlLoop(i+1);
        m_actuator_config->SetControlMode(control_mode, i + 1);
        // printJointActiveControlLoop(i+1);
      }
      m_prev_control_mode_id = m_control_mode_id;
    } catch (k_api::KDetailedException &ex) {
      printException(ex);
      return_value = false;
      running = false;
    }
  }

  try {
    m_base_cyclic->Refresh_callback(m_base_command, lambda_fct, 0);
    return_value = true;
  } catch (k_api::KDetailedException &ex) {
    printException(ex);
    return_value = false;
    running = false;
  }

  m_prev_control_id = m_control_id;
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

  std::vector<double> q(m_actuator_count);
  std::vector<double> qdot(m_actuator_count);
  std::vector<double> tau(m_actuator_count);
  std::map<std::string, sva::ForceVecd> wrenches;
  double fx, fy, fz, cx, cy, cz;
  std::map<std::string, double> current;
  // std::map<std::string,double> temp;

  for (size_t i = 0; i < m_actuator_count; i++) {
    double kt = (i > 3) ? 0.076 : 0.11;
    q[i] = jointPoseToRad(i, m_state.mutable_actuators(i)->position()) +
           m_offsets[i];
    if (m_use_filtered_velocities) {
      m_filtered_velocities[i] =
          m_velocity_filter_ratio * m_filtered_velocities[i] +
          (1 - m_velocity_filter_ratio) *
              mc_rtc::constants::toRad(
                  m_state.mutable_actuators(i)->velocity());
      qdot[i] = m_filtered_velocities[i];
    } else {
      qdot[i] =
          mc_rtc::constants::toRad(m_state.mutable_actuators(i)->velocity());
    }
    tau[i] = -m_state.mutable_actuators(i)->torque();
    m_tau_sensor(i) = tau[i];
    m_current_measurement(i) = m_state.mutable_actuators(i)->current_motor();
    m_torque_from_current_measurement(i) =
        m_state.mutable_actuators(i)->current_motor() * kt * GEAR_RATIO;
    current[fmt::format("joint_{}", i + 1)] = m_current_measurement(i);
    // temp[rjo[joint_idx]] = actuator.temperature_motor();
  }

  // fx = m_state.base().tool_external_wrench_force_x();
  // fy = m_state.base().tool_external_wrench_force_y();
  // fz = m_state.base().tool_external_wrench_force_z();
  // cx = m_state.base().tool_external_wrench_torque_x();
  // cy = m_state.base().tool_external_wrench_torque_y();
  // cz = m_state.base().tool_external_wrench_torque_z();
  // wrenches[robot.forceSensors()[0].name()] =
  //     sva::ForceVecd(Eigen::Vector3d(cx, cy, cz), Eigen::Vector3d(fx, fy, fz));

  gc.setEncoderValues(m_name, q);
  gc.setEncoderVelocities(m_name, qdot);
  gc.setJointTorques(m_name, tau);
  // gc.setWrenches(wrenches);
  gc.setJointMotorCurrents(m_name, current);
  // gc.setJointMotorTemperatures(m_name,temp);
}

void KinovaRobot::updateControl(mc_control::MCGlobalController &controller) {
  std::unique_lock<std::mutex> lock(m_update_control_mutex);
  auto &robot = controller.controller().robots().robot(m_name);
  m_command = robot.mbc();
  m_control_id++;
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

void KinovaRobot::controlThread(mc_control::MCGlobalController &controller,
                                std::mutex &startM,
                                std::condition_variable &startCV, bool &start,
                                bool &running) {
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }

  setLowServoingMode();

  int64_t now = 0;
  int64_t last = 0;
  int64_t dt = 0;

  addLogEntry(controller);

  bool return_status;

  try {

    while (not stop_controller) {
      now = GetTickUs();
      if (now - last < 1000)
        continue;
      dt = now - last;
      last = now;

      if (m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) {
        sendCommand(controller.robots().robot(m_name), running);
        t_plot += 1e-3;
      } else {
        mc_rtc::log::info("high level servoing");
        // updateState(running);
      }
    }

    removeLogEntry(controller);

    mc_rtc::log::warning("[MC_KORTEX] {} control loop killed", m_name);

    return_status = true;
  } catch (k_api::KDetailedException &ex) {
    std::cout << "Kortex error: " << ex.what() << std::endl;
    return_status = false;
  } catch (std::runtime_error &ex2) {
    std::cout << "Runtime error: " << ex2.what() << std::endl;
    return_status = false;
  }

  auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
  control_mode.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

  for (int i = 0; i < m_actuator_count; i++) {
    // printJointActiveControlLoop(i+1);
    m_actuator_config->SetControlMode(control_mode, i + 1);
    // printJointActiveControlLoop(i+1);
  }

  setSingleServoingMode();

  removeGui(controller);
}

void KinovaRobot::stopController() { stop_controller = true; }

void KinovaRobot::moveToHomePosition() {
  // Make sure the arm is in Single Level Servoing before executing an Action
  setSingleServoingMode();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Move arm to "Home" position
  mc_rtc::log::info("[mc_kortex] Moving the arm to a safe position");
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
    mc_rtc::log::warning("[mc_kortex] Can't reach safe position, exiting");
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
        "[mc_kortex] Error on waypoint list to reach initial position");
    printException(ex);
    return;
  }

  // Trajectory error report always exists and we need to make sure no elements
  // are found in order to validate the trajectory
  if (result.trajectory_error_report().trajectory_error_elements_size() == 0) {
    // Execute action
    try {
      mc_rtc::log::info("[mc_kortex] Moving the arm to initial position");
    } catch (k_api::KDetailedException &ex) {
      mc_rtc::log::error("[mc_kortex] Error when trying to execute trajectory "
                         "to reach initial position");
      printException(ex);
      return;
    }
    // Wait for future value from promise
    const auto ang_status =
        finish_future_cart.wait_for(std::chrono::seconds(100));
    m_base->Unsubscribe(promise_notification_handle_cart);
    if (ang_status != std::future_status::ready) {
      mc_rtc::log::warning("[mc_kortex] Timeout when trying to reach initial "
                           "position, try again");
    } else {
      const auto ang_promise_event = finish_future_cart.get();
      mc_rtc::log::success("[mc_kortex] Angular waypoint trajectory completed");
    }
  } else {
    mc_rtc::log::error(
        "[mc_kortex] Error found in trajectory to initial position");
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

  mc_rtc::log::info("[mc_kortex][Joint {}] Active control loops: {}", joint_id,
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

  if (m_torque_control_type == mc_kinova::TorqueControlType::Kalman) {
    gc.controller().gui()->addElement(
        {"Kortex", m_name},
        mc_rtc::gui::ArrayInput(
            "Alpha", gc.controller().robot().refJointOrder(), filtAlpha),
        mc_rtc::gui::ArrayInput("Beta", gc.controller().robot().refJointOrder(),
                                filtBeta),
        mc_rtc::gui::ArrayInput("Torque",
                                gc.controller().robot().refJointOrder(), tau_r),
        mc_rtc::gui::ArrayInput("Tau from current measurement",
                                gc.controller().robot().refJointOrder(),
                                m_torque_from_current_measurement),
        mc_rtc::gui::ArrayInput("Tau sensor",
                                gc.controller().robot().refJointOrder(),
                                m_tau_sensor),
        mc_rtc::gui::ArrayInput("Tau error",
                                gc.controller().robot().refJointOrder(),
                                m_torque_error));
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
  mc_rtc::log::success("[mc_kortex] Removed GUI");
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
    double q = jointPoseToRad(i, m_state.mutable_actuators(i)->position());
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
  mc_rtc::log::error("[mc_kortex] KError error_code: {}", err.error_code());
  mc_rtc::log::error("[mc_kortex] KError sub_code: {}", err.error_sub_code());
  mc_rtc::log::error("[mc_kortex] KError sub_string: {}",
                     err.error_sub_string());

  // Error codes by themselves are not very verbose if you don't see their
  // corresponding enum value You can use google::protobuf helpers to get the
  // string enum element for every error code and sub-code
  mc_rtc::log::error(
      "[mc_kortex] Error code string equivalent: {}",
      k_api::ErrorCodes_Name(k_api::ErrorCodes(err.error_code())));
  mc_rtc::log::error(
      "[mc_kortex] Error sub-code string equivalent: {}",
      k_api::SubErrorCodes_Name(k_api::SubErrorCodes(err.error_sub_code())));
}

void KinovaRobot::printException(k_api::KDetailedException &ex) {
  // You can print the error informations and error codes
  auto error_info = ex.getErrorInfo().getError();
  mc_rtc::log::error("[mc_kortex] KDetailedException detected : {}", ex.what());

  printError(error_info);
}

std::function<void(k_api::Base::ActionNotification)>
KinovaRobot::check_for_end_or_abort(bool &finished) {
  return [&finished](k_api::Base::ActionNotification notification) {
    mc_rtc::log::info(
        "[mc_kortex] EVENT : {}",
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
