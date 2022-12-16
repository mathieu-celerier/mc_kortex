#include "KinovaRobot.h"

namespace mc_kinova
{

KinovaRobot::KinovaRobot(const std::string& name, const std::string& ip_address, const std::string& username = "admin", const std::string& password = "admin"):
m_name(name), m_ip_address(ip_address), m_port(10000), m_port_real_time(10001), m_username(username), m_password(password)
{
    m_router                    = nullptr;
    m_router_real_time          = nullptr;
    m_transport                 = nullptr;
    m_transport_real_time       = nullptr;
    m_session_manager           = nullptr;
    m_session_manager_real_time = nullptr;
    m_base                      = nullptr;
    m_base_cyclic               = nullptr;
    m_device_manager            = nullptr;
    m_actuator_config           = nullptr;

    m_state = k_api::BaseCyclic::Feedback();
    m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
    m_control_mode_id = 0;
    m_prev_control_mode_id = 0;
    m_use_custom_torque_control = false;
}

KinovaRobot::~KinovaRobot()
{
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

std::vector<double> KinovaRobot::getJointPosition()
{
    std::vector<double> q(m_actuator_count);
    for (auto actuator : m_state.actuators()) q[jointIdFromCommandID(actuator.command_id())] = actuator.position();

    return q;
}

std::string KinovaRobot::getName(void)
{
    return m_name;
}

// ==================== Setter ==================== //

void KinovaRobot::setLowServoingMode()
{
    // Ignore if already in low level servoing mode
    // if(m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) return;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    m_base->SetServoingMode(servoingMode);
    m_servoing_mode = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
}

void KinovaRobot::setSingleServoingMode()
{
    // Ignore if already in "high" level servoing mode
    // if(m_servoing_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING) return;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    m_base->SetServoingMode(servoingMode);
    m_servoing_mode = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
}

void KinovaRobot::setCustomTorque(mc_rtc::Configuration & torque_config)
{
    m_use_custom_torque_control = true;
    if(torque_config.has("friction_compensation"))
    {
        if(torque_config("friction_compensation").has("compensation_values"))
        {
            m_friction_values = torque_config("friction_compensation")("compensation_values");
            if (!m_friction_values.size() == m_actuator_count) mc_rtc::log::error_and_throw<std::runtime_error>("[MC_KORTEX] for {} robot, value for \"compensation_values\" key does not match actuators count.\nActuators count = ",m_name,m_actuator_count);
        }
        else
        {
            m_friction_values = {2.8,2.8,2.8,2.8,1.8,1.8,1.8};
        }

        if(torque_config("friction_compensation").has("velocity_threshold"))
        {
            m_friction_vel_threshold = torque_config("friction_compensation")("velocity_threshold");
        }
        else
        {
            m_friction_vel_threshold = 0.1;
        }

        if(torque_config("friction_compensation").has("acceleration_threshold"))
        {
            m_friction_accel_threshold = torque_config("friction_compensation")("acceleration_threshold");
        }
        else
        {
            m_friction_accel_threshold = 100;
        }
    }
    else
    {
        m_friction_values = {2.8,2.8,2.8,2.8,1.8,1.8,1.8};
    }

    if(torque_config.has("integral_term"))
    {
        if(torque_config("integral_term").has("mu"))
        {
            m_mu = torque_config("integral_term")("mu");
        }
        else
        {
            m_mu = 0.9;
        }

        if(torque_config("integral_term").has("gains"))
        {
            m_integral_gains = torque_config("integral_term")("gains");
            if (!m_friction_values.size() == m_actuator_count) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_kortex] for {} robot, value for \"gains\" key does not match actuators count.\nActuators count = ",m_name,m_actuator_count);

        }
        else
        {
            m_integral_gains = {2.8,2.8,2.8,2.8,1.8,1.8,1.8};
        }
    }
    else
    {
        m_mu = 0.9;
        m_integral_gains = {2.8,2.8,2.8,2.8,1.8,1.8,1.8};
    }

    mc_rtc::log::info("[mc_kortex] {} robot is using custom torque control with parameters:",m_name);
}

void KinovaRobot::setControlMode(std::string mode)
{
    if(mode.compare("Position") == 0)
    {
        if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION) return;

        // mc_rtc::log::info("[mc_kortex] Using position control");
        m_control_mode = k_api::ActuatorConfig::ControlMode::POSITION;
        m_control_mode_id++;
        return;
    }
    if(mode.compare("Velocity") == 0)
    {
        if (m_control_mode == k_api::ActuatorConfig::ControlMode::VELOCITY) return;

        // mc_rtc::log::info("[mc_kortex] Using velocity control");
        m_control_mode = k_api::ActuatorConfig::ControlMode::VELOCITY;
        m_control_mode_id++;
        return;
    }
    if(mode.compare("Torque") == 0)
    {
        if (m_use_custom_torque_control)
        {
            if (m_control_mode == k_api::ActuatorConfig::ControlMode::CURRENT) return;

            // mc_rtc::log::info("[mc_kortex] Using torque control");
            initFiltersBuffers();
            m_control_mode = k_api::ActuatorConfig::ControlMode::CURRENT;
            m_control_mode_id++;
            return;
        }
        else
        {
            if (m_control_mode == k_api::ActuatorConfig::ControlMode::TORQUE) return;

            // mc_rtc::log::info("[mc_kortex] Using torque control");
            m_control_mode = k_api::ActuatorConfig::ControlMode::TORQUE;
            m_control_mode_id++;
            return;
        }
    }
}

// ==================== Public functions ==================== //

void KinovaRobot::init(mc_control::MCGlobalController & gc, mc_rtc::Configuration & kortexConfig)
{
    mc_rtc::log::info("[mc_kortex] Initializing connection to the robot at {}:{}",m_ip_address, m_port);

    auto error_callback = [](k_api::KError err){ mc_rtc::log::error("_________ callback error _________ {}",err.toString()); };

    // Initiate connection
    m_transport = new k_api::TransportClientTcp();
    m_router = new k_api::RouterClient(m_transport, error_callback);
    m_transport->connect(m_ip_address, m_port);

    m_transport_real_time = new k_api::TransportClientUdp();
    m_router_real_time = new k_api::RouterClient(m_transport_real_time, error_callback);
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

    // Set control mode
    auto controle_mode = k_api::ActuatorConfig::ControlModeInformation();
    controle_mode.set_control_mode(m_control_mode);
    for(int i = 0; i < m_actuator_count; i++) m_actuator_config->SetControlMode(controle_mode,i+1);

    // Init pose if desired
    if(kortexConfig.has("init_posture"))
    {
        if(kortexConfig("init_posture").has("posture"))
        {
            m_init_posture = kortexConfig("init_posture")("posture");
            if (!m_init_posture.size() == m_actuator_count) mc_rtc::log::error_and_throw<std::runtime_error>("[MC_KORTEX] for {} robot, value for \"posture\" key does not match actuators count.\nActuators count = ",m_name,m_actuator_count);
        }

        if(kortexConfig("init_posture")("on_startup",false))
        {
            moveToInitPosition();
        }
    }
    else
    {
        m_init_posture.resize(m_actuator_count);

        auto joints_feedback = m_base->GetMeasuredJointAngles();
        for (size_t i = 0; i < m_actuator_count; i++)
        {
            auto joint_feedback = joints_feedback.joint_angles(i);
            m_init_posture[joint_feedback.joint_identifier()-1] = joint_feedback.value();
        }
    }

    // Initialize state
    updateState();
    updateSensors(gc);

    // Velocity filtering init
    m_use_filtered_velocities = kortexConfig.has("filter_velocity");
    if(m_use_filtered_velocities)
    {
        m_velocity_filter_ratio = kortexConfig("filter_velocity")("ratio",0.0);
        mc_rtc::log::info("[mc_kortex] Filtering velocities for {} robot with {} ratio",m_name,m_velocity_filter_ratio);
    }
    m_filtered_velocities.assign(m_actuator_count,0.0);

    // Custom torque control init
    if (kortexConfig.has("torque_control"))
    {
        kortexConfig = kortexConfig("torque_control");
        if(!kortexConfig.has("mode")) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_kortex] For {} robot, \"torque_control\" key found in config file but \"mode\" key is missing.",m_name);
        
        std::string controle_mode = kortexConfig("mode");
        if (controle_mode.compare("custom") == 0)
        {
            setCustomTorque(kortexConfig);
        }
    }

    // Initialize Jacobian object
    auto robot = &gc.robots().robot(m_name);
    m_jac = rbd::Jacobian(robot->mb(),"tool_frame");

    // Initialize each actuator to its current position
    for(int i = 0; i < m_actuator_count; i++) m_base_command.add_actuators()->set_position(m_state.actuators(i).position());

    m_filter_command.assign(m_actuator_count,0.0);
    m_current_command.assign(m_actuator_count,0.0);
    m_torque_error_sum.assign(m_actuator_count,0.0);
    m_torque_error.assign(m_actuator_count,0.0);
    m_integral_fast_filter.assign(m_actuator_count,0.0);
    m_integral_slow_filter.assign(m_actuator_count,0.0);
    m_integral_term_command.assign(m_actuator_count,0.0);
    m_friction_compensation_mode.assign(m_actuator_count,0.0);
    m_jac_transpose_f.assign(m_actuator_count,0.0);
    m_offsets.assign(m_actuator_count,0.0);

    addGui(gc);

    mc_rtc::log::success("[mc_kortex] Connected succesfuly to robot at {}:{}",m_ip_address, m_port);
}

void KinovaRobot::addLogEntry(mc_control::MCGlobalController & gc)
{
    gc.controller().logger().addLogEntry("kortex_LoopPerf", [&, this]() {return m_dt;});

    if(m_use_custom_torque_control)
    {
        gc.controller().logger().addLogEntry("kortex_friction_velocity_threshold", [this]() {return m_friction_vel_threshold;});
        gc.controller().logger().addLogEntry("kortex_friction_acceleration_threshold", [this]() {return m_friction_accel_threshold;});
        gc.controller().logger().addLogEntry("kortex_friction_compensation_values", [this]() {return m_friction_values;});
        gc.controller().logger().addLogEntry("kortex_friction_mode", [this]() {return m_friction_compensation_mode;});

        gc.controller().logger().addLogEntry("kortex_torque_error", [this]() {return m_torque_error;});
        gc.controller().logger().addLogEntry("kortex_integral_term", [this]() {return m_torque_error_sum;});
        gc.controller().logger().addLogEntry("kortex_integral_fast_filtered_integral", [this]() {return m_integral_fast_filter;});
        gc.controller().logger().addLogEntry("kortex_integral_slow_filtered_integral", [this]() {return m_integral_slow_filter;});
        gc.controller().logger().addLogEntry("kortex_integral_mixed_term", [this]() {return m_integral_term_command;});
        gc.controller().logger().addLogEntry("kortex_integral_gains", [this]() {return m_integral_gains;});
        gc.controller().logger().addLogEntry("kortex_integral_mu", [this]() {return m_mu;});

        gc.controller().logger().addLogEntry("kortex_current_output", [this]() {return m_current_command;});

        gc.controller().logger().addLogEntry("kortex_jac_transpose_F", [this]() {return m_jac_transpose_f;});
        gc.controller().logger().addLogEntry("kortex_posture_task_offset", [this]() {return m_offsets;});
    }
}

void KinovaRobot::removeLogEntry(mc_control::MCGlobalController & gc)
{
    gc.controller().logger().removeLogEntry("kortexLoopPerf");

    if(m_use_custom_torque_control)
    {
        gc.controller().logger().removeLogEntry("kortex_friction_velocity_threshold");
        gc.controller().logger().removeLogEntry("kortex_friction_acceleration_threshold");
        gc.controller().logger().removeLogEntry("kortex_friction_compensation_values");
        gc.controller().logger().removeLogEntry("kortex_friction_mode");
        gc.controller().logger().removeLogEntry("kortex_torque_error");
        gc.controller().logger().removeLogEntry("kortex_integral_term");
        gc.controller().logger().removeLogEntry("kortex_integral_fast_filtered_integral");
        gc.controller().logger().removeLogEntry("kortex_integral_slow_filtered_integral");
        gc.controller().logger().removeLogEntry("kortex_integral_mixed_term");
        gc.controller().logger().removeLogEntry("kortex_integral_gains");
        gc.controller().logger().removeLogEntry("kortex_integral_mu");
        gc.controller().logger().removeLogEntry("kortex_current_output");
        gc.controller().logger().removeLogEntry("kortex_current_output");
        gc.controller().logger().removeLogEntry("kortex_jac_transpose_F");
        gc.controller().logger().removeLogEntry("kortex_posture_task_offset");
    }
}

void KinovaRobot::updateState()
{
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    m_state = m_base_cyclic->RefreshFeedback();
}

void KinovaRobot::updateState(bool & running)
{
    m_base_cyclic->RefreshFeedback_callback(
        [&, this](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            updateState(data);
            checkBaseFaultBanks(data.base().fault_bank_a(),data.base().fault_bank_b());
            // checkActuatorsFaultBanks(data);
            if(err.error_code() != k_api::ErrorCodes::ERROR_NONE)
            {
                printError(err);
                running = false;
            }
        }
    );
}

void KinovaRobot::updateState(const k_api::BaseCyclic::Feedback data)
{
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    m_state = data;
}

double KinovaRobot::currentTorqueControlLaw(mc_rbdyn::Robot & robot, k_api::BaseCyclic::Feedback m_state_local, Eigen::MatrixXd jacobian, double joint_idx)
{
    auto rjo = robot.refJointOrder();

    double vel = mc_rtc::constants::toRad(m_state_local.mutable_actuators(joint_idx)->velocity());
    double tau_m = m_state_local.mutable_actuators(joint_idx)->torque();

    double kt = (joint_idx > 3) ? 0.076 : 0.11;
    auto x = m_filter_input_buffer[joint_idx];
    auto y = m_filter_output_buffer[joint_idx];
    double static_friction = 0.0;
    
    double tau_d = m_command.jointTorque[robot.jointIndexByName(rjo[joint_idx])][0];
    double tau_error = tau_d + tau_m;
    m_torque_error[joint_idx] = tau_error;

    // auto fx = m_state.base().tool_external_wrench_force_x();
    // auto fy = m_state.base().tool_external_wrench_force_y();
    // auto fz = m_state.base().tool_external_wrench_force_z();
    // auto cx = m_state.base().tool_external_wrench_torque_x();
    // auto cy = m_state.base().tool_external_wrench_torque_y();
    // auto cz = m_state.base().tool_external_wrench_torque_z();
    // auto qdd_external_force = jacobian.transpose()*sva::ForceVecd(Eigen::Vector3d(cx,cy,cz),Eigen::Vector3d(fx,fy,fz)).vector();

    // auto qdd_i_val = qdd_external_force[joint_idx] + m_command.alphaD[robot.jointIndexByName(rjo[joint_idx])][0];
    auto qdd_i_val = m_command.alphaD[robot.jointIndexByName(rjo[joint_idx])][0];
    // m_jac_transpose_f[joint_idx] = qdd_external_force[joint_idx];

    if (vel > m_friction_vel_threshold)
    {
        m_friction_compensation_mode[joint_idx] = 2;
        static_friction = m_friction_values[joint_idx];
    }    
    else if (vel < -m_friction_vel_threshold)
    {
        m_friction_compensation_mode[joint_idx] = -2;
        static_friction = -m_friction_values[joint_idx];
    }
    else
    {
        if (qdd_i_val > m_friction_accel_threshold)
        {
            m_friction_compensation_mode[joint_idx] = 1;
            static_friction = m_friction_values[joint_idx]*((2/(1+exp(-10.5866096494 *vel)))-1);
        }
        else if (qdd_i_val < -m_friction_accel_threshold)
        {
            m_friction_compensation_mode[joint_idx] = -1;
            static_friction = m_friction_values[joint_idx]*((2/(1+exp(-10.5866096494*vel)))-1);
        }
    }

    m_filter_command[joint_idx] = 2.864001*y[0] - 2.730119*y[1] + 0.8661156*y[2] + 0.01993631*tau_error - 0.01862187*x[0] - 0.01993022*x[1] + 0.01862796*x[2];
    double saturated_filter_command = max(min(m_filter_command[joint_idx]*0.1, 4.0), -4.0);

    m_filter_input_buffer[joint_idx].push_front(tau_error);
    m_filter_output_buffer[joint_idx].push_front(m_filter_command[joint_idx]);

    // double current = (tau_d + static_friction + m_torque_error_sum[joint_idx])/(GEAR_RATIO*kt);
    double current = saturated_filter_command + (tau_d + static_friction)/(GEAR_RATIO*kt);
    m_current_command[joint_idx] = current;
    return current;
}

bool KinovaRobot::sendCommand(mc_rbdyn::Robot & robot, bool & running)
{
    bool return_value = true;
    k_api::BaseCyclic::Feedback m_state_local;
    {
        std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
        m_state_local = m_state;
    }

    std::unique_lock<std::mutex> lock(m_update_control_mutex);
    auto rjo = robot.refJointOrder();

    if(m_control_id == m_prev_control_id) return false;

    auto lambda_fct = [&, this](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
    {
        updateState(data);
        checkBaseFaultBanks(data.base().fault_bank_a(),data.base().fault_bank_b());
        // checkActuatorsFaultBanks(data);
        if(err.error_code() != k_api::ErrorCodes::ERROR_NONE)
        {
            printError(err);
            running = false;
        }
    };

    for(size_t i = 0; i < m_actuator_count; i++)
    {
        if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION)
        {
            m_base_command.mutable_actuators(i)->set_position(radToJointPose(i,m_command.q[robot.jointIndexByName(rjo[i])][0]));
            continue;
        }
        else
        {
            m_base_command.mutable_actuators(i)->set_position(m_state_local.mutable_actuators(i)->position());
            // m_base_command.mutable_actuators(i)->set_position(radToJointPose(i,m_command.q[robot.jointIndexByName(rjo[i])][0]));
        }

        // if (m_control_mode == k_api::ActuatorConfig::ControlMode::VELOCITY)
        // {
        //     m_base_command.mutable_actuators(i)->set_velocity(mc_rtc::constants::toDeg(m_command.alpha[robot.jointIndexByName(rjo[i])][0]));
        //     continue;
        // }
        // else
        // {
        //     m_base_command.mutable_actuators(i)->set_velocity(mc_rtc::constants::toDeg(m_command.alpha[robot.jointIndexByName(rjo[i])][0]));
        // }

        if(m_use_custom_torque_control)
        {
            m_base_command.mutable_actuators(i)->set_current_motor(currentTorqueControlLaw(robot,m_state_local,m_jac.jacobian(robot.mb(),robot.mbc()),i));
        }
        else
        {
            m_base_command.mutable_actuators(i)->set_torque_joint(m_command.jointTorque[robot.jointIndexByName(rjo[i])][0]);
        }
        // std::cout << m_base_command.mutable_actuators(i)->position() << " " << m_state_local.mutable_actuators(i)->position() << " | ";
    }
    // if (m_control_mode != k_api::ActuatorConfig::ControlMode::POSITION) std::cout << std::endl;

    // ========================= Control mode has changed in mc_rtc, change it for the robot ========================= //
    if (m_control_mode_id != m_prev_control_mode_id)
    {
        auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
        control_mode.set_control_mode(m_control_mode);

        try
        {
            mc_rtc::log::info("[mc_kortex] Changing robot control mode to {} ", m_control_mode);
            for(int i = 0; i < m_actuator_count; i++)
            {
                // printJointActiveControlLoop(i+1);
                m_actuator_config->SetControlMode(control_mode,i+1);
                // printJointActiveControlLoop(i+1);
            }
            m_prev_control_mode_id = m_control_mode_id;
        }
        catch(k_api::KDetailedException& ex)
        {
            printException(ex);
            return_value = false;
            running = false;
        }
    }

    try
    {
        m_base_cyclic->Refresh_callback(m_base_command,lambda_fct,0);
        return_value = true;
    }
    catch(k_api::KDetailedException& ex)
    {
        printException(ex);
        return_value = false;
        running = false;
    }

    m_prev_control_id = m_control_id;
    return return_value;
}

void KinovaRobot::updateSensors(mc_control::MCGlobalController & gc)
{
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    auto & robot = gc.controller().robots().robot(m_name);
    auto rjo = robot.refJointOrder();

    // Trick for getting continuous joint to take the shortest path to the target
    auto has_posture_task = gc.controller().datastore().has("getPostureTask");
    auto posture_task_pt = (has_posture_task) ? gc.controller().datastore().call<mc_tasks::PostureTaskPtr>("getPostureTask") : nullptr;
    m_offsets = computePostureTaskOffset(robot, posture_task_pt);

    std::vector<double> q(m_actuator_count);
    std::vector<double> qdot(m_actuator_count);
    std::vector<double> tau(m_actuator_count);
    std::map<std::string,sva::ForceVecd> wrenches;
    double fx,fy,fz,cx,cy,cz;
    // std::map<std::string,double> current;
    // std::map<std::string,double> temp;

    for(size_t i = 0; i < m_actuator_count; i++)
    {
        q[i]  = jointPoseToRad(i,m_state.mutable_actuators(i)->position()) + m_offsets[i];
        if (m_use_filtered_velocities)
        {
            m_filtered_velocities[i] = m_velocity_filter_ratio*m_filtered_velocities[i] + (1-m_velocity_filter_ratio)*mc_rtc::constants::toRad(m_state.mutable_actuators(i)->velocity());
            qdot[i] = m_filtered_velocities[i];
        }
        else
        {
            qdot[i]  = mc_rtc::constants::toRad(m_state.mutable_actuators(i)->velocity());
        }
        tau[i]  = -m_state.mutable_actuators(i)->torque();
        // current[rjo[joint_idx]] = actuator.current_motor();
        // temp[rjo[joint_idx]] = actuator.temperature_motor();
    }

    fx = m_state.base().tool_external_wrench_force_x();
    fy = m_state.base().tool_external_wrench_force_y();
    fz = m_state.base().tool_external_wrench_force_z();
    cx = m_state.base().tool_external_wrench_torque_x();
    cy = m_state.base().tool_external_wrench_torque_y();
    cz = m_state.base().tool_external_wrench_torque_z();
    wrenches[robot.forceSensors()[0].name()] = sva::ForceVecd(Eigen::Vector3d(cx,cy,cz),Eigen::Vector3d(fx,fy,fz));

    gc.setEncoderValues(m_name,q);
    gc.setEncoderVelocities(m_name,qdot);
    gc.setJointTorques(m_name,tau);
    gc.setWrenches(wrenches);
    // gc.setJointMotorCurrents(m_name,current);
    // gc.setJointMotorTemperatures(m_name,temp);
}

void KinovaRobot::updateControl(mc_control::MCGlobalController & controller)
{
    std::unique_lock<std::mutex> lock(m_update_control_mutex);
    auto & robot = controller.controller().robots().robot(m_name);
    m_command = robot.mbc();
    m_control_id++;
}

std::string KinovaRobot::controlLoopParamToString(k_api::ActuatorConfig::LoopSelection & loop_selected, int actuator_idx)
{
    k_api::ActuatorConfig::ControlLoopParameters parameters = m_actuator_config->GetControlLoopParameters(loop_selected,actuator_idx);
    std::ostringstream ss;
    ss << "kAz = [";
    for (size_t i = 0; i < parameters.kaz_size()-1; i++) ss << parameters.kaz(i) << ",";
    ss << parameters.kaz(parameters.kaz_size()) << "] kBz = [";
    for (size_t i = 0; i < parameters.kbz_size()-1; i++) ss << parameters.kbz(i) << ",";
    ss << parameters.kbz(parameters.kbz_size()) << "]";

    return ss.str();
}

void KinovaRobot::checkBaseFaultBanks(uint32_t fault_bank_a, uint32_t fault_bank_b)
{
    if (fault_bank_a != 0)
    {
        auto error_list = getBaseFaultList(fault_bank_a);
        std::ostringstream ss;
        ss << "[";
        std::copy(error_list.begin(),error_list.end()-1, std::ostream_iterator<std::string>(ss, ", "));
        ss << error_list.back() << "]";
        mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank A : {}",ss.str());
    }
    if (fault_bank_b != 0)
    {
        auto error_list = getBaseFaultList(fault_bank_b);
        std::ostringstream ss;
        ss << "[";
        std::copy(error_list.begin(),error_list.end()-1, std::ostream_iterator<std::string>(ss, ", "));
        ss << error_list.back() << "]";
        mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank B : {}",ss.str());
    }
}

void KinovaRobot::checkActuatorsFaultBanks(k_api::BaseCyclic::Feedback feedback)
{
    for(size_t i = 0; i < m_actuator_count; i++)
    {
        if (feedback.mutable_actuators(i)->fault_bank_a() != 0)
        {
            auto error_list = getActuatorFaultList(feedback.mutable_actuators(i)->fault_bank_a());
            std::ostringstream ss;
            ss << "[";
            std::copy(error_list.begin(),error_list.end()-1, std::ostream_iterator<std::string>(ss, ", "));
            ss << error_list.back() << "]";
            mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank A : {}",ss.str());
        }
        if (feedback.mutable_actuators(i)->fault_bank_b() != 0)
        {
            auto error_list = getActuatorFaultList(feedback.mutable_actuators(i)->fault_bank_b());
            std::ostringstream ss;
            ss << "[";
            std::copy(error_list.begin(),error_list.end()-1, std::ostream_iterator<std::string>(ss, ", "));
            ss << error_list.back() << "]";
            mc_rtc::log::error_and_throw("[MC_KORTEX] Error in base fault bank B : {}",ss.str());
        }
        
    }
}

std::vector<std::string> KinovaRobot::getBaseFaultList(uint32_t fault_bank)
{
    std::vector<string> fault_list;
    if (fault_bank & (uint32_t)0x1) fault_list.push_back("FIRMWARE_UPDATE_FAILURE");
    if (fault_bank & (uint32_t)0x2) fault_list.push_back("EXTERNAL_COMMUNICATION_ERROR");
    if (fault_bank & (uint32_t)0x4) fault_list.push_back("MAXIMUM_AMBIENT_TEMPERATURE");
    if (fault_bank & (uint32_t)0x8) fault_list.push_back("MAXIMUM_CORE_TEMPERATURE");
    if (fault_bank & (uint32_t)0x10) fault_list.push_back("JOINT_FAULT");
    if (fault_bank & (uint32_t)0x20) fault_list.push_back("CYCLIC_DATA_JITTER");
    if (fault_bank & (uint32_t)0x40) fault_list.push_back("REACHED_MAXIMUM_EVENT_LOGS");
    if (fault_bank & (uint32_t)0x80) fault_list.push_back("NO_KINEMATICS_SUPPORT");
    if (fault_bank & (uint32_t)0x100) fault_list.push_back("ABOVE_MAXIMUM_DOF");
    if (fault_bank & (uint32_t)0x200) fault_list.push_back("NETWORK_ERROR");
    if (fault_bank & (uint32_t)0x400) fault_list.push_back("UNABLE_TO_REACH_POSE");
    if (fault_bank & (uint32_t)0x800) fault_list.push_back("JOINT_DETECTION_ERROR");
    if (fault_bank & (uint32_t)0x1000) fault_list.push_back("NETWORK_INITIALIZATION_ERROR");
    if (fault_bank & (uint32_t)0x2000) fault_list.push_back("MAXIMUM_CURRENT");
    if (fault_bank & (uint32_t)0x4000) fault_list.push_back("MAXIMUM_VOLTAGE");
    if (fault_bank & (uint32_t)0x8000) fault_list.push_back("MINIMUM_VOLTAGE");
    if (fault_bank & (uint32_t)0x10000) fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_VELOCITY");
    if (fault_bank & (uint32_t)0x20000) fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_VELOCITY");
    if (fault_bank & (uint32_t)0x40000) fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_ACCELERATION");
    if (fault_bank & (uint32_t)0x80000) fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_ACCELERATION");
    if (fault_bank & (uint32_t)0x100000) fault_list.push_back("MAXIMUM_END_EFFECTOR_TRANSLATION_FORCE");
    if (fault_bank & (uint32_t)0x200000) fault_list.push_back("MAXIMUM_END_EFFECTOR_ORIENTATION_FORCE");
    if (fault_bank & (uint32_t)0x400000) fault_list.push_back("MAXIMUM_END_EFFECTOR_PAYLOAD");
    if (fault_bank & (uint32_t)0x800000) fault_list.push_back("EMERGENCY_STOP_ACTIVATED");
    if (fault_bank & (uint32_t)0x1000000) fault_list.push_back("EMERGENCY_LINE_ACTIVATED");
    if (fault_bank & (uint32_t)0x2000000) fault_list.push_back("INRUSH_CURRENT_LIMITER_FAULT");
    if (fault_bank & (uint32_t)0x4000000) fault_list.push_back("NVRAM_CORRUPTED");
    if (fault_bank & (uint32_t)0x8000000) fault_list.push_back("INCOMPATIBLE_FIRMWARE_VERSION");
    if (fault_bank & (uint32_t)0x10000000) fault_list.push_back("POWERON_SELF_TEST_FAILURE");
    if (fault_bank & (uint32_t)0x20000000) fault_list.push_back("DISCRETE_INPUT_STUCK_ACTIVE");
    if (fault_bank & (uint32_t)0x40000000) fault_list.push_back("ARM_INTO_ILLEGAL_POSITION");
    return fault_list;
}

std::vector<std::string> KinovaRobot::getActuatorFaultList(uint32_t fault_bank)
{
    std::vector<string> fault_list;
    if (fault_bank & (uint32_t)0x1) fault_list.push_back("FOLLOWING_ERROR");
    if (fault_bank & (uint32_t)0x2) fault_list.push_back("MAXIMUM_VELOCITY");
    if (fault_bank & (uint32_t)0x4) fault_list.push_back("JOINT_LIMIT_HIGH");
    if (fault_bank & (uint32_t)0x8) fault_list.push_back("JOINT_LIMIT_LOW");
    if (fault_bank & (uint32_t)0x10) fault_list.push_back("STRAIN_GAUGE_MISMATCH");
    if (fault_bank & (uint32_t)0x20) fault_list.push_back("MAXIMUM_TORQUE");
    if (fault_bank & (uint32_t)0x40) fault_list.push_back("UNRELIABLE_ABSOLUTE_POSITION");
    if (fault_bank & (uint32_t)0x80) fault_list.push_back("MAGNETIC_POSITION");
    if (fault_bank & (uint32_t)0x100) fault_list.push_back("HALL_POSITION");
    if (fault_bank & (uint32_t)0x200) fault_list.push_back("HALL_SEQUENCE");
    if (fault_bank & (uint32_t)0x400) fault_list.push_back("INPUT_ENCODER_HALL_MISMATCH");
    if (fault_bank & (uint32_t)0x800) fault_list.push_back("INPUT_ENCODER_INDEX_MISMATCH");
    if (fault_bank & (uint32_t)0x1000) fault_list.push_back("INPUT_ENCODER_MAGNETIC_MISMATCH");
    if (fault_bank & (uint32_t)0x2000) fault_list.push_back("MAXIMUM_MOTOR_CURRENT");
    if (fault_bank & (uint32_t)0x4000) fault_list.push_back("MOTOR_CURRENT_MISMATCH");
    if (fault_bank & (uint32_t)0x8000) fault_list.push_back("MAXIMUM_VOLTAGE");
    if (fault_bank & (uint32_t)0x10000) fault_list.push_back("MINIMUM_VOLTAGE");
    if (fault_bank & (uint32_t)0x20000) fault_list.push_back("MAXIMUM_MOTOR_TEMPERATURE");
    if (fault_bank & (uint32_t)0x40000) fault_list.push_back("MAXIMUM_CORE_TEMPERATURE");
    if (fault_bank & (uint32_t)0x80000) fault_list.push_back("NON_VOLATILE_MEMORY_CORRUPTED");
    if (fault_bank & (uint32_t)0x100000) fault_list.push_back("MOTOR_DRIVER_FAULT");
    if (fault_bank & (uint32_t)0x200000) fault_list.push_back("EMERGENCY_LINE_ASSERTED");
    if (fault_bank & (uint32_t)0x400000) fault_list.push_back("COMMUNICATION_TICK_LOST");
    if (fault_bank & (uint32_t)0x800000) fault_list.push_back("WATCHDOG_TRIGGERED");
    if (fault_bank & (uint32_t)0x1000000) fault_list.push_back("UNRELIABLE_CAPACITIVE_SENSOR");
    if (fault_bank & (uint32_t)0x2000000) fault_list.push_back("UNEXPECTED_GEAR_RATIO");
    if (fault_bank & (uint32_t)0x4000000) fault_list.push_back("HALL_MAGNETIC_MISMATCH");
    return fault_list;
}

void KinovaRobot::controlThread(mc_control::MCGlobalController & controller, std::mutex & startM, std::condition_variable & startCV, bool & start, bool & running)
{
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

    try
    {

        while(running)
        {
            now = GetTickUs();
            if (now - last < 1000) continue;
            dt = now - last;
            last = now;
            
            if(m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
            {
                sendCommand(controller.robots().robot(m_name),running);
            }
            else
            {
                mc_rtc::log::info("high level servoing");
                // updateState(running);
            }
        }
        
        removeLogEntry(controller);

        mc_rtc::log::warning("[MC_KORTEX] {} control loop killed",m_name);

        return_status = true;
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    auto control_mode = k_api::ActuatorConfig::ControlModeInformation();
    control_mode.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

    for(int i = 0; i < m_actuator_count; i++)
    {
        // printJointActiveControlLoop(i+1);
        m_actuator_config->SetControlMode(control_mode,i+1);
        // printJointActiveControlLoop(i+1);
    }

    setSingleServoingMode();

    removeGui(controller);
}

void KinovaRobot::moveToHomePosition()
{
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
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        mc_rtc::log::warning("[mc_kortex] Can't reach safe position, exiting");
    }
    else 
    {
        bool action_finished = false; 
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = m_base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        m_base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        { 
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        m_base->Unsubscribe(notification_handle);
    }
}

void KinovaRobot::moveToInitPosition()
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    setSingleServoingMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create trajectory object
    k_api::Base::WaypointList wpts = k_api::Base::WaypointList();
    
    // Define joint poses
    auto jointPoses = std::vector<std::array<float,7>>();
    std::array<float, 7> arr;
    for(size_t i = 0; i < min(m_actuator_count,7); i++) arr[i] = radToJointPose(i, m_init_posture[i]);
    jointPoses.push_back(arr);

    // Add initial pose as a waypoint
    k_api::Base::Waypoint *wpt = wpts.add_waypoints();
    wpt->set_name("waypoint_0");
    k_api::Base::AngularWaypoint *ang = wpt->mutable_angular_waypoint();
    for (size_t i = 0; i < m_actuator_count; i++)
    {
        ang->add_angles(jointPoses.at(0).at(i));
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> finish_promise_cart;
    auto finish_future_cart = finish_promise_cart.get_future();
    auto promise_notification_handle_cart = m_base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise_cart),
        k_api::Common::NotificationOptions()
    );

    k_api::Base::WaypointValidationReport result;
    try
    {
        // Verify validity of waypoints
        auto validationResult = m_base->ValidateWaypointList(wpts);
        result = validationResult;
    }
    catch(k_api::KDetailedException& ex)
    {
        mc_rtc::log::error("[mc_kortex] Error on waypoint list to reach initial position");
        printException(ex);
        return;
    }

    // Trajectory error report always exists and we need to make sure no elements are found in order to validate the trajectory
    if(result.trajectory_error_report().trajectory_error_elements_size() == 0)
    {
        // Execute action
        try
        {
            mc_rtc::log::info("[mc_kortex] Moving the arm to initial position");
        }
        catch(k_api::KDetailedException& ex)
        {
            mc_rtc::log::error("[mc_kortex] Error when trying to execute trajectory to reach initial position");
            printException(ex);
            return;
        }
        // Wait for future value from promise
        const auto ang_status = finish_future_cart.wait_for(std::chrono::seconds(100));
        m_base->Unsubscribe(promise_notification_handle_cart);
        if (ang_status != std::future_status::ready)
        {
            mc_rtc::log::warning("[mc_kortex] Timeout when trying to reach initial position, try again");
        }
        else
        {
            const auto ang_promise_event = finish_future_cart.get();
            mc_rtc::log::success("[mc_kortex] Angular waypoint trajectory completed");
        }
    }
    else
    {
        mc_rtc::log::error("[mc_kortex] Error found in trajectory to initial position");
        mc_rtc::log::error(result.trajectory_error_report().DebugString());
    }
}

void KinovaRobot::printState()
{
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(m_state, &serialized_data);
    std::cout << serialized_data << std::endl;
}

void KinovaRobot::printJointActiveControlLoop(int joint_id)
{
    uint32_t control_loop;
    std::vector<std::string> active_loops;
    control_loop = m_actuator_config->GetActivatedControlLoop(joint_id).control_loop();
    if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::JOINT_POSITION) active_loops.push_back("JOINT_POSITION");
    if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::JOINT_TORQUE) active_loops.push_back("JOINT_TORQUE");
    if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::JOINT_TORQUE_HIGH_VELOCITY) active_loops.push_back("JOINT_TORQUE_HIGH_VELOCITY");
    if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::JOINT_VELOCITY) active_loops.push_back("JOINT_VELOCITY");
    if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::MOTOR_CURRENT) active_loops.push_back("MOTOR_CURRENT");
    if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::MOTOR_POSITION) active_loops.push_back("MOTOR_POSITION");
    if (control_loop & k_api::ActuatorConfig::ControlLoopSelection::MOTOR_VELOCITY) active_loops.push_back("MOTOR_VELOCITY");
 
    std::ostringstream ss;
    ss << "[";
    std::copy(active_loops.begin(),active_loops.end()-1, std::ostream_iterator<std::string>(ss, ", "));
    ss << active_loops.back() << "]";

    mc_rtc::log::info("[mc_kortex][Joint {}] Active control loops: {}",joint_id,ss.str());
}

// ============================== Private methods ============================== //

void KinovaRobot::initFiltersBuffers()
{
    m_filter_input_buffer.assign(m_actuator_count,boost::circular_buffer<double>(3,0.0));
    m_filter_output_buffer.assign(m_actuator_count,boost::circular_buffer<double>(3,0.0));
}

void KinovaRobot::addGui(mc_control::MCGlobalController & gc)
{
    gc.controller().gui()->addElement({"Kortex",m_name},
        mc_rtc::gui::Button(
            "Move to initial position",
            [this]() {
                setSingleServoingMode();
                moveToInitPosition();
                setLowServoingMode();
            }
        )
    );

    gc.controller().gui()->addElement({"Kortex",m_name},
        mc_rtc::gui::ArrayLabel(
            "PostureTask offsets",
            gc.controller().robot().refJointOrder(),
            [this]() {
                return m_offsets;
            }
        )
    );

    if(m_use_custom_torque_control)
    {
        gc.controller().gui()->addElement({"Kortex",m_name},
            mc_rtc::gui::ArrayInput(
                "Integral gains",
                gc.controller().robot().refJointOrder(),
                [this]() { return m_integral_gains; },
                [this](const std::vector<double> & v) { m_integral_gains = v;}
            ),
            mc_rtc::gui::ArrayInput(
                "Friction compensation values",
                gc.controller().robot().refJointOrder(),
                [this]() { return m_friction_values; },
                [this](const std::vector<double> & v) { m_friction_values = v;}
            ),
            mc_rtc::gui::NumberInput(
                "Friction compensation velocity threshold",
                [this]() { return m_friction_vel_threshold; },
                [this](const double v) { m_friction_vel_threshold = v;}
            ),
            mc_rtc::gui::NumberInput(
                "Friction compensation acceleration threshold",
                [this]() { return m_friction_accel_threshold; },
                [this](const double v) { m_friction_accel_threshold = v;}
            ),
            mc_rtc::gui::NumberSlider(
                "Integral term blending, mu value:",
                [this]() { return m_mu; },
                [this](const double v) { m_mu = v;},
                0.001, 0.999
            )
        );
    }

    if(m_use_filtered_velocities)
    {
        gc.controller().gui()->addElement({"Kortex",m_name},
            mc_rtc::gui::NumberSlider(
                "Velocity filtering ratio (decay):",
                [this]() { return m_velocity_filter_ratio; },
                [this](const double v) { m_velocity_filter_ratio = v;},
                0.0, 1.0
            )
        );
    }
}

void KinovaRobot::removeGui(mc_control::MCGlobalController & gc)
{
    gc.controller().gui()->removeCategory({"Kortex"});
}

double KinovaRobot::jointPoseToRad(int joint_idx, double deg)
{
    return mc_rtc::constants::toRad( (deg < 180.0) ? deg : deg - 360 );
}

double KinovaRobot::radToJointPose(int joint_idx, double rad)
{
    return mc_rtc::constants::toDeg( (rad > 0) ? rad : 2*M_PI+rad );
}

std::vector<double> KinovaRobot::computePostureTaskOffset(mc_rbdyn::Robot & robot, mc_tasks::PostureTaskPtr posture_task)
{
    std::vector<double> offsets(m_actuator_count,0.0);
    
    // Posture task only in damping mode no need for offsets
    if(posture_task == nullptr) return offsets;
    if(posture_task->stiffness() == 0) return offsets;
    if(posture_task->posture().size() == 0) return offsets;

    auto rjo = robot.refJointOrder();
    
    auto target = posture_task->posture();
    // fmt::print("Posture task stiffness = {}\n",posture_task->stiffness());
    for (size_t i = 0; i < m_actuator_count; i++)
    {
        double target_pose = target[robot.jointIndexByName(rjo[i])][0];
        double q = jointPoseToRad(i,m_state.mutable_actuators(i)->position());
        // std::cout << target_pose << " " << q << " | ";
        if (i%2 == 0)
        {
            if (q > (target_pose + M_PI)) offsets[i] = -2*M_PI;
            else if (q < (target_pose - M_PI)) offsets[i] = 2*M_PI;
        }
    }
    // std::cout << "\n";
    return offsets;
}

uint32_t KinovaRobot::jointIdFromCommandID(google::protobuf::uint32 cmd_id)
{
    return (cmd_id >> 16) & 0x0000000F;
}

int64_t KinovaRobot::GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

void KinovaRobot::printError(const k_api::Error & err)
{   
    mc_rtc::log::error("[mc_kortex] KError error_code: {}",err.error_code());
    mc_rtc::log::error("[mc_kortex] KError sub_code: {}",err.error_sub_code());
    mc_rtc::log::error("[mc_kortex] KError sub_string: {}",err.error_sub_string());

    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
    mc_rtc::log::error("[mc_kortex] Error code string equivalent: {}",k_api::ErrorCodes_Name(k_api::ErrorCodes(err.error_code())));
    mc_rtc::log::error("[mc_kortex] Error sub-code string equivalent: {}",k_api::SubErrorCodes_Name(k_api::SubErrorCodes(err.error_sub_code())));
}

void KinovaRobot::printException(k_api::KDetailedException & ex)
{
    // You can print the error informations and error codes
    auto error_info = ex.getErrorInfo().getError();
    mc_rtc::log::error("[mc_kortex] KDetailedException detected : {}",ex.what());
    
    printError(error_info);
}

std::function<void(k_api::Base::ActionNotification)> KinovaRobot::check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        mc_rtc::log::info("[mc_kortex] EVENT : {}", k_api::Base::ActionEvent_Name(notification.action_event()));
        
        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}

std::function<void(k_api::Base::ActionNotification)> KinovaRobot::create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise_cart)
{
    return [&finish_promise_cart] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)                            
        {                                               
            case k_api::Base::ActionEvent::ACTION_END:      
            case k_api::Base::ActionEvent::ACTION_ABORT:    
                finish_promise_cart.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

std::string printVec(std::vector<double> vec)
{
    std::ostringstream s;
    s << "[";
    std::copy(vec.begin(),vec.end()-1, std::ostream_iterator<double>(s, ", "));
    s << vec.back() << "]";
    return s.str();
}

} // namespace mc_kinova