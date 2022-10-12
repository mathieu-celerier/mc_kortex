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

// ==================== Setter ==================== //

void KinovaRobot::setLowServoingMode()
{
    // Ignore if already in low level servoing mode
    if(m_servoing_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) return;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    m_base->SetServoingMode(servoingMode);
    m_servoing_mode = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
}

void KinovaRobot::setSingleServoingMode()
{
    // Ignore if already in "high" level servoing mode
    if(m_servoing_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING) return;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    m_base->SetServoingMode(servoingMode);
    m_servoing_mode = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
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
        if (m_control_mode == k_api::ActuatorConfig::ControlMode::TORQUE) return;
        // mc_rtc::log::info("[mc_kortex] Using torque control");
        m_control_mode = k_api::ActuatorConfig::ControlMode::TORQUE;
        m_control_mode_id++;
        return;
    }
}

// ==================== Public functions ==================== //

void KinovaRobot::init(mc_control::MCGlobalController & gc)
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

    // Initialize state
    updateState();
    updateSensors(gc);

    // Initialize each actuator to its current position
    for(int i = 0; i < m_actuator_count; i++) m_base_command.add_actuators()->set_position(m_state.actuators(i).position());

    mc_rtc::log::success("[mc_kortex] Connected succesfuly to robot at {}:{}",m_ip_address, m_port);
}

void KinovaRobot::updateState()
{
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    m_state = m_base_cyclic->RefreshFeedback();
}

void KinovaRobot::updateState(const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
{
    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    m_state = data;
}

bool KinovaRobot::sendCommand(mc_rbdyn::Robot & robot)
{
    bool return_value = true;
    std::unique_lock<std::mutex> lock(m_update_control_mutex);
    auto rjo = robot.refJointOrder();

    auto controle_mode = k_api::ActuatorConfig::ControlModeInformation();
    controle_mode.set_control_mode(m_control_mode);

    if(m_control_id == m_prev_control_id) return false;

    auto lambda_fct = [this](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
    {
        updateState(err,data);
    };

    if (m_control_mode_id != m_prev_control_mode_id)
    {
        try
        {
            mc_rtc::log::info("[mc_kortex] Changing robot control mode to {} ", m_control_mode);
            for(int i = 0; i < m_actuator_count; i++) m_actuator_config->SetControlMode(controle_mode,i+1);
            m_prev_control_mode_id = m_control_mode_id;
        }
        catch(k_api::KDetailedException& ex)
        {
            printException(ex);
            return_value = false;
        }
    }

    for(int i = 0; i < m_actuator_count; i++)
    {
        if (m_control_mode == k_api::ActuatorConfig::ControlMode::POSITION)
        {
            m_base_command.mutable_actuators(i)->set_position(radToJointPose(i,m_command.q[robot.jointIndexByName(rjo[i])][0]));
            continue;
        }
        else
        {
            m_base_command.mutable_actuators(i)->set_position(radToJointPose(i,m_command.q[robot.jointIndexByName(rjo[i])][0]));
        }

        if (m_control_mode == k_api::ActuatorConfig::ControlMode::VELOCITY)
        {
            m_base_command.mutable_actuators(i)->set_velocity(mc_rtc::constants::toDeg(m_command.alpha[robot.jointIndexByName(rjo[i])][0]));
            continue;
        }
        else
        {
            m_base_command.mutable_actuators(i)->set_velocity(mc_rtc::constants::toDeg(m_command.alpha[robot.jointIndexByName(rjo[i])][0]));
        }

        m_base_command.mutable_actuators(i)->set_torque_joint(-m_command.jointTorque[robot.jointIndexByName(rjo[i])][0]);
    }

    try
    {
        m_base_cyclic->Refresh_callback(m_base_command,lambda_fct,0);
        // mc_rtc::log::error("[mc_kortex] Sending command to robot {} ", printVec(command_rad));
        // mc_rtc::log::error_and_throw<std::runtime_error>("[mc_kortex] Sending command to robot {} ", printVec(command_deg));
        return_value = true;
    }
    catch(k_api::KDetailedException& ex)
    {
        printException(ex);
        return_value = false;
    }

    m_prev_control_id = m_control_id;
    return return_value;
}

void KinovaRobot::updateSensors(mc_control::MCGlobalController & gc)
{
    // If we are in control mode, we don't want to close loop on real robot state

    std::unique_lock<std::mutex> lock(m_update_sensor_mutex);
    auto & robot = gc.controller().robots().robot(m_name);
    auto rjo = robot.refJointOrder();

    std::vector<double> q(m_actuator_count);
    std::vector<double> qdot(m_actuator_count);
    std::vector<double> tau(m_actuator_count);
    std::map<std::string,sva::ForceVecd> wrenches;
    double fx,fy,fz,cx,cy,cz;
    // std::map<std::string,double> current;
    // std::map<std::string,double> temp;

    for(auto actuator : m_state.actuators())
    {
        uint8_t joint_idx = jointIdFromCommandID(actuator.command_id());
        q[joint_idx]  = jointPoseToRad(joint_idx,actuator.position());
        qdot[joint_idx]  = mc_rtc::constants::toRad(actuator.velocity());
        tau[joint_idx]  = actuator.torque();
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

void KinovaRobot::controlThread(mc_control::MCGlobalController & controller, std::mutex & startM, std::condition_variable & startCV, bool & start, bool & running)
{
    {
        std::unique_lock<std::mutex> lock(startM);
        startCV.wait(lock, [&]() { return start; });
    }

    setLowServoingMode();

    int64_t now = 0;
    int64_t last = 0;

    bool return_status;

    try
    {

        while(running)
        {
            now = GetTickUs();
            if (now - last < 1000) continue;

            sendCommand(controller.robots().robot(m_name));

            last = GetTickUs();
        }

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

void KinovaRobot::printState()
{
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(m_state, &serialized_data);
    std::cout << serialized_data << std::endl;
}

uint32_t KinovaRobot::jointIdFromCommandID(google::protobuf::uint32 cmd_id)
{
    return (cmd_id >> 16) & 0x0000000F;
}

// ============================== Private methods ============================== //

double KinovaRobot::jointPoseToRad(int joint_id, double deg)
{
    if (joint_id%2) 
    {
        // mc_rtc::log::info("[mc_kortex] sensor for joint {} set to {}rad from orginal value {}°",joint_id,mc_rtc::constants::toRad( (deg < 180.0) ? -deg : 360.0-deg ),deg);
        return mc_rtc::constants::toRad( (deg < 180.0) ? -deg : 360.0-deg );
    }
    else
    {
        // mc_rtc::log::info("[mc_kortex] sensor for joint {} set to {}rad from orginal value {}°",joint_id,mc_rtc::constants::toRad(deg),deg);
        return mc_rtc::constants::toRad(deg);
    }
}

double KinovaRobot::radToJointPose(int joint_id, double rad)
{
    if (joint_id%2) 
    {
        // mc_rtc::log::info("[mc_kortex] command for joint {} set to {}° from orginal value {}rad",joint_id,mc_rtc::constants::toDeg( (rad < 0) ? -rad : 2*M_PI-rad ),rad);
        return mc_rtc::constants::toDeg( (rad < 0) ? -rad : 2*M_PI-rad );
    }
    else
    {
        // mc_rtc::log::info("[mc_kortex] command for joint {} set to {}° from orginal value {}rad",joint_id,mc_rtc::constants::toDeg(rad),rad);
        return mc_rtc::constants::toDeg(rad);
    }
}

int64_t KinovaRobot::GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

void KinovaRobot::printException(k_api::KDetailedException & ex)
{
    // You can print the error informations and error codes
    auto error_info = ex.getErrorInfo().getError();
    mc_rtc::log::error("[mc_kortex] KDetailedException detected : {}",ex.what());
    
    std::cout << "[mc_kortex] KError error_code: " << error_info.error_code() << std::endl;
    std::cout << "[mc_kortex] KError sub_code: " << error_info.error_sub_code() << std::endl;
    std::cout << "[mc_kortex] KError sub_string: " << error_info.error_sub_string() << std::endl;

    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
    std::cout << "[mc_kortex] Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
    std::cout << "[mc_kortex] Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
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

std::string printVec(std::vector<double> vec)
{
    std::ostringstream s;
    s << "[";
    std::copy(vec.begin(),vec.end()-1, std::ostream_iterator<double>(s, ", "));
    s << vec.back() << "]";
    return s.str();
}

} // namespace mc_kinova