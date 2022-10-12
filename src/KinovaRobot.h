#include <mc_rtc/logging.h>
#include <mc_rbdyn/Robot.h>
#include <mc_control/mc_global_controller.h>

#include <chrono>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <InterconnectConfigClientRpc.h>
#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <ActuatorConfigClientRpc.h>

#include <google/protobuf/util/json_util.h>

namespace k_api = Kinova::Api;

namespace mc_kinova
{

class KinovaRobot
{
private:
    k_api::RouterClient*                                    m_router;
    k_api::TransportClientTcp*                              m_transport;
    k_api::RouterClient*                                    m_router_real_time;
    k_api::TransportClientUdp*                              m_transport_real_time;
    k_api::SessionManager*                                  m_session_manager;
    k_api::SessionManager*                                  m_session_manager_real_time;
    k_api::Base::BaseClient*                                m_base;
    k_api::BaseCyclic::BaseCyclicClient*                    m_base_cyclic;
    k_api::DeviceManager::DeviceManagerClient*              m_device_manager;
    k_api::ActuatorConfig::ActuatorConfigClient*            m_actuator_config;

    std::string m_username;
    std::string m_password;
    std::string m_ip_address;
    int m_port;
    int m_port_real_time;

    std::string m_name;
    int m_actuator_count;
    
    int m_control_id;
    int m_prev_control_id;
    std::mutex m_update_control_mutex;
    rbd::MultiBodyConfig m_command;
    k_api::BaseCyclic::Command m_base_command;

    std::mutex m_update_sensor_mutex;
    k_api::BaseCyclic::Feedback m_state;

    k_api::Base::ServoingMode m_servoing_mode;
    k_api::ActuatorConfig::ControlMode m_control_mode;
    int m_control_mode_id;
    int m_prev_control_mode_id;

public:
    KinovaRobot(const std::string& name, const std::string& ip_address, const std::string& username, const std::string& password);
    ~KinovaRobot();

    // ============================== Getter ============================== //
    std::vector<double> getJointPosition(void);

    // ============================== Setter ============================== //
    void setLowServoingMode(void);
    void setSingleServoingMode(void);
    void setControlMode(std::string mode);

    // Initialize connection to the robot
    void init(mc_control::MCGlobalController & gc);
    void updateState(void);
    void updateState(const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data);
    bool sendCommand(mc_rbdyn::Robot & robot);
    void updateSensors(mc_control::MCGlobalController & gc);
    void updateControl(mc_control::MCGlobalController & controller);

    void controlThread(mc_control::MCGlobalController & controller, std::mutex & startM, std::condition_variable & startCV, bool & start, bool & running);
    void moveToHomePosition(void);

    void printState(void);

    // ============================== Private methods ============================== //
private:
    double jointPoseToRad(int joint_id, double deg);
    double radToJointPose(int joint_id, double rad);
    uint32_t jointIdFromCommandID(google::protobuf::uint32 cmd_id);
    int64_t GetTickUs(void);
    void printException(k_api::KDetailedException & ex);
    std::function<void(k_api::Base::ActionNotification)> check_for_end_or_abort(bool& finished);
};

using KinovaRobotPtr = std::unique_ptr<KinovaRobot>;

std::string printVec(std::vector<double> vec);

} // namespace mc_kinova