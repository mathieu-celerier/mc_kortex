#include <mc_rtc/logging.h>
#include <mc_rbdyn/Robot.h>
#include <mc_control/mc_global_controller.h>

#include <chrono>
#include <boost/circular_buffer.hpp>

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

#define GEAR_RATIO 100.0

namespace k_api = Kinova::Api;

namespace mc_kinova
{

enum TorqueControlType
{
    Default,
    Feedforward,
    Custom
};

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

    int64_t m_dt;
    
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

    std::vector<double> m_init_posture;

    bool m_use_filtered_velocities;
    double m_velocity_filter_ratio;
    std::vector<double> m_filtered_velocities;

    // ===== Custom torque control properties =====
    TorqueControlType m_torque_control_type;

    std::vector<double> m_offsets;

    double m_mu;
    double m_friction_vel_threshold;
    double m_friction_accel_threshold;
    std::vector<double> m_friction_values;
    std::vector<double> m_friction_compensation_mode;

    std::vector<double> m_torque_error;

    std::vector<double> m_integral_gains;
    std::vector<double> m_torque_error_sum;
    std::vector<double> m_integral_fast_filter;
    std::vector<double> m_integral_slow_filter;
    std::vector<double> m_integral_term_command;
    std::vector<double> m_jac_transpose_f;
    rbd::Jacobian m_jac;

    std::vector<boost::circular_buffer<double>> m_filter_input_buffer;
    std::vector<boost::circular_buffer<double>> m_filter_output_buffer;
    std::vector<double> m_filter_command;

    std::vector<double> m_current_command;

public:
    KinovaRobot(const std::string& name, const std::string& ip_address, const std::string& username, const std::string& password);
    ~KinovaRobot();

    // ============================== Getter ============================== //
    std::vector<double> getJointPosition(void);
    std::string getName(void);

    // ============================== Setter ============================== //
    void setLowServoingMode(void);
    void setSingleServoingMode(void);
    void setCustomTorque(mc_rtc::Configuration & torque_config);
    void setControlMode(std::string mode);

    void init(mc_control::MCGlobalController & gc, mc_rtc::Configuration & kortexConfig); // Initialize connection to the robot
    void addLogEntry(mc_control::MCGlobalController & gc);
    void removeLogEntry(mc_control::MCGlobalController & gc);
    
    void updateState();
    void updateState(bool & running);
    void updateState(const k_api::BaseCyclic::Feedback data);
    bool sendCommand(mc_rbdyn::Robot & robot, bool & running);
    void updateSensors(mc_control::MCGlobalController & gc);
    void updateControl(mc_control::MCGlobalController & controller);

    double currentTorqueControlLaw(mc_rbdyn::Robot & robot, k_api::BaseCyclic::Feedback m_state_local, Eigen::MatrixXd jacobian, double joint_idx);
    void checkBaseFaultBanks(uint32_t fault_bank_a, uint32_t fault_bank_b);
    void checkActuatorsFaultBanks(k_api::BaseCyclic::Feedback feedback);
    std::vector<std::string> getBaseFaultList(uint32_t fault_bank);
    std::vector<std::string> getActuatorFaultList(uint32_t fault_bank);

    void controlThread(mc_control::MCGlobalController & controller, std::mutex & startM, std::condition_variable & startCV, bool & start, bool & running);
    void moveToHomePosition(void);
    void moveToInitPosition(void);

    std::string controlLoopParamToString(k_api::ActuatorConfig::LoopSelection & loop_selected, int actuator_idx);

    void printState(void);
    void printJointActiveControlLoop(int joint_id);

    // ============================== Private methods ============================== //
private:
    void initFiltersBuffers(void);

    void addGui(mc_control::MCGlobalController & gc);
    void removeGui(mc_control::MCGlobalController & gc);

    double jointPoseToRad(int joint_idx, double deg);
    double radToJointPose(int joint_idx, double rad);
    std::vector<double> computePostureTaskOffset(mc_rbdyn::Robot & robot, mc_tasks::PostureTaskPtr posture_task);
    uint32_t jointIdFromCommandID(google::protobuf::uint32 cmd_id);
    int64_t GetTickUs(void);
    void printError(const k_api::Error & err);
    void printException(k_api::KDetailedException & ex);
    std::function<void(k_api::Base::ActionNotification)> check_for_end_or_abort(bool& finished);
    std::function<void(k_api::Base::ActionNotification)> create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise_cart);
};

using KinovaRobotPtr = std::unique_ptr<KinovaRobot>;

std::string printVec(std::vector<double> vec);

} // namespace mc_kinova