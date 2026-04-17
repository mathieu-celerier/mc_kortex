#include "ActuatorCyclic.pb.h"
#include <cstdint>
#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rtc/logging.h>

#include <boost/circular_buffer.hpp>

#include <ActuatorConfigClientRpc.h>
#include <ActuatorCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <GripperCyclicMessage.pb.h>
#include <InterconnectCyclicClientRpc.h>
#include <InterconnectCyclicMessage.pb.h>
#include <InterconnectConfigClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>
#include <atomic>
#include <condition_variable>
#include <vector>
#define GEAR_RATIO 100.0

namespace k_api = Kinova::Api;

namespace mc_kinova {

enum TorqueControlType { Default, Feedforward, Custom };
enum LowLevelInterfaceType { Base = 0, Bypass = 1 };

struct BypassActuatorClient {
  std::string ip_address;
  uint32_t device_id = 0;
  uint32_t order = 0;
  k_api::TransportClientUdp *transport = nullptr;
  k_api::RouterClient *router = nullptr;
  k_api::ActuatorConfig::ActuatorConfigClient *actuator_config = nullptr;
  k_api::ActuatorCyclic::ActuatorCyclicClient *actuator_cyclic = nullptr;
};

struct BypassInterconnectClient {
  std::string ip_address;
  uint32_t device_id = 0;
  uint32_t order = 0;
  k_api::TransportClientUdp *transport = nullptr;
  k_api::RouterClient *router = nullptr;
  k_api::InterconnectCyclic::InterconnectCyclicClient *interconnect_cyclic =
      nullptr;
};

class KinovaRobot {
private:
  k_api::RouterClient *m_router;
  k_api::TransportClientTcp *m_transport;
  k_api::RouterClient *m_router_real_time;
  k_api::TransportClientUdp *m_transport_real_time;
  k_api::SessionManager *m_session_manager;
  k_api::SessionManager *m_session_manager_real_time;
  k_api::Base::BaseClient *m_base;
  k_api::BaseCyclic::BaseCyclicClient *m_base_cyclic;
  k_api::DeviceManager::DeviceManagerClient *m_device_manager;
  k_api::DeviceConfig::DeviceConfigClient *m_device_config;
  k_api::ActuatorConfig::ActuatorConfigClient *m_actuator_config;
  std::vector<BypassActuatorClient> m_bypass_actuator_clients;
  BypassInterconnectClient m_bypass_interconnect_client;

  std::string m_username;
  std::string m_password;
  std::string m_ip_address;
  int m_port;
  int m_port_real_time;

  std::string m_name;
  int m_actuator_count;

  std::atomic<bool> stop_controller;

  int64_t m_dt;

  double t_plot;

  int m_control_id;
  int m_prev_control_id;
  std::mutex m_update_control_mutex;
  rbd::MultiBodyConfig m_command;
  k_api::BaseCyclic::Command m_base_command;

  std::mutex m_update_sensor_mutex;
  k_api::BaseCyclic::Feedback m_state;
  std::vector<k_api::ActuatorCyclic::Feedback> m_actuator_feedback_vec;

  k_api::Base::ServoingMode m_servoing_mode;
  k_api::ActuatorConfig::ControlMode m_control_mode;
  int m_control_mode_id;
  int m_prev_control_mode_id;
  uint32_t m_bypass_command_id;

  LowLevelInterfaceType m_low_level_interface_type;

  std::vector<double> m_init_posture;

  bool m_use_filtered_velocities;
  double m_velocity_filter_ratio;
  std::vector<double> m_filtered_velocities;

  // ===== Gripper properties =====
  bool gripper_enabled;
  size_t gripper_idx;
  k_api::GripperCyclic::MotorCommand *m_gripper_motor_command;
  k_api::GripperCyclic::MotorCommand *m_interconnect_gripper_motor_command;
  float gripper_position;
  float gripper_velocity;
  k_api::InterconnectCyclic::Command m_interconnect_command;
  k_api::InterconnectCyclic::Feedback m_interconnect_feedback;
  bool m_debug_enabled;
  std::vector<bool> m_debug_bypass_init_logged;
  std::vector<bool> m_debug_bypass_command_logged;
  std::vector<bool> m_debug_bypass_feedback_logged;
  std::vector<size_t> m_bypass_joint_for_actuator;
  std::vector<size_t> m_bypass_actuator_for_joint;
  bool m_debug_bypass_gripper_command_logged;
  bool m_debug_bypass_gripper_feedback_logged;

  // ===== Custom torque control properties =====
  TorqueControlType m_torque_control_type;

  std::vector<double> m_offsets;

  double m_mu;
  double m_friction_vel_threshold;
  double m_friction_accel_threshold;
  std::vector<double> m_stiction_values;
  std::vector<double> m_friction_values;
  std::vector<double> m_viscous_values;
  std::vector<double> m_friction_compensation_mode;
  std::vector<double> m_current_friction_compensation;

  std::vector<double> m_prev_torque_error;
  std::vector<double> m_torque_error;

  std::vector<double> m_integral_slow_filter;
  std::vector<double> m_integral_slow_filter_w_gain;
  double m_integral_slow_theta;
  double m_integral_slow_gain;
  std::vector<double> m_integral_slow_bound;

  std::vector<double> m_torque_measure_corrected;

  std::vector<double> m_jac_transpose_f;
  rbd::Jacobian m_jac;

  std::vector<boost::circular_buffer<double>> m_filter_input_buffer;
  std::vector<boost::circular_buffer<double>> m_filter_output_buffer;
  std::vector<double> m_filter_command;
  std::vector<double> m_filter_command_w_gain;
  std::vector<double> m_lambda;

  Eigen::VectorXd tau_fric;

  Eigen::VectorXd m_current_command;
  Eigen::VectorXd m_current_measurement;
  Eigen::VectorXd m_torque_from_current_measurement;
  Eigen::VectorXd m_tau_sensor;
  std::atomic<bool> m_control_ready;
  std::atomic<bool> m_command_ready;

public:
  KinovaRobot(const std::string &name, const std::string &ip_address,
              const std::string &username, const std::string &password,
              bool debug_enabled = false);
  ~KinovaRobot();

  // ============================== Getter ============================== //
  std::vector<double> getJointPosition(void);
  std::string getName(void);
  bool controlReady() const;

  // ============================== Setter ============================== //
  void setLowServoingMode(void);
  void setSingleServoingMode(void);
  void setCustomTorque(mc_rtc::Configuration &torque_config);
  void setControlMode(std::string mode);
  void setTorqueMode(std::string mode);
  void debugLog(const std::string &msg);
  void debugDiscoverDevices();

  void init(mc_control::MCGlobalController &gc,
            mc_rtc::Configuration
                &kortexConfig); // Initialize connection to the robot
  void addLogEntry(mc_control::MCGlobalController &gc);
  void removeLogEntry(mc_control::MCGlobalController &gc);

  void updateState();
  void updateStateBase(const k_api::BaseCyclic::Feedback data);
  void updateStateBypass(const k_api::ActuatorCyclic::Feedback data,
                         size_t joint_idx);
  bool sendCommandBase(mc_rbdyn::Robot &robot, bool &running);
  bool sendCommandBypass(mc_rbdyn::Robot &robot, bool &running);
  void updateSensors(mc_control::MCGlobalController &gc);
  void updateControl(mc_control::MCGlobalController &controller);

  void torqueFrictionComputation(mc_rbdyn::Robot &robot,
                                 const rbd::MultiBodyConfig &command,
                                 k_api::BaseCyclic::Feedback m_state_local,
                                 double joint_idx);
  double currentTorqueControlLaw(mc_rbdyn::Robot &robot,
                                 const rbd::MultiBodyConfig &command,
                                 double measured_torque, double velocity,
                                 double joint_idx);
  void checkBaseFaultBanks(uint32_t fault_bank_a, uint32_t fault_bank_b);
  void checkActuatorFaultBanks(uint32_t fault_bank_a, uint32_t fault_bank_b,
                               size_t joint_idx);
  void checkActuatorsFaultBanks(k_api::BaseCyclic::Feedback feedback);
  std::vector<std::string> getBaseFaultList(uint32_t fault_bank);
  std::vector<std::string> getActuatorFaultList(uint32_t fault_bank);
  std::vector<std::string> getFeedbackStatusFlag(uint32_t status_flags);

  void controlThread(mc_control::MCGlobalController &controller,
                     std::mutex &startM, std::condition_variable &startCV,
                     bool &start, bool &running);
  void stopController();
  void moveToHomePosition(void);
  void moveToInitPosition(void);

  std::string
  controlLoopParamToString(k_api::ActuatorConfig::LoopSelection &loop_selected,
                           int actuator_idx);

  void printState(void);
  void printJointActiveControlLoop(int joint_id);

  // ============================== Private methods
  // ============================== //
private:
  void initFiltersBuffers(void);
  bool validateServoingMode(k_api::Base::ServoingMode mode);
  void clearBypassDeviceClients();
  void createBypassDeviceClients();
  k_api::ActuatorConfig::ActuatorConfigClient *
  actuatorConfigClient(size_t joint_idx);
  k_api::ActuatorCyclic::ActuatorCyclicClient *
  actuatorCyclicClient(size_t joint_idx);
  k_api::InterconnectCyclic::InterconnectCyclicClient *
  interconnectCyclicClient();
  void setActuatorControlMode(
      size_t joint_idx,
      const k_api::ActuatorConfig::ControlModeInformation &control_mode);
  void setActuatorCommandMode(
      size_t joint_idx,
      const k_api::ActuatorConfig::CommandModeInformation &command_mode);
  void setActuatorServoing(
      size_t joint_idx, const k_api::ActuatorConfig::Servoing &servoing);
  void clearActuatorFaults(size_t joint_idx);
  bool hasValidCommand(const mc_rbdyn::Robot &robot) const;
  void updateBypassJointMapping(
      const std::vector<k_api::ActuatorCyclic::Feedback> &raw_feedbacks);
  void initializeBypass();
  void updateBypassGripperState(const k_api::InterconnectCyclic::Feedback &data);
  void updateGripperCommand(mc_rbdyn::Robot &robot);
  std::string ipv4ToString(uint32_t ipv4) const;

  void addGui(mc_control::MCGlobalController &gc);
  void removeGui(mc_control::MCGlobalController &gc);

  void addPlot(mc_control::MCGlobalController &gc);
  void removePlot(mc_control::MCGlobalController &gc);

  double jointPoseToRad(int joint_idx, double deg);
  double radToJointPose(int joint_idx, double rad);
  std::vector<double>
  computePostureTaskOffset(mc_rbdyn::Robot &robot,
                           mc_tasks::PostureTaskPtr posture_task);
  uint32_t jointIdFromCommandID(google::protobuf::uint32 cmd_id);
  int64_t GetTickUs(void);
  void printError(const k_api::Error &err);
  void printException(k_api::KDetailedException &ex);
  std::function<void(k_api::Base::ActionNotification)>
  check_for_end_or_abort(bool &finished);
  std::function<void(k_api::Base::ActionNotification)>
  create_event_listener_by_promise(
      std::promise<k_api::Base::ActionEvent> &finish_promise_cart);
};

using KinovaRobotPtr = std::unique_ptr<KinovaRobot>;

std::string printVec(std::vector<double> vec);

} // namespace mc_kinova
