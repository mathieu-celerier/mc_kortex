#include <mc_control/mc_global_controller.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rtc/logging.h>

#include <boost/circular_buffer.hpp>

#include <atomic>
#include <deque>
#include <future>
#include <memory>

#include <ActuatorConfigClientRpc.h>
#include <ActuatorCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <GripperCyclicMessage.pb.h>
#include <InterconnectConfigClientRpc.h>
#include <InterconnectCyclicClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>
#define GEAR_RATIO 100.0

namespace k_api = Kinova::Api;

namespace mc_kinova {

enum TorqueControlType { Default, Feedforward, Custom };

// How the cyclic commands reach the actuators: through the base, which
// relays them (LOW_LEVEL_SERVOING), or straight to each actuator and to the
// interconnect, the base out of the loop (BYPASS_SERVOING)
enum class LowLevelType { Base, Bypass };

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
  k_api::ActuatorConfig::ActuatorConfigClient *m_actuator_config;
  k_api::DeviceConfig::DeviceConfigClient *m_device_config;

  // ===== Low level bypass =====
  // One UDP connection per device, to its own address, in bypass. The
  // addresses are discovered in init(), the connections exist while
  // controlling, between startControl() and stopControl()
  LowLevelType m_low_level_type;
  struct BypassDevice {
    std::string ip_address;
    uint32_t device_id = 0;
    uint32_t order = 0;
    k_api::TransportClientUdp *transport = nullptr;
    k_api::RouterClient *router = nullptr;
    // Actuators only
    k_api::ActuatorConfig::ActuatorConfigClient *config = nullptr;
    k_api::ActuatorCyclic::ActuatorCyclicClient *cyclic = nullptr;
    // Interconnect only
    k_api::InterconnectCyclic::InterconnectCyclicClient *interconnect = nullptr;
  };
  std::vector<BypassDevice> m_bypass_actuators;
  BypassDevice m_bypass_interconnect;
  bool m_bypass_has_interconnect;

  std::string m_username;
  std::string m_password;
  std::string m_ip_address;
  int m_port;
  int m_port_real_time;

  std::string m_name;
  int m_actuator_count;

  int64_t m_dt;

  rbd::MultiBodyConfig m_command;
  k_api::BaseCyclic::Command m_base_command;
  // False until the controller produced a first command: until then only the
  // feedback is requested
  bool m_has_command;

  k_api::BaseCyclic::Feedback m_state;

  // ===== Cyclic exchange =====
  // A command leaves every tick, sendCommand(), and its feedback is awaited
  // until a deadline within the tick, receiveFeedback(). A reply that misses
  // the deadline stays queued and is used at a later tick, the controller
  // meanwhile runs on the last feedback it got: the send rate never depends
  // on the reply time
public:
  // The reply to one request, filled by the API's callback on its receive
  // thread and polled by the control loop. The API's _async variants are not
  // used: each call starts a thread (std::async), 8 per tick in bypass
  template <typename T> struct Reply {
    std::atomic<bool> done{false};
    // When the callback ran, on the same clock as GetTickUs()
    int64_t arrival_us = 0;
    bool ok = false;
    std::string error;
    T value;
  };
  template <typename T> using ReplyPtr = std::shared_ptr<Reply<T>>;
  struct PendingExchange {
    // Through the base: one exchange for the whole arm
    ReplyPtr<k_api::BaseCyclic::Feedback> feedback;
    // In bypass: one exchange per actuator, and one with the interconnect
    std::vector<ReplyPtr<k_api::ActuatorCyclic::Feedback>> actuators;
    ReplyPtr<k_api::InterconnectCyclic::Feedback> interconnect;
    int64_t send_us;
    int64_t tick;
  };

private:
  std::deque<PendingExchange> m_pending;
  int64_t m_tick;
  // Tick whose exchange produced m_state
  int64_t m_state_tick;
  // In bypass, the tick whose exchange produced each actuator's feedback
  std::vector<int64_t> m_actuator_state_tick;
  int64_t m_interconnect_state_tick;
  int64_t m_last_send_us;
  // Time between sending a command and its feedback arriving
  int64_t m_refresh_rtt_us;
  // In bypass, the same for each actuator's last reply
  std::vector<double> m_actuator_rtt_us;
  // How many ticks old the feedback the controller runs on is: 0 when the
  // feedback of this tick's exchange arrived before the deadline
  int64_t m_feedback_age;
  int64_t m_fresh_feedback;
  int64_t m_late_feedback;
  int64_t m_missed_refresh;
  int m_consecutive_missed_refresh;
  unsigned int m_refresh_timeout_ms;
  int m_max_missed_refresh;
  int m_feedback_wait_us;
  // Frame identifier of the last command sent, incremented every command so
  // the actuators can reject out of time frames
  uint32_t m_frame_id;
  // command_id reported by each actuator: not an echo of m_frame_id, a
  // counter of the robot that advances at about 1 kHz of its own clock
  std::vector<double> m_actuator_counter;

  k_api::Base::ServoingMode m_servoing_mode;
  k_api::ActuatorConfig::ControlMode m_control_mode;
  int m_control_mode_id;
  int m_prev_control_mode_id;
  // Control mode change in progress, one request per actuator, see
  // updateModeSwitch()
  static constexpr unsigned int kModeSwitchTimeoutMs = 1000;
  std::vector<std::future<void>> m_mode_switch;
  int m_mode_switch_id;
  k_api::ActuatorConfig::ControlMode m_mode_switch_mode;
  int64_t m_mode_switch_start_us;
  // Duration of the last control mode change
  double m_mode_switch_ms;

  std::vector<double> m_init_posture;

  bool m_use_filtered_velocities;
  double m_velocity_filter_ratio;
  std::vector<double> m_filtered_velocities;

  // ===== Gripper properties =====
  bool gripper_enabled;
  size_t gripper_idx;
  std::string m_gripper_name;
  // Index of the actuated joint in the gripper's q(), and its joint limits,
  // used to map between Kortex's 0% (open) - 100% (closed) and joint values
  size_t m_gripper_q_idx;
  double m_gripper_open_q;
  double m_gripper_closed_q;
  k_api::GripperCyclic::MotorCommand *m_gripper_motor_command;
  float gripper_position;
  float gripper_velocity;

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

public:
  KinovaRobot(const std::string &name, const std::string &ip_address,
              const std::string &username = "admin",
              const std::string &password = "admin");
  ~KinovaRobot();

  // ============================== Getter ============================== //
  std::vector<double> getJointPosition(void);
  std::string getName(void);

  // ============================== Setter ============================== //
  void setLowServoingMode(void);
  void setSingleServoingMode(void);
  void setCustomTorque(mc_rtc::Configuration &torque_config);
  void setControlMode(std::string mode);
  void setTorqueMode(std::string mode);

  void init(mc_control::MCGlobalController &gc,
            mc_rtc::Configuration
                &kortexConfig); // Initialize connection to the robot
  void createDatastoreEntries(mc_control::MCGlobalController &gc);
  void removeDatastoreEntries(mc_control::MCGlobalController &gc);
  void addLogEntry(mc_control::MCGlobalController &gc);
  void removeLogEntry(mc_control::MCGlobalController &gc);

  void updateState();
  // Control tick, in this order: sendCommand() then receiveFeedback() for
  // every arm (the arms exchange in parallel), updateSensors(), the controller
  // runs, updateControl() then buildCommand() for the next tick
  void sendCommand();
  // Waits for the feedback of this tick until deadline_us (GetTickUs() time)
  void receiveFeedback(int64_t deadline_us, bool &running);
  int64_t GetTickUs(void);
  // How long into a tick to wait for its feedback (µs)
  int feedbackWaitUs() const { return m_feedback_wait_us; }
  void updateSensors(mc_control::MCGlobalController &gc);
  void updateControl(mc_control::MCGlobalController &controller);
  bool buildCommand(mc_rbdyn::Robot &robot, bool &running);
  // Starts, or polls, the change of the actuators' control mode. False on
  // failure
  bool updateModeSwitch();

  void torqueFrictionComputation(mc_rbdyn::Robot &robot,
                                 const k_api::BaseCyclic::Feedback &state,
                                 size_t joint_idx);
  double currentTorqueControlLaw(mc_rbdyn::Robot &robot,
                                 const k_api::BaseCyclic::Feedback &state,
                                 size_t joint_idx);
  void checkBaseFaultBanks(uint32_t fault_bank_a, uint32_t fault_bank_b);
  void checkActuatorsFaultBanks(const k_api::BaseCyclic::Feedback &feedback);
  std::vector<std::string> getBaseFaultList(uint32_t fault_bank);
  std::vector<std::string> getActuatorFaultList(uint32_t fault_bank);

  // Enter low level servoing before the first tick, leave it after the last
  void startControl(mc_control::MCGlobalController &controller);
  void stopControl(mc_control::MCGlobalController &controller);
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

  // ===== Low level bypass =====
  // Reads the address of every actuator and of the interconnect
  void discoverBypassDevices();
  // Connects to every device and prepares each actuator for cyclic control,
  // as Kinova's low level bypass example does
  void startBypass();
  void disconnectBypassDevices();
  // Collects the replies of the pending exchanges, returns whether any
  // feedback was updated and, through fresh, whether all of it is this tick's
  bool collectBaseFeedback(bool &fresh);
  bool collectBypassFeedback(bool &fresh);
  // Sets the control mode of actuator i, through the base or directly
  std::future<void> setActuatorControlModeAsync(
      int i, const k_api::ActuatorConfig::ControlModeInformation &control_mode,
      const k_api::RouterClientSendOptions &options);

  void addGui(mc_control::MCGlobalController &gc);
  void removeGui(mc_control::MCGlobalController &gc);

  double jointPoseToRad(int joint_idx, double deg);
  double radToJointPose(int joint_idx, double rad);
  double gripperPercentToJoint(double percent) const;
  double jointToGripperPercent(double q) const;
  std::vector<double>
  computePostureTaskOffset(mc_rbdyn::Robot &robot,
                           mc_tasks::PostureTaskPtr posture_task);
  uint32_t jointIdFromCommandID(google::protobuf::uint32 cmd_id);
  void printError(const k_api::Error &err);
  void printException(k_api::KDetailedException &ex);
  std::function<void(k_api::Base::ActionNotification)>
  check_for_end_or_abort(bool &finished);
  std::function<void(k_api::Base::ActionNotification)>
  create_event_listener_by_promise(
      std::promise<k_api::Base::ActionEvent> &finish_promise_cart);
};

using KinovaRobotPtr = std::unique_ptr<KinovaRobot>;

std::string printVec(const std::vector<double> &vec);

} // namespace mc_kinova
