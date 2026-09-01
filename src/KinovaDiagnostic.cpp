#include "KinovaDiagnostic.h"

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <ActuatorConfigClientRpc.h>
#include <ActuatorCyclicClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <set>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <csignal>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace k_api = Kinova::Api;

namespace mc_kinova {

namespace {

/** Sampling rate of the live feedback window */
constexpr double SAMPLING_FREQUENCY = 100.0;

/** Number of strain gauges making up one Kinova torque sensor */
constexpr int STRAIN_GAUGE_COUNT = 4;

/** Statistics accumulated over the sampling window for a single signal */
struct SignalStats {
  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::lowest();
  double sum = 0.0;
  double sum_sq = 0.0;
  size_t count = 0;
  std::set<float> distinct;

  void add(float value) {
    min = std::min(min, static_cast<double>(value));
    max = std::max(max, static_cast<double>(value));
    sum += value;
    sum_sq += static_cast<double>(value) * value;
    count += 1;
    distinct.insert(value);
  }

  double mean() const { return count ? sum / count : 0.0; }

  double stddev() const {
    if (count < 2)
      return 0.0;
    double m = mean();
    return std::sqrt(std::max(0.0, sum_sq / count - m * m));
  }
};

/** Everything we collected about one actuator */
struct ActuatorReport {
  uint32_t device_id = 0;
  std::string type = "UNKNOWN";
  std::string serial_number = "?";
  uint32_t firmware_version = 0;

  SignalStats torque;
  SignalStats current;
  SignalStats position;
  double voltage = 0.0;
  double temperature_motor = 0.0;
  double temperature_core = 0.0;

  uint32_t fault_bank_a = 0;
  uint32_t fault_bank_b = 0;
  uint32_t warning_bank_a = 0;
  uint32_t warning_bank_b = 0;

  bool has_torque_offset = false;
  double torque_offset = 0.0;

  bool has_calibration = false;
  double global_gain = 0.0;
  double global_offset = 0.0;
  std::vector<double> gauge_gain;
  std::vector<double> gauge_offset;

  bool has_strain_gauges = false;
  std::vector<SignalStats> gauge_raw;
  std::vector<SignalStats> gauge_value;

  /** Status of the calibration items the arm can run on its own, as
   * (item name, status) pairs. The torque sensor is deliberately absent from
   * that list in the API: it cannot be self-calibrated. */
  std::vector<std::pair<std::string, std::string>> calibration_status;

  std::vector<std::string> anomalies;
};

/** Decode a bank A fault/warning word using the enum shipped with the API
 *
 * Going through the protobuf descriptor rather than a hand-written table means
 * the decoding follows the Kortex version mc_kortex was built against.
 */
std::string decodeBankA(uint32_t bank) {
  if (bank == 0)
    return "none";
  std::string out;
  for (uint32_t bit = 1; bit != 0; bit <<= 1) {
    if ((bank & bit) == 0)
      continue;
    std::string name;
    if (k_api::ActuatorConfig::SafetyIdentifierBankA_IsValid(
            static_cast<int>(bit))) {
      name = k_api::ActuatorConfig::SafetyIdentifierBankA_Name(
          static_cast<k_api::ActuatorConfig::SafetyIdentifierBankA>(bit));
    } else {
      name = fmt::format("UNKNOWN_BIT_{:#x}", bit);
    }
    if (!out.empty())
      out += ", ";
    out += name;
  }
  return out;
}

std::string deviceTypeName(k_api::Common::DeviceTypes type) {
  switch (type) {
  case k_api::Common::DeviceTypes::BIG_ACTUATOR:
    return "BIG_ACTUATOR";
  case k_api::Common::DeviceTypes::SMALL_ACTUATOR:
    return "SMALL_ACTUATOR";
  case k_api::Common::DeviceTypes::XBIG_ACTUATOR:
    return "XBIG_ACTUATOR";
  default:
    return "OTHER";
  }
}

bool isActuator(k_api::Common::DeviceTypes type) {
  return type == k_api::Common::DeviceTypes::BIG_ACTUATOR ||
         type == k_api::Common::DeviceTypes::SMALL_ACTUATOR ||
         type == k_api::Common::DeviceTypes::XBIG_ACTUATOR;
}

/** The custom data channels are transported as raw 32 bit words */
float wordToFloat(uint32_t word) {
  float out = 0.0f;
  std::memcpy(&out, &word, sizeof(out));
  return out;
}

int32_t wordToInt32(uint32_t word) {
  int32_t out = 0;
  std::memcpy(&out, &word, sizeof(out));
  return out;
}

uint32_t customDataChannel(const k_api::ActuatorCyclic::CustomData &data,
                           int index) {
  switch (index) {
  case 0:
    return data.custom_data_0();
  case 1:
    return data.custom_data_1();
  case 2:
    return data.custom_data_2();
  case 3:
    return data.custom_data_3();
  case 4:
    return data.custom_data_4();
  case 5:
    return data.custom_data_5();
  case 6:
    return data.custom_data_6();
  case 7:
    return data.custom_data_7();
  default:
    return 0;
  }
}

uint32_t customDataChannel(const k_api::BaseCyclic::ActuatorCustomData &data,
                           int index) {
  switch (index) {
  case 0:
    return data.custom_data_0();
  case 1:
    return data.custom_data_1();
  case 2:
    return data.custom_data_2();
  case 3:
    return data.custom_data_3();
  case 4:
    return data.custom_data_4();
  case 5:
    return data.custom_data_5();
  case 6:
    return data.custom_data_6();
  case 7:
    return data.custom_data_7();
  default:
    return 0;
  }
}

/** Copy one actuator's four raw counts and four converted values out of a
 * custom data message, whichever service produced it, into the running
 * statistics and the dump row */
template <typename CustomDataT>
void recordGauges(const CustomDataT &data, ActuatorReport &report,
                  std::array<double, 2 * STRAIN_GAUGE_COUNT> &row) {
  for (int g = 0; g < STRAIN_GAUGE_COUNT; ++g) {
    double raw = static_cast<double>(wordToInt32(customDataChannel(data, g)));
    double value = wordToFloat(customDataChannel(data, g + STRAIN_GAUGE_COUNT));
    report.gauge_raw[static_cast<size_t>(g)].add(static_cast<float>(raw));
    report.gauge_value[static_cast<size_t>(g)].add(static_cast<float>(value));
    row[static_cast<size_t>(g)] = raw;
    row[static_cast<size_t>(g + STRAIN_GAUGE_COUNT)] = value;
  }
  report.has_strain_gauges = true;
}

/** Check that the arm answers on its TCP port before handing over to Kortex
 *
 * The Kortex transport blocks for minutes on an unreachable arm, which is a
 * poor experience for a tool meant to be run when things already go wrong.
 */
bool isReachable(const std::string &ip, uint16_t port, double timeout_s) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    return true; // cannot check, let Kortex try

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
    close(fd);
    return true; // not a plain IPv4 address, let Kortex resolve it
  }

  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  bool reachable = false;
  if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) ==
      0) {
    reachable = true;
  } else if (errno == EINPROGRESS) {
    fd_set write_set;
    FD_ZERO(&write_set);
    FD_SET(fd, &write_set);
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_s);
    tv.tv_usec = static_cast<suseconds_t>((timeout_s - tv.tv_sec) * 1e6);
    if (select(fd + 1, nullptr, &write_set, nullptr, &tv) > 0) {
      int error = 0;
      socklen_t len = sizeof(error);
      getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &len);
      reachable = (error == 0);
    }
  }
  close(fd);
  return reachable;
}

/** Connects to the arm and owns every client used by the diagnostic */
class DiagnosticSession {
public:
  DiagnosticSession(const DiagnosticOptions &opts) {
    auto error_callback = [](k_api::KError err) {
      mc_rtc::log::error("[mc_kortex] Router error: {}", err.toString());
    };

    transport_ = new k_api::TransportClientTcp();
    router_ = new k_api::RouterClient(transport_, error_callback);
    transport_->connect(opts.ip_address, 10000);

    transport_rt_ = new k_api::TransportClientUdp();
    router_rt_ = new k_api::RouterClient(transport_rt_, error_callback);
    transport_rt_->connect(opts.ip_address, 10001);

    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username(opts.username);
    session_info.set_password(opts.password);
    session_info.set_session_inactivity_timeout(60000);
    session_info.set_connection_inactivity_timeout(2000);

    session_manager_ = new k_api::SessionManager(router_);
    session_manager_->CreateSession(session_info);
    session_manager_rt_ = new k_api::SessionManager(router_rt_);
    session_manager_rt_->CreateSession(session_info);

    device_manager_ = new k_api::DeviceManager::DeviceManagerClient(router_);
    device_config_ = new k_api::DeviceConfig::DeviceConfigClient(router_);
    actuator_config_ = new k_api::ActuatorConfig::ActuatorConfigClient(router_);
    base_ = new k_api::Base::BaseClient(router_);
    base_cyclic_ = new k_api::BaseCyclic::BaseCyclicClient(router_rt_);
    actuator_cyclic_ =
        new k_api::ActuatorCyclic::ActuatorCyclicClient(router_rt_);
    // Depending on the firmware the actuator cyclic service answers on one
    // router or the other, so keep both and pick whichever replies
    actuator_cyclic_tcp_ =
        new k_api::ActuatorCyclic::ActuatorCyclicClient(router_);
    // Same story for the whole arm custom data read
    base_cyclic_tcp_ = new k_api::BaseCyclic::BaseCyclicClient(router_);
  }

  ~DiagnosticSession() {
    if (session_manager_)
      session_manager_->CloseSession();
    if (session_manager_rt_)
      session_manager_rt_->CloseSession();
    if (router_)
      router_->SetActivationStatus(false);
    if (transport_)
      transport_->disconnect();
    if (router_rt_)
      router_rt_->SetActivationStatus(false);
    if (transport_rt_)
      transport_rt_->disconnect();

    delete actuator_cyclic_tcp_;
    delete actuator_cyclic_;
    delete base_cyclic_tcp_;
    delete base_cyclic_;
    delete base_;
    delete actuator_config_;
    delete device_config_;
    delete device_manager_;
    delete session_manager_rt_;
    delete session_manager_;
    delete router_rt_;
    delete router_;
    delete transport_rt_;
    delete transport_;
  }

  k_api::DeviceManager::DeviceManagerClient *device_manager_ = nullptr;
  k_api::DeviceConfig::DeviceConfigClient *device_config_ = nullptr;
  k_api::ActuatorConfig::ActuatorConfigClient *actuator_config_ = nullptr;
  k_api::Base::BaseClient *base_ = nullptr;
  k_api::BaseCyclic::BaseCyclicClient *base_cyclic_ = nullptr;
  k_api::BaseCyclic::BaseCyclicClient *base_cyclic_tcp_ = nullptr;
  k_api::ActuatorCyclic::ActuatorCyclicClient *actuator_cyclic_ = nullptr;
  k_api::ActuatorCyclic::ActuatorCyclicClient *actuator_cyclic_tcp_ = nullptr;

private:
  k_api::TransportClientTcp *transport_ = nullptr;
  k_api::RouterClient *router_ = nullptr;
  k_api::TransportClientUdp *transport_rt_ = nullptr;
  k_api::RouterClient *router_rt_ = nullptr;
  k_api::SessionManager *session_manager_ = nullptr;
  k_api::SessionManager *session_manager_rt_ = nullptr;
};

/** Identity of the arm as a whole, as opposed to that of its actuators
 *
 * This is what a support ticket needs to name the machine: the actuator serial
 * identifies a part, the base serial identifies the robot it belongs to.
 */
struct ArmIdentity {
  std::string serial_number = "?";
  std::string model_number = "?";
  std::string part_number = "?";
  std::string part_number_revision = "?";
  std::string mac_address = "?";
  uint32_t firmware_version = 0;
  uint32_t bootloader_version = 0;
};

std::string formatMacAddress(const std::string &raw) {
  if (raw.empty())
    return "?";
  std::string out;
  for (size_t i = 0; i < raw.size(); ++i) {
    if (i != 0)
      out += ':';
    out += fmt::format("{:02X}", static_cast<uint8_t>(raw[i]));
  }
  return out;
}

/** Read the base device identity
 *
 * Device id 0 addresses the base, so every field is a plain read. Each one is
 * guarded on its own: a firmware that does not serve one of them should not
 * cost us the others.
 */
ArmIdentity readArmIdentity(DiagnosticSession &session) {
  ArmIdentity id;
  auto *cfg = session.device_config_;
  auto attempt = [](const char *what, auto &&fn) {
    try {
      fn();
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::warning("[mc_kortex] Could not read the arm {}: {}", what,
                           ex.what());
    }
  };
  attempt("serial number",
          [&] { id.serial_number = cfg->GetSerialNumber(0).serial_number(); });
  attempt("model number",
          [&] { id.model_number = cfg->GetModelNumber(0).model_number(); });
  attempt("part number",
          [&] { id.part_number = cfg->GetPartNumber(0).part_number(); });
  attempt("part number revision", [&] {
    id.part_number_revision =
        cfg->GetPartNumberRevision(0).part_number_revision();
  });
  attempt("MAC address", [&] {
    id.mac_address = formatMacAddress(cfg->GetMACAddress(0).mac_address());
  });
  attempt("firmware version", [&] {
    id.firmware_version = cfg->GetFirmwareVersion(0).firmware_version();
  });
  attempt("bootloader version", [&] {
    id.bootloader_version = cfg->GetBootloaderVersion(0).bootloader_version();
  });
  return id;
}

/** Enumerate the actuator devices, ordered as the joints are */
std::vector<ActuatorReport> discoverActuators(DiagnosticSession &session) {
  std::vector<std::pair<uint32_t, ActuatorReport>> ordered;
  auto devices = session.device_manager_->ReadAllDevices();
  for (const auto &handle : devices.device_handle()) {
    if (!isActuator(handle.device_type()))
      continue;
    ActuatorReport report;
    report.device_id = handle.device_identifier();
    report.type = deviceTypeName(handle.device_type());
    try {
      report.serial_number =
          session.device_config_->GetSerialNumber(report.device_id)
              .serial_number();
      report.firmware_version =
          session.device_config_->GetFirmwareVersion(report.device_id)
              .firmware_version();
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::warning(
          "[mc_kortex] Could not read identity of device {}: {}",
          report.device_id, ex.what());
    }
    ordered.emplace_back(handle.order(), report);
  }
  std::sort(
      ordered.begin(), ordered.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });
  std::vector<ActuatorReport> reports;
  reports.reserve(ordered.size());
  for (auto &entry : ordered)
    reports.push_back(entry.second);
  return reports;
}

/** Sample the cyclic feedback of every actuator over the requested window */
void sampleFeedback(DiagnosticSession &session,
                    std::vector<ActuatorReport> &reports, double duration) {
  size_t samples =
      std::max<size_t>(1, static_cast<size_t>(duration * SAMPLING_FREQUENCY));
  auto period =
      std::chrono::microseconds(static_cast<int64_t>(1e6 / SAMPLING_FREQUENCY));
  mc_rtc::log::info(
      "[mc_kortex] Sampling actuator feedback for {:.1f}s ({} samples), "
      "move the joints by hand to exercise the torque sensors",
      duration, samples);

  for (size_t s = 0; s < samples; ++s) {
    auto feedback = session.base_cyclic_->RefreshFeedback();
    int count = std::min<int>(feedback.actuators_size(),
                              static_cast<int>(reports.size()));
    for (int i = 0; i < count; ++i) {
      const auto &actuator = feedback.actuators(i);
      auto &report = reports[static_cast<size_t>(i)];
      report.torque.add(actuator.torque());
      report.current.add(actuator.current_motor());
      report.position.add(actuator.position());
      report.voltage = actuator.voltage();
      report.temperature_motor = actuator.temperature_motor();
      report.temperature_core = actuator.temperature_core();
      report.fault_bank_a |= actuator.fault_bank_a();
      report.fault_bank_b |= actuator.fault_bank_b();
      report.warning_bank_a |= actuator.warning_bank_a();
      report.warning_bank_b |= actuator.warning_bank_b();
    }
    std::this_thread::sleep_for(period);
  }
}

/** Read the torque offset and torque calibration of every actuator */
void readTorqueConfiguration(DiagnosticSession &session,
                             std::vector<ActuatorReport> &reports) {
  for (auto &report : reports) {
    try {
      report.torque_offset =
          session.actuator_config_->GetTorqueOffset(report.device_id)
              .torque_offset();
      report.has_torque_offset = true;
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::warning(
          "[mc_kortex] Could not read torque offset of device {}: {}",
          report.device_id, ex.what());
    }
    try {
      auto calibration =
          session.actuator_config_->ReadTorqueCalibration(report.device_id);
      report.global_gain = calibration.global_gain();
      report.global_offset = calibration.global_offset();
      for (auto gain : calibration.gain())
        report.gauge_gain.push_back(gain);
      for (auto offset : calibration.offset())
        report.gauge_offset.push_back(offset);
      report.has_calibration = true;
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::warning(
          "[mc_kortex] Could not read torque calibration of device {}: {}",
          report.device_id, ex.what());
    }
  }
}

/** Read the status of the self-calibrated items of every actuator
 *
 * Torque is not among them, so this tells apart a wipe that only hit the
 * torque calibration from a wider loss of the actuator non-volatile memory.
 */
void readCalibrationStatus(DiagnosticSession &session,
                           std::vector<ActuatorReport> &reports) {
  using k_api::DeviceConfig::CalibrationItem;
  static const std::vector<CalibrationItem> items = {
      CalibrationItem::COGGING, CalibrationItem::MAGNETIC,
      CalibrationItem::MOTOR, CalibrationItem::POSITION_RANGE};

  for (auto &report : reports) {
    for (auto item : items) {
      k_api::DeviceConfig::CalibrationElement element;
      element.set_calibration_item(item);
      std::string name = k_api::DeviceConfig::CalibrationItem_Name(item);
      try {
        auto result = session.device_config_->GetCalibrationResult(
            element, report.device_id);
        report.calibration_status.emplace_back(
            name, k_api::DeviceConfig::CalibrationStatus_Name(
                      result.calibration_status()));
      } catch (const k_api::KDetailedException &ex) {
        // Not every firmware answers this RPC on actuators. Report it once
        // rather than failing loudly seven times over.
        if (report.device_id == reports.front().device_id &&
            item == items.front()) {
          mc_rtc::log::warning(
              "[mc_kortex] Calibration status is not available on this arm: {}",
              ex.what());
        }
        report.calibration_status.emplace_back(name, "UNAVAILABLE");
      }
    }
    bool none_available = std::all_of(
        report.calibration_status.begin(), report.calibration_status.end(),
        [](const auto &entry) { return entry.second == "UNAVAILABLE"; });
    if (report.device_id == reports.front().device_id && none_available) {
      // The RPC is unsupported arm-wide, no point querying the other actuators
      for (auto &other : reports) {
        if (other.device_id == report.device_id)
          continue;
        for (auto item : items) {
          other.calibration_status.emplace_back(
              k_api::DeviceConfig::CalibrationItem_Name(item), "UNAVAILABLE");
        }
      }
      return;
    }
  }
}

/** Sample the four individual strain gauges of every torque sensor
 *
 * Every actuator is sampled within a single window rather than one after the
 * other: the caller pays the cost of low level servoing only once. The custom
 * data selection is restored afterwards so the arm is left as we found it.
 */
/** Sample the individual strain gauges of every actuator
 *
 * Returns false when the actuator cyclic service is not served on any router,
 * which is the only case where switching to low level servoing might help.
 */
bool sampleStrainGauges(DiagnosticSession &session,
                        std::vector<ActuatorReport> &reports, double duration,
                        const std::string &dump_path) {
  if (reports.empty())
    return false;

  // Find a router the actuator cyclic service actually answers on, before
  // touching any selection, so a firmware that does not serve it at all leaves
  // the arm exactly as we found it
  k_api::ActuatorCyclic::MessageId message_id;
  message_id.set_identifier(0);
  k_api::ActuatorCyclic::ActuatorCyclicClient *cyclic = nullptr;
  for (auto *candidate :
       {session.actuator_cyclic_, session.actuator_cyclic_tcp_}) {
    if (candidate == nullptr)
      continue;
    try {
      candidate->RefreshCustomData(message_id, reports.front().device_id);
      cyclic = candidate;
      break;
    } catch (const k_api::KDetailedException &) {
    }
  }
  if (cyclic == nullptr) {
    mc_rtc::log::warning(
        "[mc_kortex] The actuator cyclic service is not answering on either "
        "router, the strain gauges cannot be read");
    return false;
  }

  using k_api::ActuatorConfig::CustomDataIndex;
  static const std::vector<CustomDataIndex> channels = {
      CustomDataIndex::INT32_TORQUE_SENSOR_RAW_0,
      CustomDataIndex::INT32_TORQUE_SENSOR_RAW_1,
      CustomDataIndex::INT32_TORQUE_SENSOR_RAW_2,
      CustomDataIndex::INT32_TORQUE_SENSOR_RAW_3,
      CustomDataIndex::FLOAT_TORQUE_SENSOR_0,
      CustomDataIndex::FLOAT_TORQUE_SENSOR_1,
      CustomDataIndex::FLOAT_TORQUE_SENSOR_2,
      CustomDataIndex::FLOAT_TORQUE_SENSOR_3};

  // A transient router timeout must cost one sample, not the whole capture,
  // so a joint is only given up on after several consecutive failures
  constexpr int MAX_CONSECUTIVE_FAILURES = 5;
  std::vector<int> failures(reports.size(), 0);
  std::vector<k_api::ActuatorConfig::CustomDataSelection> previous(
      reports.size());
  std::vector<bool> restore(reports.size(), false);
  std::vector<bool> selected(reports.size(), false);

  for (size_t i = 0; i < reports.size(); ++i) {
    auto device_id = reports[i].device_id;
    try {
      previous[i] = session.actuator_config_->GetSelectedCustomData(device_id);
      restore[i] = true;
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::warning(
          "[mc_kortex] Could not read custom data selection of device {}: {}",
          device_id, ex.what());
    }
    try {
      k_api::ActuatorConfig::CustomDataSelection selection;
      for (auto channel : channels)
        selection.add_channel(channel);
      session.actuator_config_->SelectCustomData(selection, device_id);
      selected[i] = true;
      reports[i].gauge_raw.assign(STRAIN_GAUGE_COUNT, SignalStats());
      reports[i].gauge_value.assign(STRAIN_GAUGE_COUNT, SignalStats());
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::warning(
          "[mc_kortex] Could not select strain gauge data on device {}: {}",
          device_id, ex.what());
    }
  }

  size_t samples =
      std::max<size_t>(1, static_cast<size_t>(duration * SAMPLING_FREQUENCY));
  auto period =
      std::chrono::microseconds(static_cast<int64_t>(1e6 / SAMPLING_FREQUENCY));

  // One BaseCyclic::RefreshCustomData returns the custom data of every
  // actuator, so all four gauges of all seven joints come from a single
  // instant, the way the control loop takes its feedback. The per actuator
  // ActuatorCyclic path below costs one round trip each and staggered the
  // joints by up to ~175ms: invisible on a stationary arm, and larger than the
  // signal as soon as the load moves.
  k_api::BaseCyclic::BaseCyclicClient *arm_cyclic = nullptr;
  k_api::BaseCyclic::CustomData custom_request;
  std::string arm_cyclic_reason;
  for (auto *candidate : {session.base_cyclic_, session.base_cyclic_tcp_}) {
    if (candidate == nullptr)
      continue;
    try {
      auto probe = candidate->RefreshCustomData(custom_request);
      if (probe.actuators_custom_data_size() >=
          static_cast<int>(reports.size())) {
        arm_cyclic = candidate;
        break;
      }
      arm_cyclic_reason =
          fmt::format("answered with {} actuators, {} expected",
                      probe.actuators_custom_data_size(), reports.size());
    } catch (const k_api::KDetailedException &ex) {
      arm_cyclic_reason = ex.what();
    }
  }
  if (arm_cyclic == nullptr) {
    // Not an error: BaseCyclic::RefreshCustomData answers only in low level
    // servoing, which a read-only diagnostic must not enter. The per actuator
    // path below then reads each actuator's own feedback right beside its own
    // gauges, so every joint is internally consistent even though the joints
    // in one row are staggered relative to each other. Each joint is fitted on
    // its own, so that stagger does not matter; what mattered, and what this
    // avoids, is a torque read up to seven round trips away from its gauges.
    mc_rtc::log::info(
        "[mc_kortex] Whole arm custom data unavailable ({}), reading each "
        "actuator's gauges and feedback together instead",
        arm_cyclic_reason);
  }

  // Per sample dump: the timestamp of each of the two reads, every gauge of
  // every actuator, and the torque and position each actuator reports
  std::ofstream dump;
  if (!dump_path.empty()) {
    dump.open(dump_path);
    // epoch is the wall clock at the start of the row: the only column that
    // can be lined up with a stream recorded outside this process, such as the
    // force torque sensor
    dump << "t;epoch;t_custom;t_feedback";
    for (size_t i = 0; i < reports.size(); ++i)
      dump << fmt::format(";j{}_t", i + 1);
    for (size_t i = 0; i < reports.size(); ++i) {
      for (int g = 0; g < STRAIN_GAUGE_COUNT; ++g)
        dump << fmt::format(";j{}_gauge{}_raw", i + 1, g);
      for (int g = 0; g < STRAIN_GAUGE_COUNT; ++g)
        dump << fmt::format(";j{}_gauge{}_Nm", i + 1, g);
    }
    for (size_t i = 0; i < reports.size(); ++i)
      dump << fmt::format(";j{}_torque", i + 1);
    for (size_t i = 0; i < reports.size(); ++i)
      dump << fmt::format(";j{}_position", i + 1);
    dump << "\n";
  }
  auto start = std::chrono::steady_clock::now();
  auto elapsed = [&start]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         start)
        .count();
  };

  for (size_t s = 0; s < samples; ++s) {
    std::vector<std::array<double, 2 * STRAIN_GAUGE_COUNT>> row(reports.size());
    for (auto &entry : row)
      entry.fill(std::nan(""));
    double t_custom = std::nan("");
    std::vector<double> torque(reports.size(), std::nan(""));
    std::vector<double> position(reports.size(), std::nan(""));
    std::vector<double> t_joint(reports.size(), std::nan(""));

    if (arm_cyclic != nullptr) {
      try {
        auto data = arm_cyclic->RefreshCustomData(custom_request);
        t_custom = elapsed();
        int count = std::min<int>(data.actuators_custom_data_size(),
                                  static_cast<int>(reports.size()));
        for (int i = 0; i < count; ++i) {
          if (!selected[static_cast<size_t>(i)])
            continue;
          recordGauges(data.actuators_custom_data(i),
                       reports[static_cast<size_t>(i)],
                       row[static_cast<size_t>(i)]);
          t_joint[static_cast<size_t>(i)] = t_custom;
        }
      } catch (const std::exception &ex) {
        if (s == 0) {
          mc_rtc::log::warning(
              "[mc_kortex] Could not read the whole arm custom data: {}",
              ex.what());
        }
      }
    } else {
      for (size_t i = 0; i < reports.size(); ++i) {
        if (!selected[i])
          continue;
        auto &report = reports[i];
        try {
          auto data = cyclic->RefreshCustomData(message_id, report.device_id);
          t_joint[i] = elapsed();
          recordGauges(data, report, row[i]);
          // Immediately next to the gauges of this same actuator, so the pair
          // is consistent no matter how fast the load is changing
          auto fb = cyclic->RefreshFeedback(message_id, report.device_id);
          torque[i] = fb.torque();
          position[i] = fb.position();
          failures[i] = 0;
        } catch (const std::exception &ex) {
          if (failures[i] == 0) {
            mc_rtc::log::warning(
                "[mc_kortex] Could not read strain gauge data on device {}: {}",
                report.device_id, ex.what());
          }
          if (++failures[i] >= MAX_CONSECUTIVE_FAILURES) {
            mc_rtc::log::warning(
                "[mc_kortex] Giving up on device {} after {} consecutive "
                "failures",
                report.device_id, failures[i]);
            selected[i] = false;
          }
        }
      }
      t_custom = elapsed();
    }

    if (dump.is_open()) {
      double t_feedback = std::nan("");
      if (arm_cyclic != nullptr) {
        try {
          auto feedback = session.base_cyclic_->RefreshFeedback();
          t_feedback = elapsed();
          int count = std::min<int>(feedback.actuators_size(),
                                    static_cast<int>(reports.size()));
          for (int i = 0; i < count; ++i) {
            torque[static_cast<size_t>(i)] = feedback.actuators(i).torque();
            position[static_cast<size_t>(i)] = feedback.actuators(i).position();
          }
        } catch (const std::exception &) {
        }
      }
      double epoch = std::chrono::duration<double>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
      dump << fmt::format("{:.4f};{:.6f};{:.4f};{:.4f}", elapsed(), epoch,
                          t_custom, t_feedback);
      for (double v : t_joint)
        dump << ";" << fmt::format("{:.4f}", v);
      for (const auto &entry : row) {
        for (double v : entry)
          dump << ";" << fmt::format("{:.6f}", v);
      }
      for (double v : torque)
        dump << ";" << fmt::format("{:.6f}", v);
      for (double v : position)
        dump << ";" << fmt::format("{:.6f}", v);
      dump << "\n";
    }

    std::this_thread::sleep_for(period);
  }

  if (dump.is_open()) {
    dump.close();
    mc_rtc::log::success("[mc_kortex] Strain gauge samples written to {}",
                         dump_path);
  }

  for (size_t i = 0; i < reports.size(); ++i) {
    if (!restore[i])
      continue;
    try {
      session.actuator_config_->SelectCustomData(previous[i],
                                                 reports[i].device_id);
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::warning("[mc_kortex] Could not restore custom data "
                           "selection of device {}: {}",
                           reports[i].device_id, ex.what());
    }
  }

  return std::any_of(
      reports.begin(), reports.end(),
      [](const ActuatorReport &r) { return r.has_strain_gauges; });
}

/** Base client of the guard currently holding low level servoing, if any
 *
 * Only touched by the guard and by the signal handler, which is why it is a
 * plain atomic pointer: the handler must not allocate or lock.
 */
std::atomic<k_api::Base::BaseClient *> g_low_level_base{nullptr};
std::atomic<int> g_low_level_previous{
    static_cast<int>(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)};

void restoreServoingOnSignal(int signum) {
  auto *base = g_low_level_base.exchange(nullptr);
  if (base != nullptr) {
    try {
      auto mode = k_api::Base::ServoingModeInformation();
      mode.set_servoing_mode(
          static_cast<k_api::Base::ServoingMode>(g_low_level_previous.load()));
      base->SetServoingMode(mode);
    } catch (...) {
      // Nothing useful to do from a signal handler, the message below is the
      // best we can offer
    }
  }
  std::signal(signum, SIG_DFL);
  std::raise(signum);
}

/** Puts the arm in low level servoing for the lifetime of the object
 *
 * No command is ever sent while in that mode: the diagnostic only reads. The
 * previous servoing mode is restored by the destructor so it also happens on
 * the way out of an exception.
 */
class LowLevelServoingGuard {
public:
  LowLevelServoingGuard(k_api::Base::BaseClient *base,
                        k_api::Base::ServoingMode previous)
      : base_(base), previous_(previous) {
    auto mode = k_api::Base::ServoingModeInformation();
    mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base_->SetServoingMode(mode);
    active_ = true;
    // An interrupted diagnostic must not leave the arm in low level servoing
    g_low_level_previous.store(static_cast<int>(previous));
    g_low_level_base.store(base_);
    previous_sigint_ = std::signal(SIGINT, restoreServoingOnSignal);
    previous_sigterm_ = std::signal(SIGTERM, restoreServoingOnSignal);
  }

  ~LowLevelServoingGuard() {
    if (!active_)
      return;
    g_low_level_base.store(nullptr);
    std::signal(SIGINT, previous_sigint_);
    std::signal(SIGTERM, previous_sigterm_);
    try {
      auto mode = k_api::Base::ServoingModeInformation();
      mode.set_servoing_mode(previous_);
      base_->SetServoingMode(mode);
      mc_rtc::log::info("[mc_kortex] Servoing mode restored");
    } catch (const k_api::KDetailedException &ex) {
      mc_rtc::log::error("[mc_kortex] Could not restore the servoing mode, do "
                         "it from the web app before using the arm: {}",
                         ex.what());
    }
  }

private:
  k_api::Base::BaseClient *base_;
  k_api::Base::ServoingMode previous_;
  bool active_ = false;
  void (*previous_sigint_)(int) = SIG_DFL;
  void (*previous_sigterm_)(int) = SIG_DFL;
};

/** Re-read the fault banks and report anything the low level window raised */
void reportNewFaults(DiagnosticSession &session,
                     const std::vector<ActuatorReport> &before) {
  try {
    auto feedback = session.base_cyclic_->RefreshFeedback();
    int count = std::min<int>(feedback.actuators_size(),
                              static_cast<int>(before.size()));
    for (int i = 0; i < count; ++i) {
      uint32_t bank_a = feedback.actuators(i).fault_bank_a();
      uint32_t raised = bank_a & ~before[static_cast<size_t>(i)].fault_bank_a;
      if (raised != 0) {
        mc_rtc::log::warning("[mc_kortex] joint_{} raised a fault during the "
                             "low level window: {}, clear it before using the "
                             "arm",
                             i + 1, decodeBankA(raised));
      }
    }
  } catch (const k_api::KDetailedException &ex) {
    mc_rtc::log::warning("[mc_kortex] Could not re-read the fault banks: {}",
                         ex.what());
  }
}

/** Turn the collected numbers into a list of things worth looking at */
void detectAnomalies(std::vector<ActuatorReport> &reports) {
  for (size_t i = 0; i < reports.size(); ++i) {
    auto &report = reports[i];

    if (report.torque.distinct.size() == 1) {
      report.anomalies.push_back(fmt::format(
          "torque feedback is a single constant value ({:.6f} Nm) over the "
          "whole window: the torque sensor is not reporting",
          report.torque.min));
    } else if (report.torque.stddev() < 1e-6) {
      report.anomalies.push_back(
          "torque feedback does not vary: the torque sensor may be stuck");
    }

    if (report.fault_bank_a != 0) {
      report.anomalies.push_back(fmt::format("fault bank A is set: {}",
                                             decodeBankA(report.fault_bank_a)));
    }
    if (report.fault_bank_b != 0) {
      report.anomalies.push_back(
          fmt::format("fault bank B is set: {:#010x}", report.fault_bank_b));
    }

    if (report.has_calibration) {
      if (std::abs(report.global_gain) < 1e-9) {
        report.anomalies.push_back(
            "torque calibration global gain is zero: the sensor is calibrated "
            "to always read zero");
      }
      for (size_t g = 0; g < report.gauge_gain.size(); ++g) {
        if (std::abs(report.gauge_gain[g]) < 1e-9) {
          report.anomalies.push_back(
              fmt::format("strain gauge {} has a zero calibration gain", g));
        }
      }
    }

    for (const auto &entry : report.calibration_status) {
      if (entry.second == "CALIBRATED" || entry.second == "UNAVAILABLE")
        continue;
      report.anomalies.push_back(
          fmt::format("{} calibration is {}", entry.first, entry.second));
    }

    if (report.has_strain_gauges) {
      for (size_t g = 0; g < report.gauge_raw.size(); ++g) {
        if (report.gauge_raw[g].distinct.size() == 1) {
          report.anomalies.push_back(fmt::format(
              "strain gauge {} raw ADC is stuck at {:.0f}: gauge or its "
              "wiring is dead",
              g, report.gauge_raw[g].min));
        }
      }
      // Live gauges behind a dead converted value settle the question: the
      // sensor works and only its calibration is missing
      bool gauges_alive = std::all_of(
          report.gauge_raw.begin(), report.gauge_raw.end(),
          [](const SignalStats &s) { return s.distinct.size() > 1; });
      bool values_dead =
          std::all_of(report.gauge_value.begin(), report.gauge_value.end(),
                      [](const SignalStats &s) {
                        return s.distinct.size() == 1 && std::abs(s.min) < 1e-9;
                      });
      if (gauges_alive && values_dead) {
        report.anomalies.push_back(
            "the four strain gauges are alive but every converted value is "
            "zero: the sensor hardware is healthy, only its calibration is "
            "missing");
      }
    }
  }
}

void printReport(const ArmIdentity &arm,
                 const std::vector<ActuatorReport> &reports) {
  mc_rtc::log::info("===== Arm identity =====");
  mc_rtc::log::info("  serial number       {}", arm.serial_number);
  mc_rtc::log::info("  model number        {}", arm.model_number);
  mc_rtc::log::info("  part number         {}{}", arm.part_number,
                    arm.part_number_revision == "?"
                        ? ""
                        : fmt::format(" rev {}", arm.part_number_revision));
  mc_rtc::log::info("  firmware / boot     0x{:08x} / 0x{:08x}",
                    arm.firmware_version, arm.bootloader_version);
  mc_rtc::log::info("  MAC address         {}", arm.mac_address);
  mc_rtc::log::info("===== Actuator identity =====");
  for (size_t i = 0; i < reports.size(); ++i) {
    const auto &r = reports[i];
    mc_rtc::log::info(
        "joint_{} (device {}): {:14} serial {:12} firmware {:#010x}", i + 1,
        r.device_id, r.type, r.serial_number, r.firmware_version);
  }

  mc_rtc::log::info("===== Live feedback =====");
  mc_rtc::log::info("{:>7} {:>10} {:>10} {:>10} {:>10} {:>8} {:>9} {:>7}",
                    "joint", "tau_min", "tau_max", "tau_std", "tau_mean",
                    "distinct", "I_mean", "T_motor");
  for (size_t i = 0; i < reports.size(); ++i) {
    const auto &r = reports[i];
    mc_rtc::log::info(
        "{:>7} {:>10.4f} {:>10.4f} {:>10.6f} {:>10.4f} {:>8} {:>9.3f} {:>7.1f}",
        i + 1, r.torque.min, r.torque.max, r.torque.stddev(), r.torque.mean(),
        r.torque.distinct.size(), r.current.mean(), r.temperature_motor);
  }

  mc_rtc::log::info("===== Fault and warning banks =====");
  for (size_t i = 0; i < reports.size(); ++i) {
    const auto &r = reports[i];
    mc_rtc::log::info("joint_{}: fault A {:#010x} [{}] | fault B {:#010x} | "
                      "warning A {:#010x} [{}] | warning B {:#010x}",
                      i + 1, r.fault_bank_a, decodeBankA(r.fault_bank_a),
                      r.fault_bank_b, r.warning_bank_a,
                      decodeBankA(r.warning_bank_a), r.warning_bank_b);
  }

  mc_rtc::log::info("===== Torque calibration =====");
  for (size_t i = 0; i < reports.size(); ++i) {
    const auto &r = reports[i];
    std::string offset =
        r.has_torque_offset ? fmt::format("{:.6f}", r.torque_offset) : "n/a";
    if (!r.has_calibration) {
      mc_rtc::log::info("joint_{}: torque offset {} | calibration unavailable",
                        i + 1, offset);
      continue;
    }
    // The coefficients are single precision and span several decades, so they
    // are printed with the 9 significant digits that round-trip a float: they
    // are meant to be compared between arms and archived, and {:.6f} turns
    // every gauge gain into "0.000016".
    mc_rtc::log::info("joint_{}: torque offset {} | global gain {:.9g} | "
                      "global offset {:.9g}",
                      i + 1, offset, r.global_gain, r.global_offset);
    for (size_t g = 0; g < r.gauge_gain.size(); ++g) {
      double gauge_offset = g < r.gauge_offset.size() ? r.gauge_offset[g] : 0.0;
      mc_rtc::log::info("         gauge {}: gain {:.9g} offset {:.9g}", g,
                        r.gauge_gain[g], gauge_offset);
    }
  }

  mc_rtc::log::info("===== Self-calibrated items =====");
  for (size_t i = 0; i < reports.size(); ++i) {
    const auto &r = reports[i];
    if (r.calibration_status.empty()) {
      mc_rtc::log::info("joint_{}: calibration status unavailable", i + 1);
      continue;
    }
    std::string line;
    for (const auto &entry : r.calibration_status) {
      if (!line.empty())
        line += " | ";
      line += fmt::format("{} {}", entry.first, entry.second);
    }
    mc_rtc::log::info("joint_{}: {}", i + 1, line);
  }

  mc_rtc::log::info("===== Strain gauges =====");
  for (size_t i = 0; i < reports.size(); ++i) {
    const auto &r = reports[i];
    if (!r.has_strain_gauges) {
      mc_rtc::log::info("joint_{}: strain gauge data unavailable", i + 1);
      continue;
    }
    for (size_t g = 0; g < r.gauge_raw.size(); ++g) {
      mc_rtc::log::info(
          "joint_{} gauge {}: raw [{:.0f}, {:.0f}] distinct {:<6} | value "
          "[{:.4f}, {:.4f}] Nm distinct {}",
          i + 1, g, r.gauge_raw[g].min, r.gauge_raw[g].max,
          r.gauge_raw[g].distinct.size(), r.gauge_value[g].min,
          r.gauge_value[g].max, r.gauge_value[g].distinct.size());
    }
  }
}

} // namespace

int resetServoingMode(const DiagnosticOptions &opts) {
  if (!isReachable(opts.ip_address, 10000, 2.0)) {
    mc_rtc::log::error("[mc_kortex] {} does not answer on port 10000",
                       opts.ip_address);
    return 2;
  }
  try {
    DiagnosticSession session(opts);
    auto mode = k_api::Base::ServoingModeInformation();
    mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    session.base_->SetServoingMode(mode);
    mc_rtc::log::success("[mc_kortex] {} is back in single level servoing",
                         opts.ip_address);
  } catch (const std::exception &ex) {
    mc_rtc::log::error("[mc_kortex] Could not reset the servoing mode: {}",
                       ex.what());
    return 2;
  }
  return 0;
}

/** Plausibility of a calibration about to be written
 *
 * A least squares fit of the gauge gains can be statistically excellent and
 * physically impossible: on both arms measured here the four gains of one
 * actuator span at most a factor 1.26, while an ill conditioned fit happily
 * returns spans of 15 and negative gains. These bounds are deliberately loose,
 * they only catch results that cannot describe a real sensor.
 */
bool calibrationIsPlausible(const std::vector<double> &gain,
                            const std::vector<double> &offset,
                            double global_gain, double global_offset) {
  bool ok = true;
  auto complain = [&ok](const std::string &why) {
    mc_rtc::log::error("[mc_kortex] Implausible calibration: {}", why);
    ok = false;
  };
  for (size_t g = 0; g < gain.size(); ++g) {
    if (!(gain[g] > 0.0))
      complain(fmt::format("gauge {} gain is {}, it must be strictly positive",
                           g, gain[g]));
  }
  if (ok) {
    auto mm = std::minmax_element(gain.begin(), gain.end());
    double span = *mm.second / *mm.first;
    if (span > 2.0)
      complain(fmt::format(
          "the four gains span a factor {:.2f}; no real actuator measured here "
          "exceeds 1.26, so this is a fit artefact rather than a sensor",
          span));
  }
  for (size_t g = 0; g < offset.size(); ++g) {
    if (std::abs(offset[g]) > 200.0)
      complain(fmt::format("gauge {} offset is {:.1f} N.m, far outside the "
                           "-42 to +59 seen on real actuators",
                           g, offset[g]));
  }
  if (global_gain < 0.5 || global_gain > 2.0)
    complain(fmt::format(
        "global gain is {}, it is a trim and should sit near 1", global_gain));
  if (std::abs(global_offset) > 10.0)
    complain(fmt::format("global offset is {}, it is a trim and should sit "
                         "near 0",
                         global_offset));
  return ok;
}

int writeTorqueCalibration(const DiagnosticOptions &opts) {
  mc_rtc::log::info("[mc_kortex] Writing a torque calibration to {} at {}",
                    opts.name, opts.ip_address);

  // Everything that can be checked without the arm is checked first, so a bad
  // file never reaches a session
  int joint = 0;
  std::vector<double> gain, offset;
  double global_gain = 1.0;
  double global_offset = 0.0;
  try {
    mc_rtc::Configuration cfg(opts.write_calibration_path);
    joint = cfg("joint", 0);
    gain = cfg("gain", std::vector<double>{});
    offset = cfg("offset", std::vector<double>{});
    global_gain = cfg("globalGain", 1.0);
    global_offset = cfg("globalOffset", 0.0);
  } catch (const std::exception &ex) {
    mc_rtc::log::error("[mc_kortex] Could not read {}: {}",
                       opts.write_calibration_path, ex.what());
    return 2;
  }
  if (joint < 1) {
    mc_rtc::log::error("[mc_kortex] {} does not name a joint: add a 1 based "
                       "\"joint\" entry",
                       opts.write_calibration_path);
    return 2;
  }
  if (gain.size() != STRAIN_GAUGE_COUNT ||
      offset.size() != STRAIN_GAUGE_COUNT) {
    mc_rtc::log::error("[mc_kortex] Expected {} gains and {} offsets, got {} "
                       "and {}",
                       STRAIN_GAUGE_COUNT, STRAIN_GAUGE_COUNT, gain.size(),
                       offset.size());
    return 2;
  }
  if (!calibrationIsPlausible(gain, offset, global_gain, global_offset)) {
    if (!opts.write_force) {
      mc_rtc::log::error("[mc_kortex] Refusing to write, pass "
                         "--write-torque-calibration-force to override");
      return 2;
    }
    mc_rtc::log::warning("[mc_kortex] Writing implausible coefficients anyway, "
                         "--write-torque-calibration-force was given");
  }

  if (!isReachable(opts.ip_address, 10000, 2.0)) {
    mc_rtc::log::error("[mc_kortex] {} does not answer on port 10000",
                       opts.ip_address);
    return 2;
  }

  try {
    DiagnosticSession session(opts);
    auto reports = discoverActuators(session);
    if (static_cast<size_t>(joint) > reports.size()) {
      mc_rtc::log::error("[mc_kortex] The arm has {} actuators, joint {} was "
                         "asked for",
                         reports.size(), joint);
      return 2;
    }
    auto device_id = reports[static_cast<size_t>(joint) - 1].device_id;

    auto before = session.actuator_config_->ReadTorqueCalibration(device_id);
    mc_rtc::log::info(
        "[mc_kortex] joint_{} (device {}) currently holds:", joint, device_id);
    mc_rtc::log::info("           global gain {:.9g} | global offset {:.9g}",
                      before.global_gain(), before.global_offset());
    for (int g = 0; g < before.gain_size(); ++g)
      mc_rtc::log::info("           gauge {}: gain {:.9g} offset {:.9g}", g,
                        before.gain(g),
                        g < before.offset_size() ? before.offset(g) : 0.0f);

    // Always leave a way back before changing anything
    std::string backup_path = opts.write_calibration_path +
                              fmt::format(".joint{}.before.json", joint);
    {
      mc_rtc::Configuration backup;
      backup.add("joint", joint);
      std::vector<double> bg, bo;
      for (int g = 0; g < before.gain_size(); ++g)
        bg.push_back(before.gain(g));
      for (int g = 0; g < before.offset_size(); ++g)
        bo.push_back(before.offset(g));
      backup.add("gain", bg);
      backup.add("offset", bo);
      backup.add("globalGain", static_cast<double>(before.global_gain()));
      backup.add("globalOffset", static_cast<double>(before.global_offset()));
      backup.save(backup_path);
      mc_rtc::log::success("[mc_kortex] Previous calibration saved to {}",
                           backup_path);
    }

    k_api::ActuatorConfig::TorqueCalibration wanted;
    wanted.set_global_gain(static_cast<float>(global_gain));
    wanted.set_global_offset(static_cast<float>(global_offset));
    for (size_t g = 0; g < gain.size(); ++g)
      wanted.add_gain(static_cast<float>(gain[g]));
    for (size_t g = 0; g < offset.size(); ++g)
      wanted.add_offset(static_cast<float>(offset[g]));

    session.actuator_config_->WriteTorqueCalibration(wanted, device_id);
    mc_rtc::log::info("[mc_kortex] Write accepted, reading it back");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto after = session.actuator_config_->ReadTorqueCalibration(device_id);
    bool match = true;
    auto same = [&match](const char *what, float expected, float got) {
      bool ok = expected == got;
      match = match && ok;
      mc_rtc::log::info("           {:<18} wrote {:.9g} read {:.9g} {}", what,
                        expected, got, ok ? "ok" : "MISMATCH");
    };
    mc_rtc::log::info("[mc_kortex] Verification:");
    same("global gain", static_cast<float>(global_gain), after.global_gain());
    same("global offset", static_cast<float>(global_offset),
         after.global_offset());
    for (size_t g = 0; g < gain.size(); ++g)
      same(fmt::format("gauge {} gain", g).c_str(), static_cast<float>(gain[g]),
           g < static_cast<size_t>(after.gain_size())
               ? after.gain(static_cast<int>(g))
               : 0.0f);
    for (size_t g = 0; g < offset.size(); ++g)
      same(fmt::format("gauge {} offset", g).c_str(),
           static_cast<float>(offset[g]),
           g < static_cast<size_t>(after.offset_size())
               ? after.offset(static_cast<int>(g))
               : 0.0f);

    if (!match) {
      mc_rtc::log::error("[mc_kortex] The arm did not keep what was written; "
                         "restore with {}",
                         backup_path);
      return 1;
    }
    mc_rtc::log::success("[mc_kortex] joint_{} torque calibration written and "
                         "verified",
                         joint);
    mc_rtc::log::info("[mc_kortex] Power cycle the arm to confirm it persists, "
                      "then re-run the torque sensor offset removal so this "
                      "joint is zeroed like the others");
    return 0;
  } catch (const std::exception &ex) {
    mc_rtc::log::error("[mc_kortex] Writing the torque calibration failed: {}",
                       ex.what());
    return 2;
  }
}

int runDiagnostic(const DiagnosticOptions &opts) {
  mc_rtc::log::info("[mc_kortex] Running diagnostic on {} at {}", opts.name,
                    opts.ip_address);

  if (!isReachable(opts.ip_address, 10000, 2.0)) {
    mc_rtc::log::error("[mc_kortex] {} does not answer on port 10000, check "
                       "that the arm is powered and on the network",
                       opts.ip_address);
    return 2;
  }

  ArmIdentity arm;
  std::vector<ActuatorReport> reports;
  try {
    DiagnosticSession session(opts);

    auto servoing_mode = session.base_->GetServoingMode().servoing_mode();
    if (servoing_mode != k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING) {
      mc_rtc::log::warning(
          "[mc_kortex] The arm is not in single level servoing mode, the "
          "diagnostic will still only read from it");
    }

    arm = readArmIdentity(session);
    reports = discoverActuators(session);
    if (reports.empty()) {
      mc_rtc::log::error("[mc_kortex] No actuator found on the arm");
      return 2;
    }
    mc_rtc::log::info("[mc_kortex] Found {} actuators", reports.size());

    sampleFeedback(session, reports, opts.duration);
    readTorqueConfiguration(session, reports);
    readCalibrationStatus(session, reports);

    // The actuator cyclic service answers over the TCP router, so try reading
    // the gauges without touching the servoing mode first. Low level servoing
    // is only worth the risk if that genuinely fails.
    bool gauges_read = sampleStrainGauges(
        session, reports, opts.low_level_duration, opts.dump_path);
    if (!gauges_read) {
      if (opts.low_level) {
        mc_rtc::log::warning(
            "[mc_kortex] The strain gauges did not answer in the current "
            "servoing mode, switching to low level servoing for {:.1f}s. No "
            "command is sent, but make sure the area around the arm is clear",
            opts.low_level_duration);
        auto faults_before = reports;
        {
          LowLevelServoingGuard guard(session.base_, servoing_mode);
          gauges_read = sampleStrainGauges(
              session, reports, opts.low_level_duration, opts.dump_path);
        }
        reportNewFaults(session, faults_before);
      } else {
        mc_rtc::log::info(
            "[mc_kortex] The strain gauges did not answer in the current "
            "servoing mode, pass --diagnostic-low-level to retry in low level "
            "servoing");
      }
    }
  } catch (const k_api::KDetailedException &ex) {
    mc_rtc::log::error("[mc_kortex] Diagnostic failed: {}", ex.what());
    return 2;
  } catch (const std::exception &ex) {
    mc_rtc::log::error("[mc_kortex] Diagnostic failed: {}", ex.what());
    return 2;
  }

  detectAnomalies(reports);
  printReport(arm, reports);

  mc_rtc::log::info("===== Summary =====");
  bool healthy = true;
  for (size_t i = 0; i < reports.size(); ++i) {
    const auto &r = reports[i];
    if (r.anomalies.empty()) {
      mc_rtc::log::success("joint_{}: OK", i + 1);
      continue;
    }
    healthy = false;
    for (const auto &anomaly : r.anomalies)
      mc_rtc::log::error("joint_{}: {}", i + 1, anomaly);
  }
  if (healthy) {
    mc_rtc::log::success("[mc_kortex] No anomaly detected");
    return 0;
  }
  bool zeroed_calibration =
      std::any_of(reports.begin(), reports.end(), [](const auto &r) {
        return r.has_calibration && r.torque.distinct.size() == 1 &&
               std::abs(r.global_gain) < 1e-9;
      });
  bool gauges_confirmed_alive =
      std::any_of(reports.begin(), reports.end(), [](const auto &r) {
        return r.has_strain_gauges && r.torque.distinct.size() == 1 &&
               std::all_of(
                   r.gauge_raw.begin(), r.gauge_raw.end(),
                   [](const SignalStats &s) { return s.distinct.size() > 1; });
      });
  if (gauges_confirmed_alive) {
    mc_rtc::log::warning(
        "[mc_kortex] Anomalies detected, see the report above. The strain "
        "gauges of the silent sensor are alive, so the hardware is fine and "
        "the torque calibration has to be restored on that actuator, which "
        "only Kinova can supply for a given serial number");
  } else if (zeroed_calibration) {
    mc_rtc::log::warning(
        "[mc_kortex] Anomalies detected, see the report above. A torque "
        "sensor reading a constant zero with a zeroed calibration is a "
        "configuration fault: the calibration masks the sensor, so nothing "
        "can be concluded about the gauges themselves until it is restored");
  } else {
    mc_rtc::log::warning(
        "[mc_kortex] Anomalies detected, see the report above. A torque "
        "sensor that reports a constant value while its calibration gains are "
        "non-zero points at the sensor or its communication rather than at "
        "configuration");
  }
  return 1;
}

} // namespace mc_kinova
