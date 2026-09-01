#include "KinovaDiagnostic.h"
#include "KortexConfig.h"
#include "mc_kortex.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <chrono>
#include <iostream>
#include <set>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char *argv[]) {
  // ==================== Initialization ==================== //
  if (mc_rtc::MC_RTC_VERSION != mc_rtc::version()) {
    mc_rtc::log::error(
        "mc_kortex was compiled with {} but mc_rtc is at version {}, you might "
        "face subtle issues or unexpected crashes, please recompile mc_kortex",
        mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  std::string conf_file = "";
  po::options_description desc("mc_kortex options");
  // clang-format off
    desc.add_options()
        ("help", "Show this help message")
        ("init-only", po::bool_switch(), "Debug usage")
        ("diagnostic", po::bool_switch(), "Run a read-only hardware diagnostic of the arm and exit")
        ("diagnostic-duration", po::value<double>()->default_value(3.0), "Sampling window of the diagnostic, in seconds")
        ("diagnostic-low-level", po::bool_switch(), "Fallback: if the strain gauges do not answer in the current servoing mode, retry inside a brief low level servoing window")
        ("diagnostic-low-level-duration", po::value<double>()->default_value(1.0), "Strain gauge sampling window, in seconds (a sample count over a nominal 100Hz, the real rate is closer to 6Hz)")
        ("diagnostic-dump", po::value<std::string>()->default_value(""), "Write every strain gauge sample to this CSV file")
        ("diagnostic-reset-servoing", po::bool_switch(), "Put the arm back in single level servoing and exit, for when a diagnostic was interrupted")
        ("robot", po::value<std::string>()->default_value(""), "Restrict the diagnostic to a single robot of the Kortex configuration")
        ("write-torque-calibration", po::value<std::string>()->default_value(""), "Write the torque calibration described by this JSON file to one actuator, then read it back and verify. The only mode that writes to the arm")
        ("write-torque-calibration-force", po::bool_switch(), "Write the coefficients even if they fail the plausibility checks");
  // clang-format on

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  po::notify(vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file,
                                                              nullptr);
  if (!gconfig.config.has("Kortex")) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No Kortex section in the configuration");
  }
  auto kortexConfig = gconfig.config("Kortex");

  // ==================== Diagnostic mode ==================== //
  // The diagnostic talks to the arm directly: it must stay usable when the
  // controller itself cannot be started.
  if (vm["diagnostic"].as<bool>() ||
      vm["diagnostic-reset-servoing"].as<bool>() ||
      !vm["write-torque-calibration"].as<std::string>().empty()) {
    auto only_robot = vm["robot"].as<std::string>();
    int status = 0;
    size_t diagnosed = 0;

    // What to diagnose: the robot asked for, or the connection defaults
    // followed by every robot section that overrides them
    std::vector<std::pair<std::string, mc_kinova::ConnectionParameters>>
        targets;
    if (!only_robot.empty()) {
      targets.emplace_back(only_robot, mc_kinova::connectionParameters(
                                           kortexConfig, only_robot));
    } else {
      auto defaults = mc_kinova::connectionParameters(kortexConfig, "");
      if (!defaults.ip_address.empty()) {
        targets.emplace_back("default", defaults);
      }
      for (const auto &name : mc_kinova::robotSections(kortexConfig)) {
        targets.emplace_back(
            name, mc_kinova::connectionParameters(kortexConfig, name));
      }
    }

    // Several robot entries usually point at the same arm, diagnosing it once
    // is enough
    std::set<std::string> diagnosed_ips;
    for (const auto &target : targets) {
      const auto &key = target.first;
      const auto &params = target.second;
      if (params.ip_address.empty()) {
        continue;
      }
      if (!diagnosed_ips.insert(params.ip_address).second) {
        mc_rtc::log::info("[mc_kortex] Skipping {}, {} was already diagnosed",
                          key, params.ip_address);
        continue;
      }
      mc_kinova::DiagnosticOptions opts;
      opts.name = key;
      opts.ip_address = params.ip_address;
      opts.username = params.username;
      opts.password = params.password;
      opts.duration = vm["diagnostic-duration"].as<double>();
      opts.low_level = vm["diagnostic-low-level"].as<bool>();
      opts.low_level_duration =
          vm["diagnostic-low-level-duration"].as<double>();
      opts.dump_path = vm["diagnostic-dump"].as<std::string>();
      opts.write_calibration_path =
          vm["write-torque-calibration"].as<std::string>();
      opts.write_force = vm["write-torque-calibration-force"].as<bool>();
      if (!opts.write_calibration_path.empty()) {
        status = std::max(status, mc_kinova::writeTorqueCalibration(opts));
      } else if (vm["diagnostic-reset-servoing"].as<bool>()) {
        status = std::max(status, mc_kinova::resetServoingMode(opts));
      } else {
        status = std::max(status, mc_kinova::runDiagnostic(opts));
      }
      diagnosed += 1;
    }
    if (diagnosed == 0) {
      mc_rtc::log::error(
          "[mc_kortex] No robot to diagnose in the Kortex configuration{}",
          only_robot.empty() ? "" : fmt::format(" matching {}", only_robot));
      return 2;
    }
    return status;
  }

  void *data = mc_kortex::global_thread_init(gconfig);
  if (!data) {
    printf("Initialization failed\n");
    return -2;
  }

  // ==================== Run control loop ==================== //
  if (!vm["init-only"].as<bool>())
    mc_kortex::run(data);

  return 0;
}
