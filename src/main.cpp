#include "mc_kortex.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <chrono>
#include <iostream>

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
        ("init-only", po::bool_switch(), "Debug usage");
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
