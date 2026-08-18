#include "mc_kortex.h"

#include "KortexConfig.h"

#include <ctime>

namespace mc_kortex {

void *global_thread_init(
    mc_control::MCGlobalController::GlobalConfiguration &gconfig) {
  auto kortexConfig = gconfig.config("Kortex");
  auto loop_data = new ControlLoopData();
  // Create mc_rtc's global controller
  loop_data->controller = new mc_control::MCGlobalController(gconfig);
  loop_data->kinova_threads = new std::vector<std::thread>();
  auto &controller = *loop_data->controller;
  if (controller.controller().timeStep < 0.001) {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_kortex] mc_rtc cannot run faster than 1kHz with mc_kortex");
  }
  size_t freq = std::ceil(1 / controller.controller().timeStep);
  mc_rtc::log::info("[mc_kortex] mc_rtc running at {}Hz", freq);
  auto &robots = controller.controller().robots();
  // Initialize all real robots
  for (size_t i = controller.realRobots().size(); i < robots.size(); ++i) {
    controller.realRobots().robotCopy(robots.robot(i), robots.robot(i).name());
  }

  // A section that names an arm to reach, yet no robot of the controller:
  // most likely a misspelled robot name. Every other Kortex level section is
  // a shared setting and configures all of them.
  for (const auto &name : mc_kinova::robotSections(kortexConfig)) {
    if (robots.hasRobot(name)) {
      continue;
    }
    mc_rtc::log::warning("[mc_kortex] Ignoring the \"{}\" section: no robot of "
                         "the controller goes by that name",
                         name);
  }

  // Initialize controlled kinova robot
  loop_data->kinovas = new std::vector<mc_kinova::KinovaRobotPtr>();
  auto &kinovas = *loop_data->kinovas;
  // Configuration of each robot, defaults included, kept for init() below
  std::map<std::string, mc_rtc::Configuration> kinova_configs;
  {
    std::vector<std::thread> kinova_init_threads;
    std::mutex kinova_init_mutex;
    std::condition_variable kinova_init_cv;
    bool kinovas_init_ready = false;
    for (auto &robot : robots) {
      if (robot.mb().nrDof() == 0) {
        continue;
      }
      // The "default" section applies to every robot, a section named after
      // the robot overrides it
      auto robotConfig =
          mc_kinova::robotConfiguration(kortexConfig, robot.name());
      auto params = mc_kinova::connectionParameters(robotConfig);
      if (params.ip_address.empty()) {
        mc_rtc::log::warning("The loaded controller uses an actuated robot "
                             "that is not configured and not ignored: {}",
                             robot.name());
        continue;
      }
      mc_rtc::log::info("[mc_kortex] {} robot will connect to {}", robot.name(),
                        params.ip_address);
      kinova_configs[robot.name()] = robotConfig;
      // The name is copied: the threads run once the loop, and with it the
      // robot binding, is gone
      kinova_init_threads.emplace_back([&, name = robot.name(), params]() {
        {
          std::unique_lock<std::mutex> lock(kinova_init_mutex);
          kinova_init_cv.wait(
              lock, [&kinovas_init_ready]() { return kinovas_init_ready; });
        }
        auto kinova =
            std::unique_ptr<mc_kinova::KinovaRobot>(new mc_kinova::KinovaRobot(
                name, params.ip_address, params.username, params.password));
        std::unique_lock<std::mutex> lock(kinova_init_mutex);
        kinovas.emplace_back(std::move(kinova));
      });
    }
    kinovas_init_ready = true;
    kinova_init_cv.notify_all();
    for (auto &th : kinova_init_threads) {
      th.join();
    }
  }
  for (auto &kinova : kinovas) {
    kinova->init(controller, kinova_configs[kinova->getName()]);
  }
  std::vector<double> qInit = robots.robot().encoderValues();
  mc_rtc::log::info("qInit = {}", mc_kinova::printVec(qInit));
  controller.init(qInit);
  controller.running = true;
  controller.controller().gui()->addElement(
      {"Kortex"}, mc_rtc::gui::Button("Stop controller", [&controller]() {
        controller.running = false;
      }));

  // Start control loops
  static std::mutex startMutex;
  static std::condition_variable startCV;
  static bool startControl = false;
  for (auto &kinova : kinovas) {
    loop_data->kinova_threads->emplace_back([&]() {
      kinova->controlThread(controller, startMutex, startCV, startControl,
                            controller.running);
    });
  }
  startControl = true;
  startCV.notify_all();

  return loop_data;
}

void run(void *data) {
  mc_rtc::log::info("[mc_kortex] Starting control loop");
  auto control_data = static_cast<mc_kortex::ControlLoopData *>(data);
  auto controller_ptr = control_data->controller;
  auto &controller = *controller_ptr;
  auto &kinovas = *control_data->kinovas;

  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  double now = 0;
  double last = ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3;
  controller.controller().logger().addLogEntry(
      "perf_LoopDt", [&]() { return (now - last) / 1000; });

  while (controller.running) {
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now = ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3;
    if (now - last <= controller.timestep() * 1e6) {
      // Wait for the next deadline instead of spinning on the clock: this is
      // not the real-time loop, the per robot control threads are
      double deadline_us = last + controller.timestep() * 1e6;
      timespec deadline;
      deadline.tv_sec = static_cast<time_t>(deadline_us / 1e6);
      deadline.tv_nsec =
          static_cast<long>((deadline_us - deadline.tv_sec * 1e6) * 1e3);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
      continue;
    }
    {
      // mc_rtc::log::info("[mc_kortex] Control loop elapsed time {}ms",
      // (now-last)*1e-3);
      for (auto &kinova : kinovas) {
        if (controller.controller().datastore().has("TorqueMode"))
          kinova->setTorqueMode(
              controller.controller().datastore().get<std::string>(
                  "TorqueMode"));
        if (controller.controller().datastore().has("ControlMode"))
          kinova->setControlMode(
              controller.controller().datastore().get<std::string>(
                  "ControlMode"));
        kinova->updateSensors(controller);
      }

      // Run the controller
      controller.run();

      for (auto &kinova : kinovas) {
        kinova->updateControl(controller);
      }

      last = now;
    }
  }

  for (auto &kinova : kinovas) {
    kinova->stopController();
  }

  for (auto &th : *control_data->kinova_threads) {
    th.join();
  }

  delete control_data->kinovas;
  delete controller_ptr;
}

} // namespace mc_kortex
