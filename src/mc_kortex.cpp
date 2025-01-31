#include "mc_kortex.h"

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

  // Initialize controlled kinova robot
  loop_data->kinovas = new std::vector<mc_kinova::KinovaRobotPtr>();
  auto &kinovas = *loop_data->kinovas;
  {
    std::vector<std::thread> kinova_init_threads;
    std::mutex kinova_init_mutex;
    std::condition_variable kinova_init_cv;
    bool kinovas_init_ready = false;
    for (auto &robot : robots) {
      if (robot.mb().nrDof() == 0) {
        continue;
      }
      if (kortexConfig.has(robot.name())) {
        std::string ip = kortexConfig(robot.name())("ip");
        std::string username = kortexConfig(robot.name())("username");
        std::string password = kortexConfig(robot.name())("password");
        kinova_init_threads.emplace_back([&, ip, username, password]() {
          {
            std::unique_lock<std::mutex> lock(kinova_init_mutex);
            kinova_init_cv.wait(
                lock, [&kinovas_init_ready]() { return kinovas_init_ready; });
          }
          auto kinova = std::unique_ptr<mc_kinova::KinovaRobot>(
              new mc_kinova::KinovaRobot(robot.name(), ip, username, password));
          std::unique_lock<std::mutex> lock(kinova_init_mutex);
          kinovas.emplace_back(std::move(kinova));
        });
      } else {
        mc_rtc::log::warning("The loaded controller uses an actuated robot "
                             "that is not configured and not ignored: {}",
                             robot.name());
      }
    }
    kinovas_init_ready = true;
    kinova_init_cv.notify_all();
    for (auto &th : kinova_init_threads) {
      th.join();
    }
  }
  for (auto &kinova : kinovas) {
    kinova->init(controller, kortexConfig);
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
    if (now - last > controller.timestep() * 1e6) {
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
