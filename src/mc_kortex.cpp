#include "mc_kortex.h"

#include <mc_rtc_ros/ros.h>

namespace mc_kortex {

namespace {

void stopRobotPublishers(mc_control::MCGlobalController &controller) {
  auto stop_publishers = [](const std::string &prefix, mc_rbdyn::Robots &robots) {
    for (auto &r : robots) {
      mc_rtc::ROSBridge::stop_robot_publisher(prefix + r.name());
    }
  };

  stop_publishers("control/", controller.controller().robots());
  stop_publishers("real/", controller.controller().realRobots());
}

} // namespace

void *global_thread_init(
    mc_control::MCGlobalController::GlobalConfiguration &gconfig,
    bool debug_enabled) {
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
              new mc_kinova::KinovaRobot(robot.name(), ip, username, password,
                                         debug_enabled));
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
  for (auto &kinova : kinovas) {
    kinova->updateControl(controller);
  }
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
    auto *kinova_ptr = kinova.get();
    loop_data->kinova_threads->emplace_back([&, kinova_ptr]() {
      kinova_ptr->controlThread(controller, startMutex, startCV, startControl,
                                controller.running);
    });
  }
  startControl = true;
  startCV.notify_all();

  loop_data->controller_run = new std::thread([loop_data]() {
    auto controller_ptr = loop_data->controller;
    auto &controller = *controller_ptr;
    auto &kinovas = *loop_data->kinovas;
    uint64_t last_run_id = 0;

    while (controller.running) {
      {
        std::unique_lock<std::mutex> lock(loop_data->controller_run_mutex);
        loop_data->controller_run_cv.wait(lock, [&]() {
          return !controller.running ||
                 loop_data->controller_run_id != last_run_id;
        });
        if (!controller.running) {
          break;
        }
        last_run_id = loop_data->controller_run_id;
      }

      bool kinovas_ready = true;
      for (auto &kinova : kinovas) {
        kinovas_ready = kinovas_ready && kinova->controlReady();
      }
      if (!kinovas_ready) {
        continue;
      }

      for (auto &kinova : kinovas) {
        if (controller.controller().datastore().has("TorqueMode")) {
          kinova->setTorqueMode(
              controller.controller().datastore().get<std::string>(
                  "TorqueMode"));
        }
        if (controller.controller().datastore().has("ControlMode")) {
          kinova->setControlMode(
              controller.controller().datastore().get<std::string>(
                  "ControlMode"));
        }
        kinova->updateSensors(controller);
      }

      controller.run();

      for (auto &kinova : kinovas) {
        kinova->updateControl(controller);
      }
    }
  });

  return loop_data;
}

void run(void *data) {
  auto control_data = static_cast<mc_kortex::ControlLoopData *>(data);
  auto controller_ptr = control_data->controller;
  auto &controller = *controller_ptr;
  auto &kinovas = *control_data->kinovas;

  while (controller.running) {
    bool kinovas_ready = true;
    for (auto &kinova : kinovas) {
      kinovas_ready = kinovas_ready && kinova->controlReady();
    }
    if (kinovas_ready) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  mc_rtc::log::info(
      "[mc_kortex] Kinova threads ready, entering periodic control loop");

  auto next_cycle = std::chrono::steady_clock::now();
  double loop_dt_ms = 0.0;
  auto last_cycle = next_cycle;
  controller.controller().logger().addLogEntry(
      "perf_LoopDt", [&]() { return loop_dt_ms; });

  while (controller.running) {
    next_cycle += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(controller.timestep()));
    std::this_thread::sleep_until(next_cycle);
    const auto now = std::chrono::steady_clock::now();
    loop_dt_ms =
        std::chrono::duration<double, std::milli>(now - last_cycle).count();
    last_cycle = now;

    {
      std::lock_guard<std::mutex> lock(control_data->controller_run_mutex);
      control_data->controller_run_id++;
    }
    control_data->controller_run_cv.notify_one();
  }

  {
    std::lock_guard<std::mutex> lock(control_data->controller_run_mutex);
    control_data->controller_run_id++;
  }
  control_data->controller_run_cv.notify_all();

  for (auto &kinova : kinovas) {
    kinova->stopController();
  }

  if (control_data->controller_run) {
    control_data->controller_run->join();
    delete control_data->controller_run;
  }

  for (auto &th : *control_data->kinova_threads) {
    th.join();
  }

  stopRobotPublishers(controller);

  delete control_data->kinovas;
  delete control_data->kinova_threads;
  delete controller_ptr;
  delete control_data;
}

} // namespace mc_kortex
