#include "mc_kortex.h"

#include "KortexConfig.h"

namespace mc_kortex {

void *global_thread_init(
    mc_control::MCGlobalController::GlobalConfiguration &gconfig) {
  auto kortexConfig = gconfig.config("Kortex");
  auto loop_data = new ControlLoopData();
  // Create mc_rtc's global controller
  loop_data->controller = new mc_control::MCGlobalController(gconfig);
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
    {
      // The predicate must be published under the mutex, otherwise a thread
      // that is about to wait can miss the notification
      std::unique_lock<std::mutex> lock(kinova_init_mutex);
      kinovas_init_ready = true;
    }
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

  return loop_data;
}

void run(void *data) {
  mc_rtc::log::info("[mc_kortex] Starting control loop");
  auto control_data = static_cast<mc_kortex::ControlLoopData *>(data);
  auto controller_ptr = control_data->controller;
  auto &controller = *controller_ptr;
  auto &kinovas = *control_data->kinovas;

  auto now_us = []() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
  };
  const int64_t period_us =
      static_cast<int64_t>(std::llround(controller.timestep() * 1e6));
  int64_t now = now_us();
  int64_t last = now;
  int64_t next_tick = now;
  int64_t overruns = 0;
  controller.controller().logger().addLogEntry(
      "perf_LoopDt", [&]() { return static_cast<double>(now - last) / 1000; });
  controller.controller().logger().addLogEntry("perf_LoopOverruns",
                                               [&]() { return overruns; });
  // Duration of each phase of a tick (µs). The log is written while the
  // controller runs, mid-tick: the entries hold the previous, complete tick
  struct TickTiming {
    int64_t start_delay = 0;    // tick start after its scheduled time
    int64_t send = 0;           // sendCommand() of every arm
    int64_t wait = 0;           // receiveFeedback() of every arm
    int64_t wait_overshoot = 0; // end of the wait after its deadline
    int64_t sensors = 0;        // setControlMode()/updateSensors()
    int64_t controller = 0;     // controller.run(), logging included
    int64_t build = 0;          // updateControl()/buildCommand()
    int64_t total = 0;          // whole tick
  };
  TickTiming timing, last_timing;
  auto &logger = controller.controller().logger();
  logger.addLogEntry("perf_Tick_startDelay",
                     [&]() { return last_timing.start_delay; });
  logger.addLogEntry("perf_Tick_send", [&]() { return last_timing.send; });
  logger.addLogEntry("perf_Tick_wait", [&]() { return last_timing.wait; });
  logger.addLogEntry("perf_Tick_waitOvershoot",
                     [&]() { return last_timing.wait_overshoot; });
  logger.addLogEntry("perf_Tick_sensors",
                     [&]() { return last_timing.sensors; });
  logger.addLogEntry("perf_Tick_controller",
                     [&]() { return last_timing.controller; });
  logger.addLogEntry("perf_Tick_build", [&]() { return last_timing.build; });
  logger.addLogEntry("perf_Tick_total", [&]() { return last_timing.total; });

  // How long into a tick its feedback is awaited: the shortest of the arms
  int64_t feedback_wait_us = period_us;
  for (auto &kinova : kinovas) {
    feedback_wait_us =
        std::min<int64_t>(feedback_wait_us, kinova->feedbackWaitUs());
  }
  mc_rtc::log::info("[mc_kortex] Waiting up to {}us for the feedback of each "
                    "tick",
                    feedback_wait_us);

  // One thread runs the whole tick:
  //  1. every arm sends its command, at a fixed rate, then waits for its
  //     feedback until feedback_wait_us into the tick (the arms in parallel).
  //     A late reply is used at a later tick, the controller meanwhile runs
  //     on the last feedback
  //  2. sensors are updated from that feedback and the controller runs
  //  3. the command for the next tick is built from the controller output
  try {
    // Inside the try: an arm that fails to enter low level servoing, halfway
    // through, must still be brought back to single level servoing below
    for (auto &kinova : kinovas) {
      kinova->startControl(controller);
    }
    while (controller.running) {
      // A tick that ran more than half a period late skips the ticks it
      // overlapped, rather than sending the next commands in a burst to catch
      // up: commands never leave less than half a period apart
      now = now_us();
      while (now - next_tick > period_us / 2) {
        next_tick += period_us;
        overruns++;
      }
      // Deliberate spin: this is the real-time thread, it must hit its 1kHz
      // deadline and must not be descheduled by a sleep
      do {
        now = now_us();
      } while (now < next_tick);
      const int64_t tick_start = now;
      timing.start_delay = tick_start - next_tick;
      next_tick += period_us;

      for (auto &kinova : kinovas) {
        kinova->sendCommand();
      }
      int64_t t_sent = now_us();
      timing.send = t_sent - tick_start;
      const int64_t deadline = tick_start + feedback_wait_us;
      for (auto &kinova : kinovas) {
        kinova->receiveFeedback(deadline, controller.running);
      }
      int64_t t_received = now_us();
      timing.wait = t_received - t_sent;
      timing.wait_overshoot = t_received - deadline;

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
      int64_t t_sensors = now_us();
      timing.sensors = t_sensors - t_received;

      // Run the controller
      controller.run();
      int64_t t_controller = now_us();
      timing.controller = t_controller - t_sensors;

      for (auto &kinova : kinovas) {
        kinova->updateControl(controller);
        kinova->buildCommand(controller.robots().robot(kinova->getName()),
                             controller.running);
      }
      int64_t t_built = now_us();
      timing.build = t_built - t_controller;
      timing.total = t_built - tick_start;
      last_timing = timing;

      last = now;
    }
  } catch (std::exception &ex) {
    mc_rtc::log::error("[mc_kortex] Control loop error: {}", ex.what());
  }

  for (auto &kinova : kinovas) {
    kinova->stopControl(controller);
  }

  controller.controller().logger().removeLogEntry("perf_LoopDt");
  controller.controller().logger().removeLogEntry("perf_LoopOverruns");
  for (const auto &entry :
       {"perf_Tick_startDelay", "perf_Tick_send", "perf_Tick_wait",
        "perf_Tick_waitOvershoot", "perf_Tick_sensors", "perf_Tick_controller",
        "perf_Tick_build", "perf_Tick_total"}) {
    controller.controller().logger().removeLogEntry(entry);
  }

  delete control_data->kinovas;
  delete controller_ptr;
  delete control_data;
}

} // namespace mc_kortex
