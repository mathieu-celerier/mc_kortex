#include "KinovaRobot.h"
#include <mc_control/mc_global_controller.h>

namespace mc_kortex {

struct ControlLoopDataBase {
  ControlLoopDataBase()
      : controller(nullptr), controller_run(nullptr), kinova_threads(nullptr),
        controller_run_id(0) {}
  mc_control::MCGlobalController *controller;
  std::thread *controller_run;
  std::mutex controller_run_mutex;
  std::condition_variable controller_run_cv;
  uint64_t controller_run_id;
  std::vector<std::thread> *kinova_threads;
};

struct ControlLoopData : public ControlLoopDataBase {
  ControlLoopData() : ControlLoopDataBase(), kinovas(nullptr) {}
  std::vector<mc_kinova::KinovaRobotPtr> *kinovas;
};

void *global_thread_init(
    mc_control::MCGlobalController::GlobalConfiguration &gconfig,
    bool debug_enabled = false);

void run(void *data);

} // namespace mc_kortex
