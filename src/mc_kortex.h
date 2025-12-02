#include "KinovaControlLoop.h"
#include <mc_control/mc_global_controller.h>

namespace mc_kortex {

struct ControlLoopDataBase {
  ControlLoopDataBase() : controller(nullptr), kinova_threads(nullptr) {}
  mc_control::MCGlobalController *controller;
  std::vector<std::thread> *kinova_threads;
};

struct ControlLoopData : public ControlLoopDataBase {
  ControlLoopData() : ControlLoopDataBase(), kinovas(nullptr) {}
  std::vector<mc_kinova::KinovaRobotPtr> *kinovas;
};

void *global_thread_init(
    mc_control::MCGlobalController::GlobalConfiguration &gconfig);

void run(void *data);

} // namespace mc_kortex
