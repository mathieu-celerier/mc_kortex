#include "KinovaRobot.h"
#include <mc_control/mc_global_controller.h>

namespace mc_kortex {

struct ControlLoopDataBase {
  ControlLoopDataBase() : controller(nullptr) {}
  mc_control::MCGlobalController *controller;
};

struct ControlLoopData : public ControlLoopDataBase {
  ControlLoopData() : ControlLoopDataBase(), kinovas(nullptr) {}
  std::vector<mc_kinova::KinovaRobotPtr> *kinovas;
};

void *global_thread_init(
    mc_control::MCGlobalController::GlobalConfiguration &gconfig);

void run(void *data);

} // namespace mc_kortex
