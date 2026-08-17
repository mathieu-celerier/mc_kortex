#pragma once

#include <string>

namespace mc_kinova {

struct DiagnosticOptions {
  std::string name;
  std::string ip_address;
  std::string username = "admin";
  std::string password = "admin";
  /** Duration of the live sampling window, in seconds */
  double duration = 3.0;
  /** Read the individual strain gauges of every torque sensor
   *
   * This requires putting the arm in low level servoing mode, which is the
   * only mode where the actuator cyclic service is served. The diagnostic
   * sends no command while in that mode and restores the previous servoing
   * mode straight after, but the arm is briefly in its most permissive mode,
   * hence the explicit opt-in.
   */
  bool low_level = false;
  /** Duration of the low level window, in seconds. Kept separate from
   * duration, and short, to limit the time spent in low level servoing. */
  double low_level_duration = 1.0;
  /** If set, write every strain gauge sample of the low level window to this
   * CSV file, alongside the torque every actuator reports at the same
   * instant. Use it to check that the gauges react to a load applied by hand.
   */
  std::string dump_path;
};

/** Connect to a Kinova arm and print a hardware diagnostic report
 *
 * The report is read-only: no control mode, servoing mode or calibration is
 * ever written to the arm. It reports, for every actuator:
 *   - identity (type, serial number, firmware version)
 *   - live feedback statistics over a sampling window (torque, current,
 *     voltage, temperatures) including the number of distinct torque values,
 *     which is what tells a dead torque sensor apart from a quiet one
 *   - the fault and warning banks
 *   - the torque offset and the torque calibration (global gain/offset plus
 *     the per-strain-gauge gains and offsets)
 *   - the raw ADC value and converted value of each of the four strain gauges
 *     making up the torque sensor
 *
 * Returns 0 if every actuator looks healthy, 1 if at least one anomaly was
 * detected, 2 if the arm could not be reached.
 */
int runDiagnostic(const DiagnosticOptions &opts);

/** Put the arm back in single level servoing
 *
 * Recovery path for when a diagnostic using the low level window was killed
 * before it could restore the servoing mode itself.
 */
int resetServoingMode(const DiagnosticOptions &opts);

} // namespace mc_kinova
