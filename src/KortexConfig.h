#pragma once

#include <mc_rtc/Configuration.h>

#include <string>
#include <vector>

namespace mc_kinova {

/** Parameters needed to open a session with one arm */
struct ConnectionParameters {
  std::string ip_address;
  std::string username = "admin";
  std::string password = "admin";
};

/** Configuration of one robot
 *
 * The shared settings are everything the Kortex section holds outside of the
 * robot sections, the "default" section included and merged last; a section
 * named after the robot overrides them, recursively, key by key. Pass an empty
 * name to read the shared settings alone.
 *
 * Configurations written before the "default" section existed, which put the
 * settings directly at the Kortex level, are read exactly as they were, and
 * the two layouts may be mixed: a "default" section holding the connection
 * parameters next to settings left at the Kortex level.
 */
mc_rtc::Configuration
robotConfiguration(const mc_rtc::Configuration &kortexConfig,
                   const std::string &name);

/** Connection parameters held by an already resolved robot configuration */
ConnectionParameters
connectionParameters(const mc_rtc::Configuration &robotConfig);

/** Connection parameters of a robot, resolved as robotConfiguration() does
 *
 * An empty ip_address in the result means the robot is not configured: no
 * default was given and the robot has no address of its own.
 */
ConnectionParameters
connectionParameters(const mc_rtc::Configuration &kortexConfig,
                     const std::string &name);

/** Names of the robot sections of a Kortex configuration, in configuration
 * order
 *
 * A section is reported when it defines at least one connection parameter,
 * which is what tells a robot apart from the sections configuring the
 * interface itself. Used to enumerate the arms to reach, so a section that
 * only overrides settings is of no interest here.
 */
std::vector<std::string>
robotSections(const mc_rtc::Configuration &kortexConfig);

} // namespace mc_kinova
