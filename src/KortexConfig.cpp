#include "KortexConfig.h"

namespace mc_kinova {

namespace {

/** Name of the section holding the settings shared by every robot */
constexpr auto DEFAULT_SECTION = "default";

/** Read the connection parameters a section defines, leaving the others
 * untouched
 *
 * \returns True if the section defined at least one of them
 */
bool readConnectionParameters(const mc_rtc::Configuration &config,
                              ConnectionParameters &params) {
  bool defined =
      config.has("ip") || config.has("username") || config.has("password");
  config("ip", params.ip_address);
  config("username", params.username);
  config("password", params.password);
  return defined;
}

/** The settings a Kortex configuration shares between every robot
 *
 * Everything the Kortex section holds is a shared setting, except the robot
 * sections and the "default" section itself, which is merged on top: a
 * configuration may hold both, as one written before "default" existed does
 * when it gains a "default" section for the connection parameters alone.
 */
mc_rtc::Configuration
defaultSettings(const mc_rtc::Configuration &kortexConfig) {
  mc_rtc::Configuration config;
  config.load(kortexConfig);
  config.remove(DEFAULT_SECTION);
  for (const auto &section : robotSections(kortexConfig)) {
    config.remove(section);
  }

  if (kortexConfig.has(DEFAULT_SECTION) &&
      kortexConfig(DEFAULT_SECTION).isObject()) {
    config.load(kortexConfig(DEFAULT_SECTION));
  }

  return config;
}

} // namespace

mc_rtc::Configuration
robotConfiguration(const mc_rtc::Configuration &kortexConfig,
                   const std::string &name) {
  // load() merges recursively into a fresh object, leaving the configuration
  // it reads from untouched: every robot gets its own copy of the defaults
  mc_rtc::Configuration config;
  config.load(defaultSettings(kortexConfig));

  if (!name.empty() && kortexConfig.has(name) &&
      kortexConfig(name).isObject()) {
    config.load(kortexConfig(name));
  }

  return config;
}

ConnectionParameters
connectionParameters(const mc_rtc::Configuration &robotConfig) {
  ConnectionParameters params;
  readConnectionParameters(robotConfig, params);
  return params;
}

ConnectionParameters
connectionParameters(const mc_rtc::Configuration &kortexConfig,
                     const std::string &name) {
  return connectionParameters(robotConfiguration(kortexConfig, name));
}

std::vector<std::string>
robotSections(const mc_rtc::Configuration &kortexConfig) {
  std::vector<std::string> sections;
  for (const auto &key : kortexConfig.keys()) {
    if (key == DEFAULT_SECTION) {
      continue;
    }
    auto entry = kortexConfig(key);
    if (!entry.isObject()) {
      continue;
    }
    ConnectionParameters unused;
    if (!readConnectionParameters(entry, unused)) {
      continue;
    }
    sections.push_back(key);
  }
  return sections;
}

} // namespace mc_kinova
