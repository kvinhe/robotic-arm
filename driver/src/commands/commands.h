#pragma once

#include <string>
#include <vector>

namespace arm {

// The servo_tool commands that drive the whole arm. `port` may be empty to auto-detect;
// `args` are the command's own arguments.
int record_command(const std::string& port, const std::vector<std::string>& args);
int replay_command(const std::string& port, const std::vector<std::string>& args);
int calibrate_command(const std::string& port, const std::vector<std::string>& args);
int where_command(const std::string& port, const std::vector<std::string>& args);
int move_command(const std::string& port, const std::vector<std::string>& args);
int tap_command(const std::string& port, const std::vector<std::string>& args);
int swipe_command(const std::string& port, const std::vector<std::string>& args);
int unlock_command(const std::string& port, const std::vector<std::string>& args);

}  // namespace arm
