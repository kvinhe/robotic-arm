#pragma once

#include <functional>
#include <string>

namespace sim {

// Runs `command` against a simulated arm and phone (`scene`, a MuJoCo model) instead of the
// USB bus. Without `headless`, a window shows the arm while the command runs.
int run(const std::string& scene, bool headless, const std::function<int()>& command);

}  // namespace sim
