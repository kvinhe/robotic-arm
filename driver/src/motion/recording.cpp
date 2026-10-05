#include "motion/recording.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace arm {
namespace {
constexpr const char* kHeader = "# arm recording v1";
}

Pose Recording::min() const {
    Pose lo = first();
    for (const Sample& s : samples_) {
        for (size_t j = 0; j < kJointCount; ++j) lo[j] = std::min(lo[j], s.pose[j]);
    }
    return lo;
}

Pose Recording::max() const {
    Pose hi = first();
    for (const Sample& s : samples_) {
        for (size_t j = 0; j < kJointCount; ++j) hi[j] = std::max(hi[j], s.pose[j]);
    }
    return hi;
}

Pose Recording::at(double t) const {
    if (t <= 0.0) return first();
    if (t >= duration()) return last();

    const auto after = std::upper_bound(samples_.begin(), samples_.end(), t,
                                        [](double time, const Sample& s) { return time < s.t; });
    const Sample& b = *after;
    const Sample& a = *(after - 1);
    const double span = b.t - a.t;
    const double f = span > 0.0 ? (t - a.t) / span : 0.0;

    Pose p{};
    for (size_t j = 0; j < kJointCount; ++j) {
        p[j] = static_cast<int>(std::lround(a.pose[j] + f * (b.pose[j] - a.pose[j])));
    }
    return p;
}

Recording Recording::reversed() const {
    std::vector<Sample> out;
    out.reserve(samples_.size());
    const double d = duration();
    for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
        out.push_back({d - it->t, it->pose});
    }
    return Recording(std::move(out));
}

Recording Recording::retraced() const {
    std::vector<Sample> out = samples_;
    const double d = duration();
    // Mirror every sample except the turnaround point about t = d.
    for (auto it = samples_.rbegin() + 1; it != samples_.rend(); ++it) {
        out.push_back({2.0 * d - it->t, it->pose});
    }
    return Recording(std::move(out));
}

Recording Recording::scaled_to(double seconds) const {
    const double factor = duration() > 0.0 ? seconds / duration() : 1.0;
    std::vector<Sample> out = samples_;
    for (Sample& s : out) s.t *= factor;
    return Recording(std::move(out));
}

double Recording::peak_speed(double window) const {
    double peak = 0.0;
    size_t j = 0;
    for (size_t i = 0; i < samples_.size(); ++i) {
        if (j < i) j = i;
        while (j < samples_.size() && samples_[j].t - samples_[i].t < window) ++j;
        if (j == samples_.size()) break;
        const double dt = samples_[j].t - samples_[i].t;
        for (size_t k = 0; k < kJointCount; ++k) {
            peak = std::max(peak, std::abs(samples_[j].pose[k] - samples_[i].pose[k]) / dt);
        }
    }
    return peak;
}

bool Recording::save(const std::string& path, std::string* error) const {
    std::ofstream file(path);
    if (!file) {
        *error = "cannot write " + path;
        return false;
    }
    file << kHeader << "\n# t_seconds";
    for (const char* name : kJointNames) file << ' ' << name;
    file << '\n';
    char time[32];
    for (const Sample& s : samples_) {
        std::snprintf(time, sizeof(time), "%.3f", s.t);
        file << time;
        for (int p : s.pose) file << ' ' << p;
        file << '\n';
    }
    if (!file) {
        *error = "write to " + path + " failed";
        return false;
    }
    return true;
}

bool Recording::load(const std::string& path, Recording* out, std::string* error) {
    std::ifstream file(path);
    if (!file) {
        *error = "cannot open " + path;
        return false;
    }

    std::string line;
    if (!std::getline(file, line) || line != kHeader) {
        *error = path + " is not an arm recording (first line should be \"" + kHeader + "\")";
        return false;
    }

    std::vector<Sample> samples;
    int line_number = 1;
    while (std::getline(file, line)) {
        ++line_number;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream fields(line);
        Sample s{};
        fields >> s.t;
        for (int& p : s.pose) fields >> p;
        std::string extra;
        const std::string where = path + ":" + std::to_string(line_number);
        if (fields.fail() || (fields >> extra)) {
            *error = where + ": expected a time and " + std::to_string(kJointCount) + " positions";
            return false;
        }
        for (int p : s.pose) {
            if (p < 0 || p > 4095) {
                *error = where + ": position " + std::to_string(p) + " outside 0-4095";
                return false;
            }
        }
        if (!samples.empty() && s.t < samples.back().t) {
            *error = where + ": time goes backwards";
            return false;
        }
        samples.push_back(s);
    }

    if (samples.size() < 2 || samples.back().t <= samples.front().t) {
        *error = path + " has no motion in it";
        return false;
    }

    const double t0 = samples.front().t;
    for (Sample& s : samples) s.t -= t0;
    *out = Recording(std::move(samples));
    return true;
}

}  // namespace arm
