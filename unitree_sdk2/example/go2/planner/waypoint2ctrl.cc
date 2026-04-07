// waypoint2ctrl.cc  —  Reads an SE(2) waypoint file (x y theta) and emits
//                       unicycle controller commands (vx, vtheta).
//
// The robot is a unicycle: vy is always 0.
//   vx     = forward displacement along current heading
//   vtheta = heading change (rotation in place)
//
// Usage:  ./waypoint2ctrl <waypoints.txt>  [-o controls.txt]
//
// Build:  g++ -std=c++17 -O2 -Wall -o waypoint2ctrl waypoint2ctrl.cc

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

// ────────────────────────────────────────────────────────────
struct Pose { double x, y, theta; };

static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// ────────────────────────────────────────────────────────────
// Load waypoints from the planner's output file
// Format:  header line, then "x y theta" per row
// ────────────────────────────────────────────────────────────
static std::vector<Pose> load_waypoints(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "ERROR: cannot open " << path << "\n";
        return {};
    }

    std::vector<Pose> pts;
    std::string line;

    // Skip header
    if (!std::getline(f, line)) return pts;

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        Pose p{};
        if (ss >> p.x >> p.y >> p.theta)
            pts.push_back(p);
    }
    return pts;
}

// ────────────────────────────────────────────────────────────
// Emit controls: one pair per segment
//   (vx, vtheta)
//   vx     = body-frame forward displacement (vy constrained to 0)
//   vtheta = heading change
// ────────────────────────────────────────────────────────────
static void emit_controls(const std::vector<Pose>& pts, std::ostream& os) {
    if (pts.size() < 2) return;

    for (std::size_t i = 1; i < pts.size(); ++i) {
        const auto& prev = pts[i - 1];
        const auto& next = pts[i];

        double dx     = next.x - prev.x;
        double dy     = next.y - prev.y;
        double dtheta = wrap(next.theta - prev.theta);

        // Body-frame projection: vx = displacement along heading
        double vx = dx * std::cos(prev.theta) + dy * std::sin(prev.theta);

        char buf[128];
        std::snprintf(buf, sizeof(buf), "%.6f, 0.000000, %.6f", vx, dtheta);
        os << buf << "\n";
    }
}

// ────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::printf("Usage: %s <waypoints.txt> [-o controls.txt]\n", argv[0]);
        return 1;
    }

    std::string wp_path = argv[1];
    std::string out_path;

    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc)
            out_path = argv[++i];
    }

    auto pts = load_waypoints(wp_path);
    if (pts.empty()) {
        std::cerr << "No waypoints loaded.\n";
        return 1;
    }
    std::printf("Loaded %zu waypoints from %s\n\n", pts.size(), wp_path.c_str());

    // Always print to stdout
    emit_controls(pts, std::cout);

    // Optionally write to file
    if (!out_path.empty()) {
        std::ofstream of(out_path);
        if (!of) {
            std::cerr << "ERROR: cannot open " << out_path << "\n";
            return 1;
        }
        emit_controls(pts, of);
        std::printf("\nControls written to %s\n", out_path.c_str());
    }

    return 0;
}
