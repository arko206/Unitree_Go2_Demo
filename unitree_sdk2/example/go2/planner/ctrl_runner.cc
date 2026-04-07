// ctrl_runner.cc — Standalone script that reads controls.txt (vx, vy, vtheta)
//                   and calls SetMotion() for each line, sleeping between steps.
//
// Build:  g++ -std=c++17 -O2 -Wall -o ctrl_runner ctrl_runner.cc
// Usage:  ./ctrl_runner [controls.txt]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#define TIME_SLEEP 5  // seconds to wait between SetMotion calls

// ── Stub: replace with actual robot API call ───────────────
static void SetMotion(double vx, double vy, double vtheta) {
    std::printf("SetMotion(vx=%.6f, vy=%.6f, vtheta=%.6f)\n", vx, vy, vtheta);
}

int main(int argc, char* argv[]) {
    const char* path = (argc > 1) ? argv[1] : "controls.txt";

    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 1;
    }

    std::string line;
    int step = 0;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        // Replace commas with spaces for uniform parsing
        for (char& c : line)
            if (c == ',') c = ' ';

        std::istringstream ss(line);
        double vx, vy, vtheta;
        if (!(ss >> vx >> vy >> vtheta)) {
            std::fprintf(stderr, "WARN: skipping malformed line: %s\n", line.c_str());
            continue;
        }

        std::printf("[step %2d] ", step++);
        SetMotion(vx, vy, vtheta);

        std::this_thread::sleep_for(std::chrono::seconds(TIME_SLEEP));
    }

    // Stop the robot
    std::printf("[done]   ");
    SetMotion(0.0, 0.0, 0.0);

    return 0;
}
