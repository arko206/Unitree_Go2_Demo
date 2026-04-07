#ifndef PLANNER_CONFIG_H
#define PLANNER_CONFIG_H
// planner_config.h  —  Shared types, geometry helpers, and config-file loader
//                       for the SE(2) A* planner.
//
// Include this from any translation unit that needs PlannerConfig.
// No external dependencies beyond the C++17 standard library.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// ============================================================
// SE(2) pose
// ============================================================
struct Pose2D {
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;
};

// ============================================================
// Rectangular obstacle
// ============================================================
struct Obstacle {
    Pose2D pose;
    double length = 0.355;   // along obstacle local-x
    double width  = 0.255;   // along obstacle local-y
};

// ============================================================
// Full planner configuration
// ============================================================
struct PlannerConfig {
    // Map bounds (floor X-Y plane, metres)
    double x_min = -1.0;
    double x_max =  1.5;
    double y_min = -1.0;
    double y_max =  2.75;

    // Grid resolution
    double dx     = 0.2;
    double dy     = 0.2;
    int    n_theta = 8;                         // heading bins
    double dtheta = 2.0 * M_PI / 8;            // derived

    // Goal tolerance (Euclidean, metres)
    double goal_tol = 0.35;

    // Robot modelled as a circle
    double robot_radius = 0.25;

    // Multiple rectangular obstacles (oriented in floor frame)
    std::vector<Obstacle> obstacles;
    double safety_margin = 0.02;                // extra inflation

    // Poses
    Pose2D start;
    Pose2D goal;

    // Post-planning offset  (offset = real_start − planned_start)
    double delta_x     = 0.0;
    double delta_y     = 0.0;
    double delta_theta = 0.0;

    // Output trajectory file
    std::string traj_file = "se2_path_floor.txt";

    // Recompute dtheta from n_theta (call after changing n_theta)
    void refresh_dtheta() { dtheta = 2.0 * M_PI / n_theta; }
};

// ============================================================
// Geometry helpers
// ============================================================
inline double wrap_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

inline double hypot2(double dx, double dy) {
    return std::sqrt(dx * dx + dy * dy);
}

// Point vs single inflated rotated rectangle
inline bool point_in_inflated_rect(double px, double py,
                                   const Obstacle& ob,
                                   double robot_radius,
                                   double safety_margin) {
    double total = robot_radius + safety_margin;
    double dx = px - ob.pose.x;
    double dy = py - ob.pose.y;
    double c  = std::cos(ob.pose.theta);
    double s  = std::sin(ob.pose.theta);
    double lx =  c * dx + s * dy;
    double ly = -s * dx + c * dy;
    return std::fabs(lx) <= (ob.length / 2.0 + total)
        && std::fabs(ly) <= (ob.width  / 2.0 + total);
}

// Point vs any obstacle in the config
inline bool point_hits_any_obstacle(double px, double py,
                                    const PlannerConfig& cfg) {
    for (const auto& ob : cfg.obstacles)
        if (point_in_inflated_rect(px, py, ob,
                                   cfg.robot_radius, cfg.safety_margin))
            return true;
    return false;
}

// Segment collision check (sample along the line)
inline bool segment_collides(double x0, double y0,
                             double x1, double y1,
                             const PlannerConfig& cfg,
                             double sample_step = 0.01) {
    double dist = hypot2(x1 - x0, y1 - y0);
    int n = std::max(1, static_cast<int>(std::ceil(dist / sample_step)));
    for (int i = 0; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        if (point_hits_any_obstacle(x0 + t * (x1 - x0),
                                    y0 + t * (y1 - y0), cfg))
            return true;
    }
    return false;
}

// ============================================================
// Config-file loader  (key = value,  # comments)
// ============================================================
inline bool load_config_file(const std::string& path, PlannerConfig& cfg) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Cannot open config: " << path << "\n";
        return false;
    }

    std::string line;
    while (std::getline(f, line)) {
        // Strip comment
        auto sharp = line.find('#');
        if (sharp != std::string::npos) line.erase(sharp);
        // Strip whitespace
        auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;          // blank

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;              // malformed

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // trim both
        auto trim = [](std::string& s) {
            auto a = s.find_first_not_of(" \t\r\n");
            auto b = s.find_last_not_of(" \t\r\n");
            s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
        };
        trim(key);
        trim(val);
        if (key.empty() || val.empty()) continue;

        // Match keys
        try {
            if      (key == "x_min")         cfg.x_min         = std::stod(val);
            else if (key == "x_max")         cfg.x_max         = std::stod(val);
            else if (key == "y_min")         cfg.y_min         = std::stod(val);
            else if (key == "y_max")         cfg.y_max         = std::stod(val);
            else if (key == "dx")            cfg.dx            = std::stod(val);
            else if (key == "dy")            cfg.dy            = std::stod(val);
            else if (key == "n_theta")     { cfg.n_theta       = std::stoi(val);
                                             cfg.refresh_dtheta(); }
            else if (key == "goal_tol")      cfg.goal_tol      = std::stod(val);
            else if (key == "robot_radius")  cfg.robot_radius  = std::stod(val);
            else if (key == "start_x")       cfg.start.x       = std::stod(val);
            else if (key == "start_y")       cfg.start.y       = std::stod(val);
            else if (key == "start_theta")   cfg.start.theta   = std::stod(val);
            else if (key == "goal_x")        cfg.goal.x        = std::stod(val);
            else if (key == "goal_y")        cfg.goal.y        = std::stod(val);
            else if (key == "goal_theta")    cfg.goal.theta     = std::stod(val);
            else if (key == "safety_margin") cfg.safety_margin = std::stod(val);
            else if (key == "delta_x")       cfg.delta_x       = std::stod(val);
            else if (key == "delta_y")       cfg.delta_y       = std::stod(val);
            else if (key == "delta_theta")   cfg.delta_theta   = std::stod(val);
            else if (key == "traj_file")     cfg.traj_file     = val;
            // obs.N.field  (e.g. obs.0.x, obs.1.length)
            else if (key.rfind("obs.", 0) == 0) {
                // parse  obs.<idx>.<field>
                auto dot1 = key.find('.', 4);
                if (dot1 != std::string::npos) {
                    int idx   = std::stoi(key.substr(4, dot1 - 4));
                    auto fld  = key.substr(dot1 + 1);
                    if (idx < 0) throw std::runtime_error("negative obstacle index");
                    auto uidx = static_cast<std::size_t>(idx);
                    if (uidx >= cfg.obstacles.size())
                        cfg.obstacles.resize(uidx + 1);
                    auto& ob = cfg.obstacles[uidx];
                    if      (fld == "x")      ob.pose.x     = std::stod(val);
                    else if (fld == "y")      ob.pose.y     = std::stod(val);
                    else if (fld == "theta")  ob.pose.theta = std::stod(val);
                    else if (fld == "length") ob.length     = std::stod(val);
                    else if (fld == "width")  ob.width      = std::stod(val);
                }
            }
            // unknown keys are silently ignored
        } catch (const std::exception& e) {
            std::cerr << "Config parse error for key '" << key
                      << "': " << e.what() << "\n";
        }
    }
    return true;
}

// ============================================================
// TF-JSON loader  (same structure as the Python TF logger output)
// ============================================================
namespace detail {
inline double json_number(const std::string& blob, const std::string& key) {
    std::string needle = "\"" + key + "\"";
    auto pos = blob.find(needle);
    if (pos == std::string::npos)
        throw std::runtime_error("JSON key not found: " + key);
    pos = blob.find(':', pos);
    if (pos == std::string::npos)
        throw std::runtime_error("Malformed JSON near key: " + key);
    ++pos;
    while (pos < blob.size() && (blob[pos] == ' ' || blob[pos] == '\t'))
        ++pos;
    if (pos < blob.size() && blob[pos] == '"') ++pos;
    return std::stod(blob.substr(pos));
}

inline Pose2D extract_se2(const std::string& blob,
                           const std::string& block_key) {
    auto bpos = blob.find("\"" + block_key + "\"");
    if (bpos == std::string::npos)
        throw std::runtime_error("Block not found: " + block_key);
    auto sub = blob.substr(bpos);
    auto se2pos = sub.find("\"se2_floor_xy\"");
    if (se2pos == std::string::npos)
        throw std::runtime_error("se2_floor_xy not found in " + block_key);
    auto chunk = sub.substr(se2pos, 300);
    Pose2D p;
    p.x     = json_number(chunk, "x");
    p.y     = json_number(chunk, "y");
    p.theta = json_number(chunk, "theta");
    return p;
}
} // namespace detail

inline bool load_tf_json(const std::string& path, PlannerConfig& cfg) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Cannot open TF JSON: " << path << "\n";
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    std::string blob = ss.str();
    try {
        cfg.start = detail::extract_se2(blob, "floor_to_base_link");
        cfg.goal  = detail::extract_se2(blob, "floor_to_object_3");
        // Load obstacle pose from JSON into the first obstacle slot
        Pose2D obs_pose = detail::extract_se2(blob, "floor_to_object_2");
        if (cfg.obstacles.empty())
            cfg.obstacles.push_back(Obstacle{});
        cfg.obstacles[0].pose = obs_pose;
    } catch (const std::exception& e) {
        std::cerr << "TF JSON parse error: " << e.what() << "\n";
        return false;
    }
    return true;
}

#endif // PLANNER_CONFIG_H
