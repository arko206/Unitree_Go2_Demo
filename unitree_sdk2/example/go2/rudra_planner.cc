// planner.cc — SE(2) A* planner + controller
//
// Two-phase CLI:
//   ./astar --plan    → runs A*, writes se2_waypoints.txt
//   ./astar --gen_control → reads se2_waypoints.txt, writes controls.txt
//
// Config split:
//   planner.cfg  — static settings (map, grid, robot, tolerances, obstacles)
//   query.cfg    — dynamic settings (start, goal) — updated per run
//
// Build:  make

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// ============================================================
// SE(2) pose
// ============================================================
struct Pose2D
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// ============================================================
// Rectangular obstacle
// ============================================================
struct Obstacle
{
    Pose2D pose;
    double length = 0.35;
    double width = 0.25;
};

// ============================================================
// Full planner configuration
// ============================================================
struct PlannerConfig
{
    // Map bounds (metres)
    double x_min = -1.5;
    double x_max = 1.5;
    double y_min = -1.5;
    double y_max = 2.75;

    // Grid resolution
    double dx = 0.2;
    double dy = 0.2;
    int n_theta = 8;
    double dtheta = 2.0 * M_PI / 8;

    // Goal tolerance (Euclidean, metres)
    double goal_tol = 0.35;

    // Robot circle radius
    double robot_radius = 0.5;

    // Obstacles
    std::vector<Obstacle> obstacles;
    double safety_margin = 0.15;

    // Poses
    Pose2D start;
    Pose2D goal;

    // Controller: time per step (seconds) — controls = displacement / step_time
    double step_time = 2.0;

    void refresh_dtheta() { dtheta = 2.0 * M_PI / n_theta; }
};

// ============================================================
// Geometry helpers
// ============================================================
static inline double wrap_angle(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

static inline double hypot2(double dx, double dy)
{
    return std::sqrt(dx * dx + dy * dy);
}

static inline bool point_in_inflated_rect(double px, double py,
                                          const Obstacle &ob,
                                          double robot_radius,
                                          double safety_margin)
{
    double total = robot_radius + safety_margin;
    double dx = px - ob.pose.x;
    double dy = py - ob.pose.y;
    double c = std::cos(ob.pose.theta);
    double s = std::sin(ob.pose.theta);
    double lx = c * dx + s * dy;
    double ly = -s * dx + c * dy;
    return std::fabs(lx) <= (ob.length / 2.0 + total) && std::fabs(ly) <= (ob.width / 2.0 + total);
}

static inline bool point_hits_any_obstacle(double px, double py,
                                           const PlannerConfig &cfg)
{
    for (const auto &ob : cfg.obstacles)
        if (point_in_inflated_rect(px, py, ob, cfg.robot_radius, cfg.safety_margin))
            return true;
    return false;
}

// ============================================================
// Config-file loader (key = value, # comments)
// ============================================================
static bool load_config_file(const std::string &path, PlannerConfig &cfg)
{
    std::ifstream f(path);
    if (!f)
    {
        std::cerr << "Cannot open config: " << path << "\n";
        return false;
    }

    auto trim = [](std::string &s)
    {
        auto a = s.find_first_not_of(" \t\r\n");
        auto b = s.find_last_not_of(" \t\r\n");
        s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    };

    std::string line;
    while (std::getline(f, line))
    {
        auto sharp = line.find('#');
        if (sharp != std::string::npos)
            line.erase(sharp);
        auto first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
            continue;

        auto eq = line.find('=');
        if (eq == std::string::npos)
            continue;

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        trim(key);
        trim(val);
        if (key.empty() || val.empty())
            continue;

        try
        {
            if (key == "x_min")
                cfg.x_min = std::stod(val);
            else if (key == "x_max")
                cfg.x_max = std::stod(val);
            else if (key == "y_min")
                cfg.y_min = std::stod(val);
            else if (key == "y_max")
                cfg.y_max = std::stod(val);
            else if (key == "dx")
                cfg.dx = std::stod(val);
            else if (key == "dy")
                cfg.dy = std::stod(val);
            else if (key == "n_theta")
            {
                cfg.n_theta = std::stoi(val);
                cfg.refresh_dtheta();
            }
            else if (key == "goal_tol")
                cfg.goal_tol = std::stod(val);
            else if (key == "robot_radius")
                cfg.robot_radius = std::stod(val);
            else if (key == "start_x")
                cfg.start.x = std::stod(val);
            else if (key == "start_y")
                cfg.start.y = std::stod(val);
            else if (key == "start_theta")
                cfg.start.theta = std::stod(val);
            else if (key == "goal_x")
                cfg.goal.x = std::stod(val);
            else if (key == "goal_y")
                cfg.goal.y = std::stod(val);
            else if (key == "goal_theta")
                cfg.goal.theta = std::stod(val);
            else if (key == "safety_margin")
                cfg.safety_margin = std::stod(val);
            else if (key == "step_time")
                cfg.step_time = std::stod(val);
            else if (key.rfind("obs.", 0) == 0)
            {
                auto dot1 = key.find('.', 4);
                if (dot1 != std::string::npos)
                {
                    int idx = std::stoi(key.substr(4, dot1 - 4));
                    auto fld = key.substr(dot1 + 1);
                    if (idx < 0)
                        throw std::runtime_error("negative obstacle index");
                    auto uidx = static_cast<std::size_t>(idx);
                    if (uidx >= cfg.obstacles.size())
                        cfg.obstacles.resize(uidx + 1);
                    auto &ob = cfg.obstacles[uidx];
                    if (fld == "x")
                        ob.pose.x = std::stod(val);
                    else if (fld == "y")
                        ob.pose.y = std::stod(val);
                    else if (fld == "theta")
                        ob.pose.theta = std::stod(val);
                    else if (fld == "length")
                        ob.length = std::stod(val);
                    else if (fld == "width")
                        ob.width = std::stod(val);
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Config parse error for key '" << key
                      << "': " << e.what() << "\n";
        }
    }
    return true;
}

// ============================================================
// Discrete grid
// ============================================================
struct Grid
{
    double x_min, y_min;
    double dx, dy;
    int NX, NY, NT;
    double dtheta;

    static constexpr int DIR[8][2] = {
        {1, 0},
        {1, 1},
        {0, 1},
        {-1, 1},
        {-1, 0},
        {-1, -1},
        {0, -1},
        {1, -1},
    };
    static constexpr double DIR_COST[8] = {
        1.0,
        1.41421356,
        1.0,
        1.41421356,
        1.0,
        1.41421356,
        1.0,
        1.41421356,
    };

    void init(const PlannerConfig &cfg)
    {
        x_min = cfg.x_min;
        y_min = cfg.y_min;
        dx = cfg.dx;
        dy = cfg.dy;
        NT = cfg.n_theta;
        dtheta = 2.0 * M_PI / NT;
        NX = static_cast<int>(std::round((cfg.x_max - cfg.x_min) / dx)) + 1;
        NY = static_cast<int>(std::round((cfg.y_max - cfg.y_min) / dy)) + 1;
    }

    double wx(int xi) const { return x_min + xi * dx; }
    double wy(int yi) const { return y_min + yi * dy; }
    double wt(int ti) const { return ti * dtheta; }

    int to_xi(double x) const { return static_cast<int>(std::floor((x - x_min) / dx + 0.5 + 1e-9)); }
    int to_yi(double y) const { return static_cast<int>(std::floor((y - y_min) / dy + 0.5 + 1e-9)); }
    int to_ti(double theta) const
    {
        double t = wrap_angle(theta);
        if (t < 0.0)
            t += 2.0 * M_PI;
        int ti = static_cast<int>(std::round(t / dtheta)) % NT;
        return ti;
    }

    bool in_bounds(int xi, int yi) const
    {
        return xi >= 0 && xi < NX && yi >= 0 && yi < NY;
    }
};

// ============================================================
// Collision checking
// ============================================================
static bool is_free(double px, double py, const PlannerConfig &cfg)
{
    double r = cfg.robot_radius;
    if (px - r < cfg.x_min || px + r > cfg.x_max ||
        py - r < cfg.y_min || py + r > cfg.y_max)
        return false;
    return !point_hits_any_obstacle(px, py, cfg);
}

static bool segment_free(double x0, double y0,
                         double x1, double y1,
                         const PlannerConfig &cfg)
{
    double d = hypot2(x1 - x0, y1 - y0);
    int n = std::max(2, static_cast<int>(std::ceil(d / 0.01)));
    for (int i = 0; i <= n; ++i)
    {
        double t = static_cast<double>(i) / n;
        double px = x0 + t * (x1 - x0);
        double py = y0 + t * (y1 - y0);
        if (!is_free(px, py, cfg))
            return false;
    }
    return true;
}

static double clearance_to_obstacle(double px, double py,
                                    const Obstacle &ob,
                                    double robot_radius,
                                    double safety_margin)
{
    double total = robot_radius + safety_margin;
    double ddx = px - ob.pose.x;
    double ddy = py - ob.pose.y;
    double c = std::cos(ob.pose.theta);
    double s = std::sin(ob.pose.theta);
    double lx = c * ddx + s * ddy;
    double ly = -s * ddx + c * ddy;
    double cx = std::fabs(lx) - (ob.length / 2.0 + total);
    double cy = std::fabs(ly) - (ob.width / 2.0 + total);
    return std::max(cx, cy);
}

// ============================================================
// A* types
// ============================================================
struct State
{
    int xi = 0, yi = 0, ti = 0;
    bool operator==(const State &o) const
    {
        return xi == o.xi && yi == o.yi && ti == o.ti;
    }
};

struct StateHash
{
    std::size_t operator()(const State &s) const
    {
        std::size_t h = std::hash<int>()(s.xi);
        h ^= std::hash<int>()(s.yi) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(s.ti) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

struct Node
{
    double f = 0, g = 0;
    State state;
    int parent = -1;
    bool operator>(const Node &o) const { return f > o.f; }
};

struct PathPoint
{
    double x, y, theta;
};

// ============================================================
// A* search
// ============================================================
static bool run_astar(const PlannerConfig &cfg,
                      std::vector<PathPoint> &out_path)
{
    Grid grid;
    grid.init(cfg);

    std::printf("  Grid dims  : NX=%d  NY=%d  NT=%d\n", grid.NX, grid.NY, grid.NT);

    State start_s{grid.to_xi(cfg.start.x), grid.to_yi(cfg.start.y),
                  grid.to_ti(cfg.start.theta)};
    State goal_s{grid.to_xi(cfg.goal.x), grid.to_yi(cfg.goal.y),
                 grid.to_ti(cfg.goal.theta)};

    double sx = grid.wx(start_s.xi), sy = grid.wy(start_s.yi);
    double gx = cfg.goal.x, gy = cfg.goal.y;  // true query goal, not grid-snapped

    std::printf("  Grid start : (%d, %d, %d) -> (%.3f, %.3f, %.3f)\n",
                start_s.xi, start_s.yi, start_s.ti, sx, sy, grid.wt(start_s.ti));
    std::printf("  Grid goal  : (%d, %d, %d) -> (%.3f, %.3f, %.3f)  true=(%.3f, %.3f)\n",
                goal_s.xi, goal_s.yi, goal_s.ti,
                grid.wx(goal_s.xi), grid.wy(goal_s.yi), grid.wt(goal_s.ti),
                gx, gy);

    if (!grid.in_bounds(start_s.xi, start_s.yi))
    {
        std::cerr << "ERROR: start is outside grid.\n";
        return false;
    }
    if (!is_free(sx, sy, cfg))
    {
        std::cerr << "ERROR: start (" << sx << ", " << sy
                  << ") is in collision or outside map.\n";
        for (std::size_t i = 0; i < cfg.obstacles.size(); ++i)
        {
            double cl = clearance_to_obstacle(sx, sy, cfg.obstacles[i],
                                              cfg.robot_radius, cfg.safety_margin);
            std::cerr << "  obs[" << i << "] clearance = " << cl << " m\n";
        }
        return false;
    }
    if (!grid.in_bounds(goal_s.xi, goal_s.yi))
    {
        std::cerr << "ERROR: goal is outside grid.\n";
        return false;
    }
    double gx_snap = grid.wx(goal_s.xi), gy_snap = grid.wy(goal_s.yi);
    if (!is_free(gx_snap, gy_snap, cfg))
    {
        std::cerr << "ERROR: goal snap (" << gx_snap << ", " << gy_snap
                  << ") is in collision or outside map.\n";
        for (std::size_t i = 0; i < cfg.obstacles.size(); ++i)
        {
            double cl = clearance_to_obstacle(gx_snap, gy_snap, cfg.obstacles[i],
                                              cfg.robot_radius, cfg.safety_margin);
            std::cerr << "  obs[" << i << "] clearance = " << cl << " m\n";
        }
        return false;
    }

    std::vector<Node> closed;
    closed.reserve(100000);

    using PQ = std::priority_queue<Node, std::vector<Node>, std::greater<Node> >;
    PQ open;
    std::unordered_map<State, double, StateHash> best;

    double h0 = hypot2(sx - gx, sy - gy);
    Node sn{};
    sn.f = h0;
    sn.g = 0.0;
    sn.state = start_s;
    sn.parent = -1;

    open.push(sn);
    best[start_s] = 0.0;

    double goal_tol_sq = cfg.goal_tol * cfg.goal_tol;
    int expanded = 0, generated = 0;

    while (!open.empty())
    {
        Node cur = open.top();
        open.pop();

        auto it = best.find(cur.state);
        if (it != best.end() && cur.g > it->second + 1e-9)
            continue;

        int ci = static_cast<int>(closed.size());
        closed.push_back(cur);
        ++expanded;

        double cur_wx = grid.wx(cur.state.xi);
        double cur_wy = grid.wy(cur.state.yi);

        double ddx = cur_wx - gx;
        double ddy = cur_wy - gy;
        if (ddx * ddx + ddy * ddy <= goal_tol_sq)
        {
            std::vector<PathPoint> path;
            int pidx = ci;
            while (pidx >= 0)
            {
                const Node &n = closed[static_cast<std::size_t>(pidx)];
                path.push_back({grid.wx(n.state.xi),
                                grid.wy(n.state.yi),
                                grid.wt(n.state.ti)});
                pidx = n.parent;
            }
            std::reverse(path.begin(), path.end());
            out_path = std::move(path);
            std::printf("Path found: %zu waypoints (expanded=%d, generated=%d)\n",
                        out_path.size(), expanded, generated);
            return true;
        }

        // ── In-place rotation: stay at (xi,yi), change heading ±1 ──
        double rot_cost = 0.5 * std::min(cfg.dx, cfg.dy);
        int rot_headings[2] = {
            (cur.state.ti + 1) % grid.NT,
            ((cur.state.ti - 1) + grid.NT) % grid.NT,
        };
        for (int r = 0; r < 2; ++r)
        {
            State rs{cur.state.xi, cur.state.yi, rot_headings[r]};
            double rg = cur.g + rot_cost;

            auto rit = best.find(rs);
            if (rit != best.end() && rit->second <= rg)
                continue;
            best[rs] = rg;

            double rh = hypot2(cur_wx - gx, cur_wy - gy);
            Node rn{};
            rn.f = rg + rh;
            rn.g = rg;
            rn.state = rs;
            rn.parent = ci;
            open.push(rn);
            ++generated;
        }

        // ── Forward move along current heading ──
        int di = Grid::DIR[cur.state.ti][0];
        int dj = Grid::DIR[cur.state.ti][1];
        double step_cost = Grid::DIR_COST[cur.state.ti] * std::min(cfg.dx, cfg.dy);

        int nxi = cur.state.xi + di;
        int nyi = cur.state.yi + dj;

        if (!grid.in_bounds(nxi, nyi))
            continue;

        double nwx = grid.wx(nxi);
        double nwy = grid.wy(nyi);

        if (!is_free(nwx, nwy, cfg))
            continue;
        if (!segment_free(cur_wx, cur_wy, nwx, nwy, cfg))
            continue;

        int headings[3] = {
            cur.state.ti,
            (cur.state.ti + 1) % grid.NT,
            ((cur.state.ti - 1) + grid.NT) % grid.NT,
        };
        double costs[3] = {step_cost, step_cost * 1.2, step_cost * 1.2};

        for (int p = 0; p < 3; ++p)
        {
            State ns{nxi, nyi, headings[p]};
            double ng = cur.g + costs[p];

            auto bit = best.find(ns);
            if (bit != best.end() && bit->second <= ng)
                continue;
            best[ns] = ng;

            double nh = hypot2(nwx - gx, nwy - gy);
            Node nb{};
            nb.f = ng + nh;
            nb.g = ng;
            nb.state = ns;
            nb.parent = ci;
            open.push(nb);
            ++generated;
        }
    }

    std::printf("No path found. (expanded=%d, generated=%d)\n", expanded, generated);
    return false;
}

// ============================================================
// Controller: waypoints -> velocity commands
// ============================================================
struct ControlCmd
{
    double vx;       // forward velocity (m/s)
    double vy;       // lateral velocity (m/s) — always 0
    double vtheta;   // angular velocity (rad/s)
    double duration; // how long to hold this command (s)
};

static std::vector<ControlCmd> compute_controls(
    const std::vector<PathPoint> &path,
    double step_time,
    double goal_theta)
{
    std::vector<ControlCmd> cmds;
    if (path.size() < 2)
        return cmds;

    // Track the current heading as we emit commands
    double cur_theta = path[0].theta;

    auto push_rot = [&](double angle) {
        ControlCmd r;
        r.vx = 0.0;  r.vy = 0.0;
        r.vtheta = angle / step_time;
        r.duration = step_time;
        cmds.push_back(r);
        cur_theta += angle;
    };
    auto push_fwd = [&](double displacement) {
        ControlCmd t;
        t.vx = displacement / step_time;  t.vy = 0.0;
        t.vtheta = 0.0;
        t.duration = step_time;
        cmds.push_back(t);
    };

    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const auto &prev = path[i - 1];
        const auto &next = path[i];

        double dx_w = next.x - prev.x;
        double dy_w = next.y - prev.y;
        double dist = hypot2(dx_w, dy_w);

        if (dist > 1e-9)
        {
            // 1) Rotate to face direction of travel
            double travel_theta = std::atan2(dy_w, dx_w);
            double rot_to_travel = wrap_angle(travel_theta - cur_theta);
            if (std::fabs(rot_to_travel) > 1e-9)
                push_rot(rot_to_travel);

            // 2) Translate forward
            push_fwd(dist);

            // 3) Rotate to destination heading (if different from travel dir)
            double rot_to_dest = wrap_angle(next.theta - cur_theta);
            if (std::fabs(rot_to_dest) > 1e-9)
                push_rot(rot_to_dest);
        }
        else
        {
            // Pure in-place rotation
            double dtheta = wrap_angle(next.theta - cur_theta);
            if (std::fabs(dtheta) > 1e-9)
                push_rot(dtheta);
        }
    }

    // Final heading correction to match goal_theta
    double heading_err = wrap_angle(goal_theta - cur_theta);
    if (std::fabs(heading_err) > 1e-6)
        push_rot(heading_err);

    return cmds;
}

// ============================================================
// Write SE2 waypoints to file (x, y, theta per line)
// ============================================================
static bool write_waypoints_file(const std::vector<PathPoint> &path,
                                 const std::string &filepath)
{
    std::ofstream f(filepath);
    if (!f)
    {
        std::cerr << "ERROR: cannot open " << filepath << " for writing\n";
        return false;
    }
    for (const auto &p : path)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%f, %f, %f", p.x, p.y, p.theta);
        f << buf << "\n";
    }
    std::printf("Wrote %zu waypoints to %s\n", path.size(), filepath.c_str());
    return true;
}

// ============================================================
// Read SE2 waypoints from file
// ============================================================
static bool read_waypoints_file(const std::string &filepath,
                                std::vector<PathPoint> &path)
{
    std::ifstream f(filepath);
    if (!f)
    {
        std::cerr << "ERROR: cannot open " << filepath << " for reading\n";
        return false;
    }
    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty())
            continue;
        // Replace commas with spaces for uniform parsing
        for (auto &ch : line)
            if (ch == ',')
                ch = ' ';
        std::istringstream iss(line);
        PathPoint p;
        if (iss >> p.x >> p.y >> p.theta)
            path.push_back(p);
    }
    std::printf("Read %zu waypoints from %s\n", path.size(), filepath.c_str());
    return !path.empty();
}

// ============================================================
// Write controls to file (vx, vy, vtheta per line)
// ============================================================
static bool write_controls_file(const std::vector<ControlCmd> &cmds,
                                const std::string &path)
{
    std::ofstream f(path);
    if (!f)
    {
        std::cerr << "ERROR: cannot open " << path << " for writing\n";
        return false;
    }
    for (const auto &c : cmds)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%f, %f, %f", c.vx, c.vy, c.vtheta);
        f << buf << "\n";
    }
    std::printf("Wrote %zu controls to %s\n", cmds.size(), path.c_str());
    return true;
}

// ============================================================
// Safety audit
// ============================================================
static void audit_path(const std::vector<PathPoint> &path,
                       const PlannerConfig &cfg)
{
    std::printf("\nPath safety audit:\n");
    std::printf("  %-4s  %-8s %-8s  ", "WP", "x", "y");
    for (std::size_t j = 0; j < cfg.obstacles.size(); ++j)
        std::printf("obs[%zu] clr  ", j);
    std::printf("map_clr\n");

    bool any_violation = false;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        double px = path[i].x, py = path[i].y;
        std::printf("  %-4zu  %7.3f  %7.3f  ", i, px, py);
        for (std::size_t j = 0; j < cfg.obstacles.size(); ++j)
        {
            double cl = clearance_to_obstacle(px, py, cfg.obstacles[j],
                                              cfg.robot_radius, cfg.safety_margin);
            std::printf("  %+.4f    ", cl);
            if (cl < -1e-6)
                any_violation = true;
        }
        double map_cl = std::min({px - cfg.robot_radius - cfg.x_min,
                                  cfg.x_max - px - cfg.robot_radius,
                                  py - cfg.robot_radius - cfg.y_min,
                                  cfg.y_max - py - cfg.robot_radius});
        std::printf("  %+.4f", map_cl);
        if (map_cl < -1e-6)
            any_violation = true;
        std::printf("\n");
    }
    if (any_violation)
        std::printf("  *** WARNING: path has safety violations! ***\n");
    else
        std::printf("  All waypoints have positive clearance.\n");
}

// ============================================================
// Print configuration summary
// ============================================================
static void print_config(const PlannerConfig &cfg)
{
    std::printf("\n=== SE(2) A* Planner ===\n");
    std::printf("  Start      : (%.3f, %.3f, %.3f)\n",
                cfg.start.x, cfg.start.y, cfg.start.theta);
    std::printf("  Goal       : (%.3f, %.3f, %.3f)\n",
                cfg.goal.x, cfg.goal.y, cfg.goal.theta);
    std::printf("  Robot R    : %.3f m\n", cfg.robot_radius);
    std::printf("  Safety     : %.3f m\n", cfg.safety_margin);
    std::printf("  Inflation  : %.3f m (robot_radius + safety_margin)\n",
                cfg.robot_radius + cfg.safety_margin);
    std::printf("  Step time  : %.1f s\n", cfg.step_time);
    std::printf("  Obstacles  : %zu\n", cfg.obstacles.size());
    for (std::size_t i = 0; i < cfg.obstacles.size(); ++i)
    {
        const auto &ob = cfg.obstacles[i];
        std::printf("    [%zu] centre=(%.3f, %.3f) th=%.3f  L=%.3f W=%.3f\n",
                    i, ob.pose.x, ob.pose.y, ob.pose.theta,
                    ob.length, ob.width);
    }
    std::printf("  Map        : X[%.1f, %.1f]  Y[%.1f, %.1f]\n",
                cfg.x_min, cfg.x_max, cfg.y_min, cfg.y_max);
    std::printf("  Grid       : dx=%.2f  dy=%.2f  n_theta=%d\n\n",
                cfg.dx, cfg.dy, cfg.n_theta);
}

// ============================================================
// Load both config files
// ============================================================
static bool load_configs(const std::string &static_cfg,
                         const std::string &dynamic_cfg,
                         PlannerConfig &cfg)
{
    std::printf("Loading static config:  %s\n", static_cfg.c_str());
    if (!load_config_file(static_cfg, cfg))
    {
        std::cerr << "ERROR: cannot load static config.\n";
        return false;
    }
    std::printf("Loading dynamic config: %s\n", dynamic_cfg.c_str());
    if (!load_config_file(dynamic_cfg, cfg))
    {
        std::cerr << "ERROR: cannot load dynamic config.\n";
        return false;
    }
    cfg.start.theta = wrap_angle(cfg.start.theta);
    cfg.goal.theta = wrap_angle(cfg.goal.theta);
    return true;
}

// ============================================================
// main
// ============================================================
int main(int argc, char *argv[])
{
    PlannerConfig cfg;

    std::string static_cfg = "planner.cfg";
    std::string dynamic_cfg = "query.cfg";
    std::string waypoints_file = "se2_waypoints.txt";
    std::string controls_out = "controls.txt";

    enum Mode { MODE_PLAN, MODE_GEN_CONTROL, MODE_BOTH };
    Mode mode = MODE_BOTH;

    // Parse CLI
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc)
            static_cfg = argv[++i];
        else if (std::strcmp(argv[i], "--query") == 0 && i + 1 < argc)
            dynamic_cfg = argv[++i];
        else if (std::strcmp(argv[i], "--waypoints") == 0 && i + 1 < argc)
            waypoints_file = argv[++i];
        else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc)
            controls_out = argv[++i];
        else if (std::strcmp(argv[i], "--plan") == 0)
            mode = MODE_PLAN;
        else if (std::strcmp(argv[i], "--gen_control") == 0)
            mode = MODE_GEN_CONTROL;
        else if (std::strcmp(argv[i], "-h") == 0 ||
                 std::strcmp(argv[i], "--help") == 0)
        {
            std::printf(
                "Usage: %s [options]\n"
                "\n"
                "Modes (pick one, or omit for both):\n"
                "  --plan            Run A*, write se2_waypoints.txt\n"
                "  --gen_control     Read se2_waypoints.txt, write controls.txt\n"
                "\n"
                "Options:\n"
                "  --config <path>     Static config   (default: planner.cfg)\n"
                "  --query  <path>     Dynamic config   (default: query.cfg)\n"
                "  --waypoints <path>  Waypoints file   (default: se2_waypoints.txt)\n"
                "  --output <path>     Controls output  (default: controls.txt)\n"
                "  -h, --help          This message\n",
                argv[0]);
            return 0;
        }
    }

    if (!load_configs(static_cfg, dynamic_cfg, cfg))
        return 1;

    print_config(cfg);

    if (mode == MODE_PLAN || mode == MODE_BOTH)
    {
        // ── Plan ──
        PathPoint zeros{};
        std::vector<PathPoint> path = {zeros};

        run_astar(cfg, path);
        audit_path(path, cfg);

        if (!write_waypoints_file(path, waypoints_file))
            return 1;

        if (mode == MODE_PLAN)
            return 0;

        // Fall through to gen_control in MODE_BOTH
        auto cmds = compute_controls(path, cfg.step_time, cfg.goal.theta);

        std::printf("\n=== Controls (%zu commands, step_time=%.1fs) ===\n",
                    cmds.size(), cfg.step_time);
        std::printf("  %-4s  %10s  %10s  %10s  %8s\n",
                    "Seg", "vx(m/s)", "vy(m/s)", "vth(rad/s)", "dur(s)");
        for (std::size_t i = 0; i < cmds.size(); ++i)
        {
            std::printf("  %-4zu  %10.4f  %10.4f  %10.4f  %8.1f\n",
                        i, cmds[i].vx, cmds[i].vy, cmds[i].vtheta, cmds[i].duration);
        }

        if (!write_controls_file(cmds, controls_out))
            return 1;
    }
    else if (mode == MODE_GEN_CONTROL)
    {
        // ── Gen control from existing waypoints ──
        std::vector<PathPoint> path;
        if (!read_waypoints_file(waypoints_file, path))
        {
            std::cerr << "ERROR: no waypoints loaded from " << waypoints_file << "\n";
            return 1;
        }

        auto cmds = compute_controls(path, cfg.step_time, cfg.goal.theta);

        std::printf("\n=== Controls (%zu commands, step_time=%.1fs) ===\n",
                    cmds.size(), cfg.step_time);
        std::printf("  %-4s  %10s  %10s  %10s  %8s\n",
                    "Seg", "vx(m/s)", "vy(m/s)", "vth(rad/s)", "dur(s)");
        for (std::size_t i = 0; i < cmds.size(); ++i)
        {
            std::printf("  %-4zu  %10.4f  %10.4f  %10.4f  %8.1f\n",
                        i, cmds[i].vx, cmds[i].vy, cmds[i].vtheta, cmds[i].duration);
        }

        if (!write_controls_file(cmds, controls_out))
            return 1;
    }

    return 0;
}
