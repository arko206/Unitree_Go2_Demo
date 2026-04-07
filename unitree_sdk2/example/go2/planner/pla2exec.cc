// pla2exec.cc  —  SE(2) A* planner on a discrete (x, y, θ) grid
//
// COMPLETE REWRITE — pure grid-based A*.
//
// State space (all integer indices):
//   xi ∈ [0 .. NX-1]   →   world x = x_min + xi * dx
//   yi ∈ [0 .. NY-1]   →   world y = y_min + yi * dy
//   ti ∈ [0 .. n_theta-1] → world θ = ti * 2π/n_theta
//
// Motion model:
//   8 heading bins → 8 direction vectors (di, dj):
//     θ=0°→(+1,0), 45°→(+1,+1), 90°→(0,+1), 135°→(-1,+1),
//     180°→(-1,0), 225°→(-1,-1), 270°→(0,-1), 315°→(+1,-1)
//   3 primitives per state:
//     forward:       step along current heading, keep heading
//     forward-left:  step along current heading, heading ← (ti+1) mod n
//     forward-right: step along current heading, heading ← (ti-1) mod n
//
// Collision:
//   A grid node (wx, wy) is FREE iff:
//     1. Robot circle fits inside map  (wx ± R within [x_min, x_max], same y)
//     2. Robot centre is outside every inflated obstacle AABB
//   A transition is free iff every sampled point along the segment is free.
//
// Config loaded from planner.cfg via planner_config.h.
//
// Build:  g++ -std=c++17 -O2 -Wall -o pla2exec pla2exec.cc
// Run:    ./pla2exec [--config planner.cfg] [--json tf.json]

#include "planner_config.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <queue>
#include <unordered_map>
#include <vector>

// ============================================================
// Discrete grid
// ============================================================
struct Grid {
    double x_min, y_min;
    double dx, dy;
    int NX, NY, NT;
    double dtheta;

    // Direction table: heading bin → (di, dj) grid offset
    //   bin 0 = 0° = east, bin 1 = 45° = NE, ...
    static constexpr int DIR[8][2] = {
        { 1,  0}, { 1,  1}, { 0,  1}, {-1,  1},
        {-1,  0}, {-1, -1}, { 0, -1}, { 1, -1},
    };
    // Cost: 1.0 for cardinal, sqrt(2) for diagonal
    static constexpr double DIR_COST[8] = {
        1.0, 1.41421356, 1.0, 1.41421356,
        1.0, 1.41421356, 1.0, 1.41421356,
    };

    void init(const PlannerConfig& cfg) {
        x_min  = cfg.x_min;
        y_min  = cfg.y_min;
        dx     = cfg.dx;
        dy     = cfg.dy;
        NT     = cfg.n_theta;
        dtheta = 2.0 * M_PI / NT;
        NX     = static_cast<int>(std::round((cfg.x_max - cfg.x_min) / dx)) + 1;
        NY     = static_cast<int>(std::round((cfg.y_max - cfg.y_min) / dy)) + 1;
    }

    // Grid index → world coordinate
    double wx(int xi) const { return x_min + xi * dx; }
    double wy(int yi) const { return y_min + yi * dy; }
    double wt(int ti) const { return ti * dtheta; }

    // World coordinate → nearest grid index
    int to_xi(double x) const { return static_cast<int>(std::round((x - x_min) / dx)); }
    int to_yi(double y) const { return static_cast<int>(std::round((y - y_min) / dy)); }
    int to_ti(double theta) const {
        double t = wrap_angle(theta);
        if (t < 0.0) t += 2.0 * M_PI;
        int ti = static_cast<int>(std::round(t / dtheta)) % NT;
        return ti;
    }

    bool in_bounds(int xi, int yi) const {
        return xi >= 0 && xi < NX && yi >= 0 && yi < NY;
    }
};

// ============================================================
// Collision checking
// ============================================================

// Is a world point free? (robot circle must fit in map + no obstacle hit)
static bool is_free(double px, double py, const PlannerConfig& cfg) {
    double r = cfg.robot_radius;
    if (px - r < cfg.x_min || px + r > cfg.x_max ||
        py - r < cfg.y_min || py + r > cfg.y_max)
        return false;
    return !point_hits_any_obstacle(px, py, cfg);
}

// Is the straight-line segment from (x0,y0) to (x1,y1) free?
static bool segment_free(double x0, double y0,
                          double x1, double y1,
                          const PlannerConfig& cfg) {
    double d = hypot2(x1 - x0, y1 - y0);
    int n = std::max(2, static_cast<int>(std::ceil(d / 0.01)));
    for (int i = 0; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        double px = x0 + t * (x1 - x0);
        double py = y0 + t * (y1 - y0);
        if (!is_free(px, py, cfg))
            return false;
    }
    return true;
}

// Minimum clearance from a point to an obstacle (negative = inside)
static double clearance_to_obstacle(double px, double py,
                                     const Obstacle& ob,
                                     double robot_radius,
                                     double safety_margin) {
    double total = robot_radius + safety_margin;
    double ddx = px - ob.pose.x;
    double ddy = py - ob.pose.y;
    double c  = std::cos(ob.pose.theta);
    double s  = std::sin(ob.pose.theta);
    double lx =  c * ddx + s * ddy;
    double ly = -s * ddx + c * ddy;
    double cx = std::fabs(lx) - (ob.length / 2.0 + total);
    double cy = std::fabs(ly) - (ob.width  / 2.0 + total);
    return std::max(cx, cy);  // positive = outside, negative = inside
}

// ============================================================
// A* state  (packed integer triple)
// ============================================================
struct State {
    int xi = 0, yi = 0, ti = 0;
    bool operator==(const State& o) const {
        return xi == o.xi && yi == o.yi && ti == o.ti;
    }
};

struct StateHash {
    std::size_t operator()(const State& s) const {
        std::size_t h = std::hash<int>()(s.xi);
        h ^= std::hash<int>()(s.yi) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int>()(s.ti) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

struct Node {
    double f = 0, g = 0;
    State  state;
    int    parent = -1;
    bool operator>(const Node& o) const { return f > o.f; }
};

// ============================================================
// Path point (for output)
// ============================================================
struct PathPoint { double x, y, theta; };
struct PlannerStats { int expanded = 0, generated = 0, stale = 0; };

// ============================================================
// A* search  (pure grid-based)
// ============================================================
static bool run_astar(const PlannerConfig& cfg,
                      std::vector<PathPoint>& out_path,
                      PlannerStats& stats) {
    Grid grid;
    grid.init(cfg);

    std::printf("  Grid dims  : NX=%d  NY=%d  NT=%d\n", grid.NX, grid.NY, grid.NT);

    // Convert start/goal to grid indices
    State start_s{grid.to_xi(cfg.start.x), grid.to_yi(cfg.start.y),
                  grid.to_ti(cfg.start.theta)};
    State goal_s{grid.to_xi(cfg.goal.x), grid.to_yi(cfg.goal.y),
                 grid.to_ti(cfg.goal.theta)};

    double sx = grid.wx(start_s.xi), sy = grid.wy(start_s.yi);
    double gx = grid.wx(goal_s.xi),  gy = grid.wy(goal_s.yi);

    std::printf("  Grid start : (%d, %d, %d) → (%.3f, %.3f, %.3f)\n",
                start_s.xi, start_s.yi, start_s.ti, sx, sy, grid.wt(start_s.ti));
    std::printf("  Grid goal  : (%d, %d, %d) → (%.3f, %.3f, %.3f)\n",
                goal_s.xi, goal_s.yi, goal_s.ti, gx, gy, grid.wt(goal_s.ti));

    // Validate start
    if (!grid.in_bounds(start_s.xi, start_s.yi)) {
        std::cerr << "ERROR: start is outside grid.\n";
        return false;
    }
    if (!is_free(sx, sy, cfg)) {
        std::cerr << "ERROR: start (" << sx << ", " << sy
                  << ") is in collision or outside map.\n";
        for (std::size_t i = 0; i < cfg.obstacles.size(); ++i) {
            double cl = clearance_to_obstacle(sx, sy, cfg.obstacles[i],
                                               cfg.robot_radius, cfg.safety_margin);
            std::cerr << "  obs[" << i << "] clearance = " << cl << " m\n";
        }
        return false;
    }
    // Validate goal
    if (!grid.in_bounds(goal_s.xi, goal_s.yi)) {
        std::cerr << "ERROR: goal is outside grid.\n";
        return false;
    }
    if (!is_free(gx, gy, cfg)) {
        std::cerr << "ERROR: goal (" << gx << ", " << gy
                  << ") is in collision or outside map.\n";
        for (std::size_t i = 0; i < cfg.obstacles.size(); ++i) {
            double cl = clearance_to_obstacle(gx, gy, cfg.obstacles[i],
                                               cfg.robot_radius, cfg.safety_margin);
            std::cerr << "  obs[" << i << "] clearance = " << cl << " m\n";
        }
        return false;
    }

    // ── A* data structures ─────────────────────────────────
    std::vector<Node> closed;
    closed.reserve(100000);

    using PQ = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
    PQ open;
    std::unordered_map<State, double, StateHash> best;

    double h0 = hypot2(sx - gx, sy - gy);
    Node sn{};
    sn.f = h0; sn.g = 0.0;
    sn.state = start_s;
    sn.parent = -1;

    open.push(sn);
    best[start_s] = 0.0;

    double goal_tol_sq = cfg.goal_tol * cfg.goal_tol;

    while (!open.empty()) {
        Node cur = open.top();
        open.pop();

        // Skip stale entries
        auto it = best.find(cur.state);
        if (it != best.end() && cur.g > it->second + 1e-9) {
            ++stats.stale;
            continue;
        }

        int ci = static_cast<int>(closed.size());
        closed.push_back(cur);
        ++stats.expanded;

        double cur_wx = grid.wx(cur.state.xi);
        double cur_wy = grid.wy(cur.state.yi);

        // Goal test (Euclidean distance between grid cell centres)
        double ddx = cur_wx - gx;
        double ddy = cur_wy - gy;
        if (ddx * ddx + ddy * ddy <= goal_tol_sq) {
            // Trace path back
            std::vector<PathPoint> path;
            int pidx = ci;
            while (pidx >= 0) {
                const Node& n = closed[static_cast<std::size_t>(pidx)];
                path.push_back({grid.wx(n.state.xi),
                                grid.wy(n.state.yi),
                                grid.wt(n.state.ti)});
                pidx = n.parent;
            }
            std::reverse(path.begin(), path.end());
            out_path = std::move(path);
            return true;
        }

        // ── Expand 3 primitives ────────────────────────────
        // All move in the direction of the CURRENT heading (cur.state.ti)
        int di = Grid::DIR[cur.state.ti][0];
        int dj = Grid::DIR[cur.state.ti][1];
        double step_cost = Grid::DIR_COST[cur.state.ti] * std::min(cfg.dx, cfg.dy);

        int nxi = cur.state.xi + di;
        int nyi = cur.state.yi + dj;

        // Bounds check
        if (!grid.in_bounds(nxi, nyi))
            continue;       // no primitives possible from this heading

        double nwx = grid.wx(nxi);
        double nwy = grid.wy(nyi);

        // Collision check: destination AND segment
        if (!is_free(nwx, nwy, cfg))
            continue;
        if (!segment_free(cur_wx, cur_wy, nwx, nwy, cfg))
            continue;

        // This direction is traversable — try all 3 heading outcomes
        int headings[3] = {
            cur.state.ti,                              // forward: keep heading
            (cur.state.ti + 1) % grid.NT,              // forward-left
            ((cur.state.ti - 1) + grid.NT) % grid.NT,  // forward-right
        };
        double costs[3] = { step_cost, step_cost * 1.2, step_cost * 1.2 };

        for (int p = 0; p < 3; ++p) {
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
            ++stats.generated;
        }
    }
    return false;   // open set exhausted
}

// ============================================================
// Save trajectory  (x y theta)
// ============================================================
static void save_trajectory(const std::vector<PathPoint>& path,
                            const std::string& filename) {
    std::ofstream f(filename);
    if (!f) { std::cerr << "ERROR: cannot open " << filename << "\n"; return; }
    f << "x y theta\n";
    for (const auto& p : path) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%.6f %.6f %.6f\n", p.x, p.y, p.theta);
        f << buf;
    }
    std::printf("Trajectory (%zu pts) saved to %s\n", path.size(), filename.c_str());
}

// ============================================================
// Print move-and-stop tuples
// ============================================================
static void print_move_stop_tuples(const std::vector<PathPoint>& path,
                                   std::ostream& os) {
    for (const auto& p : path) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "(\"move\",  %.6f, %.6f, %.6f)\n", p.x, p.y, p.theta);
        os << buf;
        std::snprintf(buf, sizeof(buf),
                      "(\"stop\",  %.6f, %.6f, %.6f)\n", p.x, p.y, p.theta);
        os << buf;
    }
}

// ============================================================
// Post-path safety audit — print per-waypoint clearance
// ============================================================
static void audit_path(const std::vector<PathPoint>& path,
                       const PlannerConfig& cfg) {
    std::printf("\nPath safety audit:\n");
    std::printf("  %-4s  %-8s %-8s  ", "WP", "x", "y");
    for (std::size_t j = 0; j < cfg.obstacles.size(); ++j)
        std::printf("obs[%zu] clr  ", j);
    std::printf("map_clr\n");

    bool any_violation = false;
    for (std::size_t i = 0; i < path.size(); ++i) {
        double px = path[i].x, py = path[i].y;
        std::printf("  %-4zu  %7.3f  %7.3f  ", i, px, py);
        for (std::size_t j = 0; j < cfg.obstacles.size(); ++j) {
            double cl = clearance_to_obstacle(px, py, cfg.obstacles[j],
                                               cfg.robot_radius, cfg.safety_margin);
            std::printf("  %+.4f    ", cl);
            if (cl < -1e-6) any_violation = true;
        }
        // Map clearance (min dist from robot edge to map boundary)
        double map_cl = std::min({px - cfg.robot_radius - cfg.x_min,
                                  cfg.x_max - px - cfg.robot_radius,
                                  py - cfg.robot_radius - cfg.y_min,
                                  cfg.y_max - py - cfg.robot_radius});
        std::printf("  %+.4f", map_cl);
        if (map_cl < -1e-6) any_violation = true;
        std::printf("\n");
    }
    if (any_violation)
        std::printf("  *** WARNING: path has safety violations! ***\n");
    else
        std::printf("  All waypoints have positive clearance. Path is safe.\n");
}

// Apply post-planning offset
static void apply_offset(std::vector<PathPoint>& path,
                         const PlannerConfig& cfg) {
    if (cfg.delta_x == 0.0 && cfg.delta_y == 0.0 && cfg.delta_theta == 0.0)
        return;
    for (auto& p : path) {
        p.x     += cfg.delta_x;
        p.y     += cfg.delta_y;
        p.theta  = wrap_angle(p.theta + cfg.delta_theta);
    }
}

// ============================================================
// CLI
// ============================================================
static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "  --config <path>        Config file    (default: planner.cfg)\n"
        "  --json <path>          TF JSON file   (overrides poses from config)\n"
        "  --start-x <f>         override start x\n"
        "  --start-y <f>         override start y\n"
        "  --start-theta <f>     override start theta\n"
        "  --goal-x <f>          override goal x\n"
        "  --goal-y <f>          override goal y\n"
        "  --goal-theta <f>      override goal theta\n"
        "  --goal-tol <f>        goal tolerance  (default from config)\n"
        "  --traj-file <path>    output file     (default from config)\n"
        "  -h, --help            this message\n",
        prog);
}

static PlannerConfig parse_args(int argc, char* argv[]) {
    PlannerConfig cfg;
    std::string config_path = "planner.cfg";
    std::string json_path;
    bool have_sx = false, have_sy = false, have_st = false;
    bool have_gx = false, have_gy = false, have_gt = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
            break;
        }
    }

    load_config_file(config_path, cfg);

    for (int i = 1; i < argc; ++i) {
        auto eq = [&](const char* s) { return std::strcmp(argv[i], s) == 0; };
        auto nf = [&]() -> double {
            if (i + 1 >= argc) { print_usage(argv[0]); std::exit(1); }
            return std::stod(argv[++i]);
        };
        auto ns = [&]() -> std::string {
            if (i + 1 >= argc) { print_usage(argv[0]); std::exit(1); }
            return argv[++i];
        };

        if      (eq("-h") || eq("--help"))  { print_usage(argv[0]); std::exit(0); }
        else if (eq("--config"))            { ++i; }
        else if (eq("--json"))              { json_path = ns(); }
        else if (eq("--start-x"))           { cfg.start.x     = nf(); have_sx = true; }
        else if (eq("--start-y"))           { cfg.start.y     = nf(); have_sy = true; }
        else if (eq("--start-theta"))       { cfg.start.theta = nf(); have_st = true; }
        else if (eq("--goal-x"))            { cfg.goal.x      = nf(); have_gx = true; }
        else if (eq("--goal-y"))            { cfg.goal.y      = nf(); have_gy = true; }
        else if (eq("--goal-theta"))        { cfg.goal.theta  = nf(); have_gt = true; }
        else if (eq("--goal-tol"))          { cfg.goal_tol    = nf(); }
        else if (eq("--traj-file"))         { cfg.traj_file   = ns(); }
        else {
            std::fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            print_usage(argv[0]);
            std::exit(1);
        }
    }

    if (!json_path.empty()) {
        Pose2D cs = cfg.start, cg = cfg.goal;
        if (!load_tf_json(json_path, cfg))
            std::cerr << "Warning: JSON load failed, using config/CLI values.\n";
        if (have_sx) cfg.start.x     = cs.x;
        if (have_sy) cfg.start.y     = cs.y;
        if (have_st) cfg.start.theta = cs.theta;
        if (have_gx) cfg.goal.x      = cg.x;
        if (have_gy) cfg.goal.y      = cg.y;
        if (have_gt) cfg.goal.theta  = cg.theta;
    }

    cfg.start.theta = wrap_angle(cfg.start.theta);
    cfg.goal.theta  = wrap_angle(cfg.goal.theta);
    return cfg;
}

// ============================================================
// main
// ============================================================
int main(int argc, char* argv[]) {
    PlannerConfig cfg = parse_args(argc, argv);

    std::printf("SE(2) A* planner  [grid-based rewrite]\n");
    std::printf("  Start      : (%.3f, %.3f, %.3f)\n",
                cfg.start.x, cfg.start.y, cfg.start.theta);
    std::printf("  Goal       : (%.3f, %.3f, %.3f)\n",
                cfg.goal.x, cfg.goal.y, cfg.goal.theta);
    std::printf("  Robot R    : %.3f m\n", cfg.robot_radius);
    std::printf("  Safety     : %.3f m\n", cfg.safety_margin);
    std::printf("  Inflation  : %.3f m  (R + margin)\n",
                cfg.robot_radius + cfg.safety_margin);
    std::printf("  Obstacles  : %zu\n", cfg.obstacles.size());
    for (std::size_t i = 0; i < cfg.obstacles.size(); ++i) {
        const auto& ob = cfg.obstacles[i];
        double inf = cfg.robot_radius + cfg.safety_margin;
        std::printf("    [%zu] centre=(%.3f, %.3f) θ=%.3f  L=%.3f W=%.3f\n",
                    i, ob.pose.x, ob.pose.y, ob.pose.theta,
                    ob.length, ob.width);
        std::printf("        inflated AABB: x=[%.3f, %.3f]  y=[%.3f, %.3f]\n",
                    ob.pose.x - ob.length / 2.0 - inf,
                    ob.pose.x + ob.length / 2.0 + inf,
                    ob.pose.y - ob.width / 2.0 - inf,
                    ob.pose.y + ob.width / 2.0 + inf);
    }
    std::printf("  Map        : X[%.1f, %.1f]  Y[%.1f, %.1f]\n",
                cfg.x_min, cfg.x_max, cfg.y_min, cfg.y_max);
    std::printf("  Grid       : dx=%.2f  dy=%.2f  n_theta=%d\n",
                cfg.dx, cfg.dy, cfg.n_theta);

    std::vector<PathPoint> path;
    PlannerStats stats{};

    if (run_astar(cfg, path, stats)) {
        std::printf("Path found: %zu waypoints  "
                    "(expanded=%d, generated=%d, stale=%d)\n",
                    path.size(), stats.expanded, stats.generated, stats.stale);

        apply_offset(path, cfg);
        save_trajectory(path, cfg.traj_file);
        audit_path(path, cfg);

        std::printf("\nMove-and-stop tuples:\n");
        print_move_stop_tuples(path, std::cout);
    } else {
        std::printf("No path found.  (expanded=%d, generated=%d, stale=%d)\n",
                    stats.expanded, stats.generated, stats.stale);
        return 1;
    }
    return 0;
}
