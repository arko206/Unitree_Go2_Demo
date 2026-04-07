/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

// ============================================================================
// Parameter Macros
// These act as compile-time defaults and can be overridden with -D flags.
// Runtime environment variables can still override many of them without rebuild.
// ============================================================================

#ifndef DEMO_PARAM_COMMAND_TIMEOUT_SEC
#define DEMO_PARAM_COMMAND_TIMEOUT_SEC 10.0
#endif

#ifndef DEMO_PARAM_CONTROL_DT_SEC
#define DEMO_PARAM_CONTROL_DT_SEC 0.05
#endif

#ifndef DEMO_PARAM_POSE_STALE_TIMEOUT_SEC
#define DEMO_PARAM_POSE_STALE_TIMEOUT_SEC 0.50
#endif

#ifndef DEMO_PARAM_KP_FORWARD
#define DEMO_PARAM_KP_FORWARD 1.0
#endif

#ifndef DEMO_PARAM_KP_LATERAL
#define DEMO_PARAM_KP_LATERAL 0.20
#endif

#ifndef DEMO_PARAM_KP_YAW
#define DEMO_PARAM_KP_YAW 0.3
#endif

#ifndef DEMO_PARAM_MAX_VX
#define DEMO_PARAM_MAX_VX 0.12
#endif

#ifndef DEMO_PARAM_MAX_VY
#define DEMO_PARAM_MAX_VY 0.08
#endif

#ifndef DEMO_PARAM_MAX_VYAW
#define DEMO_PARAM_MAX_VYAW 0.60
#endif

#ifndef DEMO_PARAM_WAYPOINT_POS_TOL
#define DEMO_PARAM_WAYPOINT_POS_TOL 0.12
#endif

#ifndef DEMO_PARAM_WAYPOINT_ANGLE_TOL
#define DEMO_PARAM_WAYPOINT_ANGLE_TOL 0.10
#endif

#ifndef DEMO_PARAM_START_WAYPOINT_INDEX
#define DEMO_PARAM_START_WAYPOINT_INDEX 1
#endif

#ifndef DEMO_PARAM_MAX_EXECUTE_WAYPOINTS
#define DEMO_PARAM_MAX_EXECUTE_WAYPOINTS 10
#endif

#ifndef DEMO_PARAM_STATUS_PRINT_EVERY_N
#define DEMO_PARAM_STATUS_PRINT_EVERY_N 10
#endif

#ifndef DEMO_PARAM_STATE_LOG_EVERY_N
#define DEMO_PARAM_STATE_LOG_EVERY_N 5
#endif

#ifndef DEMO_PARAM_LOG_DIR
#define DEMO_PARAM_LOG_DIR "./Demo_live_Go2_logs"
#endif

#ifndef DEMO_PARAM_POSE_FILE_PRIMARY
#define DEMO_PARAM_POSE_FILE_PRIMARY "../../live_bridge/Second_DraftDemo_floor_robot_pose.txt"
#endif

#ifndef DEMO_PARAM_WAYPOINT_FILE_PRIMARY
#define DEMO_PARAM_WAYPOINT_FILE_PRIMARY "../../live_bridge/ThirdAstar_colldemo_se2_path_floor.txt"
#endif


namespace fs = std::filesystem;
using unitree::common::ThreadPtr;

namespace {

double WrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

template <typename T>
T ClampValue(T value, T low, T high) {
  return std::max(low, std::min(high, value));
}

double ReadEnvDouble(const char* name, double fallback) {
  const char* raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0') {
    return fallback;
  }

  char* end = nullptr;
  const double parsed = std::strtod(raw, &end);
  return end == raw ? fallback : parsed;
}

size_t ReadEnvSizeT(const char* name, size_t fallback) {
  const char* raw = std::getenv(name);
  if (raw == nullptr || *raw == '\0') {
    return fallback;
  }

  char* end = nullptr;
  const unsigned long parsed = std::strtoul(raw, &end, 10);
  return end == raw ? fallback : static_cast<size_t>(parsed);
}

std::string ReadEnvString(const char* name, const std::string& fallback) {
  const char* raw = std::getenv(name);
  return (raw == nullptr || *raw == '\0') ? fallback : std::string(raw);
}

std::string ResolvePathFromCandidates(
    const std::string& env_name,
    const std::vector<std::string>& candidates) {
  const std::string env_value = ReadEnvString(env_name.c_str(), "");
  if (!env_value.empty()) {
    return env_value;
  }

  for (const auto& candidate : candidates) {
    std::error_code ec;
    if (fs::exists(candidate, ec)) {
      return candidate;
    }
  }

  return candidates.empty() ? std::string() : candidates.front();
}

std::string TimeStringNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&now_time, &local_tm);

  std::ostringstream oss;
  oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

bool ParseFirstThreeNumbers(const std::string& input,
                            double& a,
                            double& b,
                            double& c) {
  std::vector<double> values;
  std::string token;

  auto flush_token = [&]() {
    if (token.empty()) {
      return;
    }
    try {
      values.push_back(std::stod(token));
    } catch (const std::exception&) {
    }
    token.clear();
  };

  for (const char ch : input) {
    const bool numeric =
        std::isdigit(static_cast<unsigned char>(ch)) != 0 ||
        ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E';
    if (numeric) {
      token.push_back(ch);
    } else {
      flush_token();
    }
  }
  flush_token();

  if (values.size() < 3) {
    return false;
  }

  a = values[0];
  b = values[1];
  c = values[2];
  return true;
}

struct Waypoint {
  int idx = 0;
  double x = 0.0;
  double y = 0.0;   // second planar axis; works for both old z and new floor-y files
  double theta = 0.0;
  double g = 0.0;
  double h = 0.0;
  double f = 0.0;
  std::string action;
};

struct PoseSample {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  std::chrono::steady_clock::time_point stamp = std::chrono::steady_clock::now();
};

struct DemoParams {
  double command_timeout_sec = ReadEnvDouble(
      "DEMO_LIVE_GO2_COMMAND_TIMEOUT_SEC", DEMO_PARAM_COMMAND_TIMEOUT_SEC);
  double control_dt_sec = ReadEnvDouble(
      "DEMO_LIVE_GO2_CONTROL_DT_SEC", DEMO_PARAM_CONTROL_DT_SEC);
  double pose_stale_timeout_sec = ReadEnvDouble(
      "DEMO_LIVE_GO2_POSE_STALE_TIMEOUT_SEC", DEMO_PARAM_POSE_STALE_TIMEOUT_SEC);

  double kp_forward = ReadEnvDouble(
      "DEMO_LIVE_GO2_KP_FORWARD", DEMO_PARAM_KP_FORWARD);
  double kp_lateral = ReadEnvDouble(
      "DEMO_LIVE_GO2_KP_LATERAL", DEMO_PARAM_KP_LATERAL);
  double kp_yaw = ReadEnvDouble(
      "DEMO_LIVE_GO2_KP_YAW", DEMO_PARAM_KP_YAW);

  double max_vx = ReadEnvDouble("DEMO_LIVE_GO2_MAX_VX", DEMO_PARAM_MAX_VX);
  double max_vy = ReadEnvDouble("DEMO_LIVE_GO2_MAX_VY", DEMO_PARAM_MAX_VY);
  double max_vyaw = ReadEnvDouble("DEMO_LIVE_GO2_MAX_VYAW", DEMO_PARAM_MAX_VYAW);

  double waypoint_pos_tol = ReadEnvDouble(
      "DEMO_LIVE_GO2_WAYPOINT_POS_TOL", DEMO_PARAM_WAYPOINT_POS_TOL);
  double waypoint_angle_tol = ReadEnvDouble(
      "DEMO_LIVE_GO2_WAYPOINT_ANGLE_TOL", DEMO_PARAM_WAYPOINT_ANGLE_TOL);

  size_t start_waypoint_index = ReadEnvSizeT(
      "DEMO_LIVE_GO2_START_WAYPOINT_INDEX", DEMO_PARAM_START_WAYPOINT_INDEX);
  size_t max_execute_waypoints = ReadEnvSizeT(
      "DEMO_LIVE_GO2_MAX_EXECUTE_WAYPOINTS", DEMO_PARAM_MAX_EXECUTE_WAYPOINTS);
  size_t status_print_every_n = ReadEnvSizeT(
      "DEMO_LIVE_GO2_STATUS_PRINT_EVERY_N", DEMO_PARAM_STATUS_PRINT_EVERY_N);
  size_t state_log_every_n = ReadEnvSizeT(
      "DEMO_LIVE_GO2_STATE_LOG_EVERY_N", DEMO_PARAM_STATE_LOG_EVERY_N);

  std::string log_dir = ReadEnvString(
      "DEMO_LIVE_GO2_LOG_DIR", DEMO_PARAM_LOG_DIR);
  std::string pose_file = ResolvePathFromCandidates(
      "DEMO_LIVE_GO2_POSE_FILE",
      {DEMO_PARAM_POSE_FILE_PRIMARY});
  std::string waypoint_file = ResolvePathFromCandidates(
      "DEMO_LIVE_GO2_WAYPOINT_FILE",
      {DEMO_PARAM_WAYPOINT_FILE_PRIMARY});
};

}  // namespace

class Custom {
 public:
  Custom() {
    sport_client_.SetTimeout(static_cast<float>(params_.command_timeout_sec));
    sport_client_.Init();

    subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
            TOPIC_HIGHSTATE));
    subscriber_->InitChannel(
        std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

    OpenLogs();
    LoadWaypoints();
    PrintParameterSummary();
  }

  ~Custom() {
    if (state_log_.is_open()) {
      state_log_.close();
    }
    if (control_log_.is_open()) {
      control_log_.close();
    }
  }

  double control_dt_sec() const { return params_.control_dt_sec; }

  void GetInitState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_) {
      std::cout << "[Init] Waiting for robot state..." << std::endl;
      return;
    }

    const double px0 = state_.position()[0];
    const double py0 = state_.position()[1];
    const double yaw0 = state_.imu_state().rpy()[2];
    std::cout << "[Init] position=(" << px0 << ", " << py0
              << ") yaw=" << yaw0 << std::endl;

    sport_client_.StaticWalk();
    sleep(5);
  }

  void RobotControl() {
    ++control_cycle_count_;

    PollPoseFile();

    if (!HasFreshPose()) {
        StopRobot("bridge pose unavailable or stale");
        return;
    }

    if (!waypoints_loaded_) {
        StopRobot("waypoint file not loaded");
        return;
    }

    if (current_waypoint_idx_ >= execution_waypoint_limit_index_) {
        StopRobot("configured waypoint execution limit reached");
        return;
    }

    if (current_waypoint_idx_ >= waypoints_.size()) {
        StopRobot("all waypoints completed");
        return;
    }

    const Waypoint& wp = waypoints_[current_waypoint_idx_];
    const PoseSample& pose = pose_sample_;

    const double ex_world = wp.x - pose.x;
    const double ey_world = wp.y - pose.y;
    const double dist_to_waypoint = std::hypot(ex_world, ey_world);

    const double desired_heading = std::atan2(ey_world, ex_world);
    const double e_theta = WrapAngle(desired_heading - pose.theta);

    const double e_forward =
        std::cos(pose.theta) * ex_world + std::sin(pose.theta) * ey_world;

    const double e_left =
        -std::sin(pose.theta) * ex_world + std::cos(pose.theta) * ey_world;




    double vx_cmd = ClampValue(params_.kp_forward * e_forward,
                               -params_.max_vx,
                               params_.max_vx);

    double vy_cmd = 0.0;

    double vyaw_cmd = ClampValue(params_.kp_yaw * e_theta,
                                 -params_.max_vyaw,
                                 params_.max_vyaw);

    sport_client_.Move(vx_cmd, vy_cmd, vyaw_cmd);



    if (dist_to_waypoint <= params_.waypoint_pos_tol) {
        std::cout << "[Waypoint] reached idx=" << wp.idx
                  << " action=" << wp.action
                  << " pose=(" << pose.x << ", " << pose.y << ", " << pose.theta << ")"
                  << std::endl;

        ++current_waypoint_idx_;

        if (current_waypoint_idx_ >= execution_waypoint_limit_index_ ||
            current_waypoint_idx_ >= waypoints_.size()) {
            StopRobot("final test waypoint reached");
        }
        return;
    }

  
    last_stop_reason_.clear();

    LogControlSnapshot(wp,
                       pose,
                       e_forward,
                       e_left,
                       e_theta,
                       dist_to_waypoint,
                       vx_cmd,
                       vy_cmd,
                       vyaw_cmd);

    const size_t print_every = std::max<size_t>(params_.status_print_every_n, 1);
    if ((control_cycle_count_ % print_every) == 0) {
        std::cout << "[Track] wp=" << wp.idx
                  << " target=(" << wp.x << ", " << wp.y << ", " << wp.theta << ")"
                  << " pose=(" << pose.x << ", " << pose.y << ", " << pose.theta << ")"
                  << " err_f=" << e_forward
                  << " err_l=" << e_left
                  << " err_yaw=" << e_theta
                  << " dist=" << dist_to_waypoint
                  << " cmd=(" << vx_cmd << ", " << vy_cmd << ", " << vyaw_cmd << ")"
                  << std::endl;
    }
}
 private:
  void PrintParameterSummary() const {
    std::cout << "[Config] control_dt_sec=" << params_.control_dt_sec
              << " pose_stale_timeout_sec=" << params_.pose_stale_timeout_sec
              << " start_waypoint_index=" << params_.start_waypoint_index
              << " max_execute_waypoints=" << params_.max_execute_waypoints
              << std::endl;
    std::cout << "[Config] kp_forward=" << params_.kp_forward
              << " kp_lateral=" << params_.kp_lateral
              << " kp_yaw=" << params_.kp_yaw << std::endl;
    std::cout << "[Config] max_vx=" << params_.max_vx
              << " max_vy=" << params_.max_vy
              << " max_vyaw=" << params_.max_vyaw << std::endl;
    std::cout << "[Config] pose_file=" << params_.pose_file << std::endl;
    std::cout << "[Config] waypoint_file=" << params_.waypoint_file << std::endl;
    std::cout << "[Config] log_dir=" << params_.log_dir << std::endl;
  }

  void OpenLogs() {
    std::error_code ec;
    fs::create_directories(params_.log_dir, ec);

    state_log_.open(fs::path(params_.log_dir) / "sport_state.log",
                    std::ios::out | std::ios::app);
    control_log_.open(fs::path(params_.log_dir) / "controller_trace.csv",
                      std::ios::out | std::ios::app);

    if (control_log_.is_open()) {
      control_log_ << "timestamp,waypoint_idx,target_x,target_y,target_theta,"
                   << "pose_x,pose_y,pose_theta,error_forward,error_right,"
                   << "error_theta,distance,vx_cmd,vy_cmd,vyaw_cmd,action"
                   << std::endl;
    }
  }

  bool LoadWaypoints() {
    std::ifstream infile(params_.waypoint_file);
    if (!infile.is_open()) {
      std::cerr << "[Waypoints] failed to open " << params_.waypoint_file << std::endl;
      waypoints_loaded_ = false;
      return false;
    }

    waypoints_.clear();

    std::string header_line;
    std::getline(infile, header_line);

    std::string line;
    while (std::getline(infile, line)) {
      if (line.empty()) {
        continue;
      }

      std::istringstream iss(line);
      Waypoint wp;
      if (!(iss >> wp.idx >> wp.x >> wp.y >> wp.theta >> wp.g >> wp.h >> wp.f >>
            wp.action)) {
        std::cerr << "[Waypoints] skipping malformed line: " << line << std::endl;
        continue;
      }
      waypoints_.push_back(wp);
    }

    if (waypoints_.empty()) {
      std::cerr << "[Waypoints] no valid entries loaded" << std::endl;
      waypoints_loaded_ = false;
      return false;
    }

    current_waypoint_idx_ =
        std::min(params_.start_waypoint_index, waypoints_.size() - 1);
    execution_waypoint_limit_index_ = waypoints_.size();
    if (params_.max_execute_waypoints > 0) {
      execution_waypoint_limit_index_ = std::min(
          waypoints_.size(),
          current_waypoint_idx_ + params_.max_execute_waypoints);
    }
    waypoints_loaded_ = true;

    std::cout << "[Waypoints] loaded " << waypoints_.size()
              << " entries, starting at index " << current_waypoint_idx_
              << ", execution limit index " << execution_waypoint_limit_index_
              << std::endl;
    return true;
  }

  bool PollPoseFile() {
    if (params_.pose_file.empty()) {
      pose_valid_ = false;
      return false;
    }

    std::error_code ec;
    if (!fs::exists(params_.pose_file, ec)) {
      pose_valid_ = false;
      return false;
    }

    const auto write_time = fs::last_write_time(params_.pose_file, ec);
    if (ec) {
      pose_valid_ = false;
      return false;
    }

    if (pose_file_write_time_.has_value() &&
        write_time == *pose_file_write_time_ &&
        pose_valid_) {
      return true;
    }

    std::ifstream infile(params_.pose_file);
    if (!infile.is_open()) {
      pose_valid_ = false;
      return false;
    }

    std::string line;
    std::string last_non_empty_line;
    while (std::getline(infile, line)) {
      if (!line.empty()) {
        last_non_empty_line = line;
      }
    }

    if (last_non_empty_line.empty()) {
      pose_valid_ = false;
      return false;
    }

    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_theta = 0.0;
    if (!ParseFirstThreeNumbers(last_non_empty_line, pose_x, pose_y, pose_theta)) {
      pose_valid_ = false;
      return false;
    }

    pose_sample_.x = pose_x;
    pose_sample_.y = pose_y;
    pose_sample_.theta = WrapAngle(pose_theta);
    pose_sample_.stamp = std::chrono::steady_clock::now();
    pose_file_write_time_ = write_time;
    pose_valid_ = true;
    return true;
  }

  bool HasFreshPose() const {
    if (!pose_valid_) {
      return false;
    }

    const auto age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - pose_sample_.stamp);
    return age.count() <= params_.pose_stale_timeout_sec;
  }

  void StopRobot(const std::string& reason) {
    sport_client_.StopMove();
    if (last_stop_reason_ != reason) {
      std::cout << "[Stop] " << reason << std::endl;
      last_stop_reason_ = reason;
    }
  }

  void LogControlSnapshot(const Waypoint& wp,
                          const PoseSample& pose,
                          double e_forward,
                          double e_right,
                          double e_theta,
                          double distance,
                          double vx_cmd,
                          double vy_cmd,
                          double vyaw_cmd) {
    if (!control_log_.is_open()) {
      return;
    }

    control_log_ << TimeStringNow() << ','
                 << wp.idx << ','
                 << wp.x << ','
                 << wp.y << ','
                 << wp.theta << ','
                 << pose.x << ','
                 << pose.y << ','
                 << pose.theta << ','
                 << e_forward << ','
                 << e_right << ','
                 << e_theta << ','
                 << distance << ','
                 << vx_cmd << ','
                 << vy_cmd << ','
                 << vyaw_cmd << ','
                 << wp.action << std::endl;
  }

  void HighStateHandler(const void* message) {
    const auto* incoming =
        static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = *incoming;
      state_received_ = true;
    }

    ++state_log_counter_;
    const size_t log_every = std::max<size_t>(params_.state_log_every_n, 1);
    if (!state_log_.is_open() || (state_log_counter_ % log_every) != 0) {
      return;
    }

    state_log_ << TimeStringNow()
               << " position=(" << incoming->position()[0] << ", "
               << incoming->position()[1] << ", "
               << incoming->position()[2] << ")"
               << " velocity=(" << incoming->velocity()[0] << ", "
               << incoming->velocity()[1] << ", "
               << incoming->velocity()[2] << ")"
               << " rpy=(" << incoming->imu_state().rpy()[0] << ", "
               << incoming->imu_state().rpy()[1] << ", "
               << incoming->imu_state().rpy()[2] << ")"
               << std::endl;
  }

  DemoParams params_{};

  unitree_go::msg::dds_::SportModeState_ state_{};
  std::mutex state_mutex_;
  bool state_received_ = false;

  unitree::robot::go2::SportClient sport_client_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
      subscriber_;

  std::ofstream state_log_;
  std::ofstream control_log_;

  std::vector<Waypoint> waypoints_;
  bool waypoints_loaded_ = false;
  size_t current_waypoint_idx_ = 0;
  size_t execution_waypoint_limit_index_ = 0;

  PoseSample pose_sample_{};
  bool pose_valid_ = false;
  std::optional<fs::file_time_type> pose_file_write_time_;

  size_t control_cycle_count_ = 0;
  size_t state_log_counter_ = 0;
  std::string last_stop_reason_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    return -1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  std::locale::global(std::locale::classic());
  std::cout.imbue(std::locale::classic());
  std::cerr.imbue(std::locale::classic());

  Custom custom;

  std::this_thread::sleep_for(std::chrono::seconds(1));
  custom.GetInitState();

  ThreadPtr control_thread = unitree::common::CreateRecurrentThread(
      static_cast<int64_t>(custom.control_dt_sec() * 1000000.0),
      std::bind(&Custom::RobotControl, &custom));
  (void)control_thread;

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
