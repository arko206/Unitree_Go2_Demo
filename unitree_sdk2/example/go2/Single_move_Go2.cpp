/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <atomic>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

namespace fs = std::filesystem;

namespace {

std::string TimeStringNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&now_time, &local_tm);

  std::ostringstream oss;
  oss << "[" << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S") << "]";
  return oss.str();
}

std::string NormalizeCommandLine(std::string line) {
  for (char& ch : line) {
    if (ch == ',') {
      ch = ' ';
    }
  }
  return line;
}

std::string ToLowerAscii(std::string text) {
  for (char& ch : text) {
    if (ch >= 'A' && ch <= 'Z') {
      ch = static_cast<char>(ch - 'A' + 'a');
    }
  }
  return text;
}

bool TryParseDouble(const std::string& token, double& value) {
  char* end = nullptr;
  value = std::strtod(token.c_str(), &end);
  return end != token.c_str() && *end == '\0';
}

std::vector<std::string> Tokenize(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;

  while (iss >> token) {
    tokens.push_back(token);
  }

  return tokens;
}

}  // namespace

class Custom {
 public:
  Custom() {
    sport_client_.SetTimeout(10.0f);
    sport_client_.Init();

    suber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
            TOPIC_HIGHSTATE));
    suber_->InitChannel(
        std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

    const fs::path log_dir = "./Single_Movement_Go2/Fifth_Move";
    std::error_code ec;
    fs::create_directories(log_dir, ec);

    const fs::path pos_log = log_dir / "Fifth_Move_pos_log.txt";
    const fs::path foot_log = log_dir / "Fifth_Move_Foot_log.txt";

    highstate_logfile_.open(pos_log, std::ios::out | std::ios::app);
    if (!highstate_logfile_.is_open()) {
      std::cerr << "Failed to open " << pos_log << " (errno=" << errno
                << " - " << std::strerror(errno) << ")\n";
    }

    footpos_velocity_logfile_.open(foot_log, std::ios::out | std::ios::app);
    if (!footpos_velocity_logfile_.is_open()) {
      std::cerr << "Failed to open " << foot_log << " (errno=" << errno
                << " - " << std::strerror(errno) << ")\n";
    }
  }

  ~Custom() {
    StopActiveCommand("program shutdown", true);
    ShutdownCommandLoop();

    if (highstate_logfile_.is_open()) {
      highstate_logfile_.close();
    }

    if (footpos_velocity_logfile_.is_open()) {
      footpos_velocity_logfile_.close();
    }
  }

  void GetInitState() {
    std::cout << "Waiting for robot state..." << std::endl;
    while (!state_received_.load()) {
      usleep(10000);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      px0_ = state_.position()[0];
      py0_ = state_.position()[1];
      yaw0_ = state_.imu_state().rpy()[2];
    }

    std::cout << "Initial position: x0=" << px0_ << ", y0=" << py0_
              << ", yaw0=" << yaw0_ << std::endl;

    sport_client_.StaticWalk();
    sleep(6);

    const PoseSnapshot pose = GetPoseSnapshot();
    std::cout << "Mode of the Go2 is: " << pose.mode << std::endl;
  }

  void StartCommandLoop() {
    bool expected = false;
    if (!command_loop_started_.compare_exchange_strong(expected, true)) {
      return;
    }

    shutdown_requested_.store(false);
    command_thread_ = std::thread(&Custom::CommandLoop, this);
  }

  void RunInteractiveCli() {
    PrintHelp();

    std::string line;
    while (true) {
      std::cout << "\ncmd> " << std::flush;
      if (!std::getline(std::cin, line)) {
        std::cout << "\nInput stream closed. Exiting." << std::endl;
        break;
      }

      const std::vector<std::string> tokens = Tokenize(NormalizeCommandLine(line));
      if (tokens.empty()) {
        continue;
      }

      const std::string command = ToLowerAscii(tokens[0]);

      if (command == "help" || command == "h" || command == "?") {
        PrintHelp();
        continue;
      }

      if (command == "status" || command == "s") {
        PrintStatus();
        continue;
      }

      if (command == "stop") {
        StopActiveCommand("manual stop", false);
        continue;
      }

      if (command == "quit" || command == "exit" || command == "q") {
        StopActiveCommand("CLI exit", true);
        break;
      }

      if (command == "set") {
        if (tokens.size() != 4) {
          std::cout << "Usage: set <vx> <vy> <vyaw>" << std::endl;
          continue;
        }

        double vx = 0.0;
        double vy = 0.0;
        double vyaw = 0.0;
        if (!TryParseDouble(tokens[1], vx) ||
            !TryParseDouble(tokens[2], vy) ||
            !TryParseDouble(tokens[3], vyaw)) {
          std::cout << "Failed to parse numeric values for set." << std::endl;
          continue;
        }

        SetStreamingCommand(vx, vy, vyaw);
        continue;
      }

      if (command == "run" || command == "once" || command == "pulse") {
        if (tokens.size() != 5) {
          std::cout << "Usage: run <vx> <vy> <vyaw> <duration_sec>" << std::endl;
          continue;
        }

        double vx = 0.0;
        double vy = 0.0;
        double vyaw = 0.0;
        double duration_sec = 0.0;
        if (!TryParseDouble(tokens[1], vx) ||
            !TryParseDouble(tokens[2], vy) ||
            !TryParseDouble(tokens[3], vyaw) ||
            !TryParseDouble(tokens[4], duration_sec)) {
          std::cout << "Failed to parse numeric values for run." << std::endl;
          continue;
        }

        if (duration_sec <= 0.0) {
          std::cout << "Duration must be greater than zero." << std::endl;
          continue;
        }

        RunTimedCommand(vx, vy, vyaw, duration_sec);
        continue;
      }

      std::vector<double> values;
      values.reserve(tokens.size());
      bool all_numeric = true;
      for (const auto& token : tokens) {
        double value = 0.0;
        if (!TryParseDouble(token, value)) {
          all_numeric = false;
          break;
        }
        values.push_back(value);
      }

      if (all_numeric && values.size() == 3) {
        SetStreamingCommand(values[0], values[1], values[2]);
        continue;
      }

      if (all_numeric && values.size() == 4) {
        if (values[3] <= 0.0) {
          std::cout << "Duration must be greater than zero." << std::endl;
          continue;
        }

        RunTimedCommand(values[0], values[1], values[2], values[3]);
        continue;
      }

      std::cout << "Unknown command. Type `help` for usage." << std::endl;
    }
  }

 private:
  struct PoseSnapshot {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    int mode = 0;
  };

  struct MotionCommand {
    bool active = false;
    bool timed = false;
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    double duration_sec = 0.0;
    std::chrono::steady_clock::time_point start_time{};
    std::chrono::steady_clock::time_point deadline{};
    PoseSnapshot start_pose{};
  };

  void HighStateHandler(const void* message) {
    const auto* incoming =
        static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

    unitree_go::msg::dds_::SportModeState_ snapshot;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = *incoming;
      snapshot = state_;
    }
    state_received_.store(true);

    const std::string time_string = TimeStringNow();

    if (highstate_logfile_.is_open()) {
      highstate_logfile_ << time_string << " Position: " << snapshot.position()[0]
                         << ", " << snapshot.position()[1] << ", "
                         << snapshot.position()[2] << " | IMU RPY: "
                         << snapshot.imu_state().rpy()[0] << ", "
                         << snapshot.imu_state().rpy()[1] << ", "
                         << snapshot.imu_state().rpy()[2] << " | Angular Vel: "
                         << snapshot.imu_state().gyroscope()[0] << ", "
                         << snapshot.imu_state().gyroscope()[1] << ", "
                         << snapshot.imu_state().gyroscope()[2]
                         << " | Acceleration: "
                         << snapshot.imu_state().accelerometer()[0] << ", "
                         << snapshot.imu_state().accelerometer()[1] << ", "
                         << snapshot.imu_state().accelerometer()[2]
                         << " | Quaternion: "
                         << snapshot.imu_state().quaternion()[0] << ","
                         << snapshot.imu_state().quaternion()[1] << ","
                         << snapshot.imu_state().quaternion()[2] << ","
                         << snapshot.imu_state().quaternion()[3] << " | Mode: "
                         << std::to_string(snapshot.mode())
                         << " | String Value for Mode: "
                         << std::bitset<8>(snapshot.mode()).to_string()
                         << " | Velocity: "
                         << "Vx=" << snapshot.velocity()[0] << ", "
                         << "Vy=" << snapshot.velocity()[1] << ", "
                         << "Vz=" << snapshot.velocity()[2] << std::endl;
      highstate_logfile_ << std::flush;
    }

    if (footpos_velocity_logfile_.is_open()) {
      footpos_velocity_logfile_ << time_string << " Velocity: "
                                << "Vx=" << snapshot.velocity()[0] << ", "
                                << "Vy=" << snapshot.velocity()[1] << ", "
                                << "Vz=" << snapshot.velocity()[2] << ", "
                                << "FR(x,y,z)="
                                << snapshot.foot_position_body()[0] << ","
                                << snapshot.foot_position_body()[1] << ","
                                << snapshot.foot_position_body()[2] << ", "
                                << "FL(x,y,z)="
                                << snapshot.foot_position_body()[3] << ","
                                << snapshot.foot_position_body()[4] << ","
                                << snapshot.foot_position_body()[5] << ", "
                                << "RR(x,y,z)="
                                << snapshot.foot_position_body()[6] << ","
                                << snapshot.foot_position_body()[7] << ","
                                << snapshot.foot_position_body()[8] << ", "
                                << "RL(x,y,z)="
                                << snapshot.foot_position_body()[9] << ","
                                << snapshot.foot_position_body()[10] << ","
                                << snapshot.foot_position_body()[11]
                                << std::endl;
      footpos_velocity_logfile_ << std::flush;
    }
  }

  PoseSnapshot GetPoseSnapshot() const {
    PoseSnapshot pose;

    std::lock_guard<std::mutex> lock(state_mutex_);
    pose.x = state_.position()[0];
    pose.y = state_.position()[1];
    pose.z = state_.position()[2];
    pose.roll = state_.imu_state().rpy()[0];
    pose.pitch = state_.imu_state().rpy()[1];
    pose.yaw = state_.imu_state().rpy()[2];
    pose.vx = state_.velocity()[0];
    pose.vy = state_.velocity()[1];
    pose.vz = state_.velocity()[2];
    pose.mode = static_cast<int>(state_.mode());

    return pose;
  }

  void PrintHelp() const {
    std::cout << "\nInteractive motion CLI\n"
              << "  help                     Show this help\n"
              << "  status                   Print pose, velocity, mode, and active command\n"
              << "  set  <vx> <vy> <vyaw>    Start or replace a continuous streamed command\n"
              << "  run  <vx> <vy> <vyaw> <duration_sec>\n"
              << "                           Run a timed command, then auto-stop\n"
              << "  stop                     Stop the active command\n"
              << "  quit                     Stop and exit\n"
              << "\nShortcuts:\n"
              << "  <vx> <vy> <vyaw>                     same as set\n"
              << "  <vx> <vy> <vyaw> <duration_sec>      same as run\n"
              << std::endl;
  }

  void PrintStatus() const {
    const PoseSnapshot pose = GetPoseSnapshot();

    MotionCommand command_snapshot;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_snapshot = command_;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "Pose: x=" << pose.x << " y=" << pose.y << " z=" << pose.z
        << " yaw=" << pose.yaw << " | Velocity: vx=" << pose.vx
        << " vy=" << pose.vy << " vz=" << pose.vz
        << " | Mode=" << pose.mode << '\n';

    if (command_snapshot.active) {
      oss << "Active command: vx=" << command_snapshot.vx
          << " vy=" << command_snapshot.vy
          << " vyaw=" << command_snapshot.vyaw;
      if (command_snapshot.timed) {
        const auto now = std::chrono::steady_clock::now();
        const double remaining_sec = std::max(
            0.0,
            std::chrono::duration<double>(command_snapshot.deadline - now).count());
        oss << " | remaining=" << remaining_sec << " s";
      } else {
        oss << " | streaming";
      }
    } else {
      oss << "Active command: none";
    }

    std::cout << oss.str() << std::endl;
  }

  void SetStreamingCommand(double vx, double vy, double vyaw) {
    const PoseSnapshot pose = GetPoseSnapshot();
    const auto now = std::chrono::steady_clock::now();

    MotionCommand previous_command;
    bool had_previous = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_.active) {
        previous_command = command_;
        had_previous = true;
      }

      command_ = {};
      command_.active = true;
      command_.timed = false;
      command_.vx = vx;
      command_.vy = vy;
      command_.vyaw = vyaw;
      command_.start_time = now;
      command_.start_pose = pose;
    }

    if (had_previous) {
      PrintCommandSummary(previous_command, pose, "replaced");
    }

    std::cout << "Streaming command updated: vx=" << vx << ", vy=" << vy
              << ", vyaw=" << vyaw << std::endl;
  }

  void RunTimedCommand(double vx, double vy, double vyaw, double duration_sec) {
    const PoseSnapshot pose = GetPoseSnapshot();
    const auto now = std::chrono::steady_clock::now();

    MotionCommand previous_command;
    bool had_previous = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_.active) {
        previous_command = command_;
        had_previous = true;
      }

      command_ = {};
      command_.active = true;
      command_.timed = true;
      command_.vx = vx;
      command_.vy = vy;
      command_.vyaw = vyaw;
      command_.duration_sec = duration_sec;
      command_.start_time = now;
      command_.deadline = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(duration_sec));
      command_.start_pose = pose;
    }

    if (had_previous) {
      PrintCommandSummary(previous_command, pose, "replaced");
    }

    std::cout << "Timed command started: vx=" << vx << ", vy=" << vy
              << ", vyaw=" << vyaw << ", duration=" << duration_sec << " s"
              << std::endl;
  }

  void StopActiveCommand(const std::string& reason, bool quiet_if_idle) {
    const PoseSnapshot pose = GetPoseSnapshot();

    MotionCommand finished_command;
    bool had_active = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_.active) {
        finished_command = command_;
        command_ = {};
        had_active = true;
      }
    }

    if (had_active) {
      PrintCommandSummary(finished_command, pose, reason);
    } else if (!quiet_if_idle) {
      std::cout << "No active motion command." << std::endl;
    }
  }

  void PrintCommandSummary(const MotionCommand& command,
                           const PoseSnapshot& end_pose,
                           const std::string& reason) const {
    const double elapsed_sec = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() -
                                   command.start_time)
                                   .count();
    const double dx_world = end_pose.x - command.start_pose.x;
    const double dy_world = end_pose.y - command.start_pose.y;
    const double distance_world =
        std::sqrt(dx_world * dx_world + dy_world * dy_world);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "\n========== COMMAND SUMMARY ==========\n";
    oss << "Reason: " << reason << '\n';
    oss << "Commanded body velocity: vx=" << command.vx << " m/s, vy="
        << command.vy << " m/s, vyaw=" << command.vyaw << " rad/s\n";
    if (command.timed) {
      oss << "Requested duration: " << command.duration_sec << " s\n";
    }
    oss << "Measured duration: " << elapsed_sec << " s\n";
    oss << "Start pose: x=" << command.start_pose.x
        << ", y=" << command.start_pose.y
        << ", yaw=" << command.start_pose.yaw << '\n';
    oss << "End pose: x=" << end_pose.x << ", y=" << end_pose.y
        << ", yaw=" << end_pose.yaw << '\n';
    oss << "World-frame displacement: dx=" << dx_world << " m, dy="
        << dy_world << " m, distance=" << distance_world << " m\n";
    oss << "=====================================\n";

    std::cout << oss.str() << std::flush;
  }

  void CommandLoop() {
    bool last_command_was_active = false;

    while (!shutdown_requested_.load()) {
      MotionCommand expired_command;
      bool command_expired = false;
      MotionCommand command_snapshot;

      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (command_.active && command_.timed &&
            std::chrono::steady_clock::now() >= command_.deadline) {
          expired_command = command_;
          command_ = {};
          command_expired = true;
        }
        command_snapshot = command_;
      }

      if (command_expired) {
        sport_client_.StopMove();
        last_command_was_active = false;
        PrintCommandSummary(expired_command, GetPoseSnapshot(), "completed");
      } else if (command_snapshot.active) {
        sport_client_.Move(command_snapshot.vx,
                           command_snapshot.vy,
                           command_snapshot.vyaw);
        last_command_was_active = true;
      } else if (last_command_was_active) {
        sport_client_.StopMove();
        last_command_was_active = false;
      }

      std::this_thread::sleep_for(
          std::chrono::duration<double>(control_dt_sec_));
    }

    if (last_command_was_active) {
      sport_client_.StopMove();
    }
  }

  void ShutdownCommandLoop() {
    if (!command_loop_started_.exchange(false)) {
      return;
    }

    shutdown_requested_.store(true);
    if (command_thread_.joinable()) {
      command_thread_.join();
    }
  }

  unitree_go::msg::dds_::SportModeState_ state_{};
  unitree::robot::go2::SportClient sport_client_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
      suber_;

  std::ofstream highstate_logfile_;
  std::ofstream footpos_velocity_logfile_;

  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;

  std::atomic<bool> state_received_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> command_loop_started_{false};

  MotionCommand command_{};
  std::thread command_thread_;

  double px0_ = 0.0;
  double py0_ = 0.0;
  double yaw0_ = 0.0;
  double control_dt_sec_ = 0.005;
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

  sleep(1);
  custom.GetInitState();
  custom.StartCommandLoop();
  custom.RunInteractiveCli();

  return 0;
}
