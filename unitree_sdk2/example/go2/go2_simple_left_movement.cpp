/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <atomic>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <sstream>
#include <string>
#include <unistd.h>

// #define ROT_ERR 11.0
#define ROT_ERR 8
#define TIME_SLEEP 2.5

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

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

    highstate_logfile_.open(
        "Movement_HighState_pos_log.txt", std::ios::out | std::ios::app);
    highstate_logfile_.imbue(std::locale::classic());
    if (!highstate_logfile_.is_open()) {
      std::cerr << "Failed to open Movement_HighState_pos_log.txt" << std::endl;
    }

    footpos_velocity_logfile_.open("Foot_log.txt", std::ios::out | std::ios::app);
    footpos_velocity_logfile_.imbue(std::locale::classic());
    if (!footpos_velocity_logfile_.is_open()) {
      std::cerr << "Failed to open Foot_log.txt" << std::endl;
    }
  }

  ~Custom() {
    StopMotion();

    if (highstate_logfile_.is_open()) {
      highstate_logfile_.close();
    }

    if (footpos_velocity_logfile_.is_open()) {
      footpos_velocity_logfile_.close();
    }
  }

  void RobotControl() {
    ct_ += dt_;

    MotionCommand command_snapshot;
    bool command_expired = false;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (command_.active && now >= command_.end_time) {
        command_ = {};
        command_expired = true;
      }
      command_snapshot = command_;
    }

    if (command_expired) {
      sport_client_.StopMove();
      last_move_sent_.store(false);
      std::cout << "\n1-second command finished. Motion stopped." << std::endl;
    } else if (command_snapshot.active) {
      sport_client_.Move(
          command_snapshot.vx, command_snapshot.vy, command_snapshot.vyaw);
      last_move_sent_.store(true);
    } else if (last_move_sent_.exchange(false)) {
      sport_client_.StopMove();
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

    sport_client_.BalanceStand();
    sleep(5);
    sport_client_.StaticWalk();

    std::cout << "Mode of the Go2 is: " << GetMode() << std::endl;
  }

  void RunInteractiveCli() {
    PrintHelp();

    std::string line;
    std::cout << "\nDo you want to continue?: "
                << std::flush;
    if (!std::getline(std::cin, line)) {
      std::cout << "\nInput stream closed. Exiting." << std::endl;
      // break;
    }

    const char* path = "controls.txt";

    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "ERROR: cannot open %s\n", path);
    }

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

        double err = ROT_ERR * 3.1417/180;
        if(vtheta > 0)
        {
          vtheta += err;
        }
        else
        {
          if(vtheta < 0)
          {
            vtheta -= err;
          } 
        }



        SetMotion(vx, vy, vtheta);

        // std::this_thread::sleep_for(std::chrono::seconds(TIME_SLEEP));
        sleep(TIME_SLEEP);
    }

    // Stop the robot
    std::printf("[done]   ");
    SetMotion(0.0, 0.0, 0.0);

    // std::string jinput;
    // std::cout << "Press Enter to jump, or type n to cancel: ";
    // std::getline(std::cin, jinput);

    // if (jinput != "n" && jinput != "N") {
    //     sport_client_.FrontJump();
    // }

    // while (true) {
    //   std::cout << "\nEnter vx vy vyaw for a 1-second move, or type stop/status/quit: "
    //             << std::flush;
    //   if (!std::getline(std::cin, line)) {
    //     std::cout << "\nInput stream closed. Exiting." << std::endl;
    //     break;
    //   }

    //   if (line == "quit" || line == "q" || line == "exit") {
    //     StopMotion();
    //     break;
    //   }

    //   if (line == "stop") {
    //     StopMotion();
    //     continue;
    //   }

    //   if (line == "status") {
    //     PrintStatus();
    //     continue;
    //   }

    //   if (line == "help" || line == "h" || line == "?") {
    //     PrintHelp();
    //     continue;
    //   }

    //   std::istringstream iss(line);
    //   double vx = 0.0;
    //   double vy = 0.0;
    //   double vyaw = 0.0;
    //   std::string extra;
    //   if (!(iss >> vx >> vy >> vyaw) || (iss >> extra)) {
    //     std::cout << "Invalid input. Example: 0.2 0.0 0.1" << std::endl;
    //     continue;
    //   }

    //   if (line == "condition"){
    //   std::cout<< "Entering the Block" <<std::endl;
    //   SetMotion(0.000000, 0.000000, 0.392699);
    //   sleep(1);
    //   SetMotion(0.050000, 0.000000, 0.000000);
    //   sleep(1);
    //   SetMotion(0.000000, 0.000000, 0.392699);
    //   sleep(1);
    //   SetMotion(0.050000, 0.000000, 0.000000);
    //   sleep(1);
    //   SetMotion(0.050000, 0.000000, 0.000000);
    //   sleep(1);
    //   SetMotion(0.050000, 0.000000, 0.000000);
    //   sleep(1);
    //   SetMotion(0.050000, 0.000000, 0.000000);
    //   sleep(1);

    //   }

    //   //SetMotion(vx, vy, vyaw);

      
    // }
  }

  float GetDtSeconds() const { return dt_; }

 private:
  struct MotionCommand {
    bool active = false;
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
    std::chrono::steady_clock::time_point end_time{};
  };

  void SetMotion(double vx, double vy, double vyaw) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_.active = true;
      command_.vx = vx;
      command_.vy = vy;
      command_.vyaw = vyaw;
      command_.end_time =
          std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(command_duration_sec_));
    }

    std::cout << "1-second command started: vx=" << vx << ", vy=" << vy
              << ", vyaw=" << vyaw << std::endl;
  }

  void StopMotion() {
    bool was_active = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      was_active = command_.active;
      command_ = {};
    }

    if (was_active || last_move_sent_.exchange(false)) {
      sport_client_.StopMove();
      std::cout << "Motion stopped." << std::endl;
    }
  }

  int GetMode() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return static_cast<int>(state_.mode());
  }

  void PrintStatus() const {
    MotionCommand command_snapshot;
    unitree_go::msg::dds_::SportModeState_ state_snapshot;
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      command_snapshot = command_;
    }
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      state_snapshot = state_;
    }

    std::cout << std::fixed << std::setprecision(3)
              << "Pose: x=" << state_snapshot.position()[0]
              << ", y=" << state_snapshot.position()[1]
              << ", z=" << state_snapshot.position()[2]
              << ", yaw=" << state_snapshot.imu_state().rpy()[2]
              << " | Velocity: vx=" << state_snapshot.velocity()[0]
              << ", vy=" << state_snapshot.velocity()[1]
              << ", vz=" << state_snapshot.velocity()[2]
              << " | Mode=" << static_cast<int>(state_snapshot.mode())
              << std::endl;

    if (command_snapshot.active) {
      const auto now = std::chrono::steady_clock::now();
      const double remaining_sec = std::max(
          0.0,
          std::chrono::duration<double>(command_snapshot.end_time - now).count());
      std::cout << "Active streamed command: vx=" << command_snapshot.vx
                << ", vy=" << command_snapshot.vy
                << ", vyaw=" << command_snapshot.vyaw
                << " | remaining=" << remaining_sec << " s" << std::endl;
    } else {
      std::cout << "Active streamed command: none" << std::endl;
    }
  }

  void PrintHelp() const {
    std::cout << "\nRuntime motion control\n"
              << "  Input three numbers: <vx> <vy> <vyaw>  (runs for 1 second)\n"
              << "  stop   Stop the current motion\n"
              << "  status Show current pose and active command\n"
              << "  quit   Stop motion and exit\n"
              << "  help   Show this message\n"
              << std::endl;
  }

  void HighStateHandler(const void* message) {
    unitree_go::msg::dds_::SportModeState_ snapshot =
        *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_ = snapshot;
    }
    state_received_.store(true);

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    localtime_r(&now_time, &local_tm);
    std::ostringstream time_stream;
    time_stream << "[" << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S") << "]";

    if (highstate_logfile_.is_open()) {
      highstate_logfile_ << time_stream.str() << " Position: "
                         << snapshot.position()[0] << ", "
                         << snapshot.position()[1] << ", "
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
                         << static_cast<int>(snapshot.mode()) << std::endl;
      highstate_logfile_ << std::flush;
    }

    if (footpos_velocity_logfile_.is_open()) {
      footpos_velocity_logfile_ << time_stream.str() << " Velocity: "
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

  unitree_go::msg::dds_::SportModeState_ state_{};
  unitree::robot::go2::SportClient sport_client_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
      suber_;

  std::ofstream highstate_logfile_;
  std::ofstream footpos_velocity_logfile_;

  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;

  std::atomic<bool> state_received_{false};
  std::atomic<bool> last_move_sent_{false};

  MotionCommand command_{};

  double px0_ = 0.0;
  double py0_ = 0.0;
  double yaw0_ = 0.0;
  double ct_ = 0.0;
  float dt_ = 0.02f;
  double command_duration_sec_ = 1.0;
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

  ThreadPtr thread_ptr = CreateRecurrentThread(
      static_cast<uint32_t>(custom.GetDtSeconds() * 1000000.0f),
      std::bind(&Custom::RobotControl, &custom));

  custom.RunInteractiveCli();

  return 0;
}
