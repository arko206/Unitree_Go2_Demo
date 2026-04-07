/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <fstream>
#include <locale>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <map>
#include <vector>
#include <filesystem>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <mutex>
#include <limits>
#include <bitset>
#include <unistd.h>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

#define TOPIC_HIGHSTATE   "rt/sportmodestate"
#define TOPIC_RANGE_INFO  "rt/utlidar/range_info"

using namespace unitree::common;

class Custom
{
public:
  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // Subscribe high-level robot state
    suber.reset(
        new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(
        std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

    // Subscribe LiDAR processed range info
    range_suber.reset(
        new unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>(TOPIC_RANGE_INFO));
    range_suber->InitChannel(
        std::bind(&Custom::RangeInfoHandler, this, std::placeholders::_1), 1);

    namespace fs = std::filesystem;
    fs::path log_dir = "./Single_Movement_Go2/Fifth_Move";
    std::error_code ec;
    fs::create_directories(log_dir, ec);

    fs::path pos   = log_dir / "Demo_Move_pos_log.txt";
    fs::path foot  = log_dir / "Demo_Move_Foot_log.txt";
    fs::path range = log_dir / "Demo_Move_range_log.txt";

    highstate_logfile_.open(pos, std::ios::out | std::ios::app);
    if (!highstate_logfile_.is_open()) {
      std::cerr << "Failed to open " << pos
                << " (errno=" << errno << " - " << std::strerror(errno) << ")\n";
    }

    footpos_velocity_logfile_.open(foot, std::ios::out | std::ios::app);
    if (!footpos_velocity_logfile_.is_open()) {
      std::cerr << "Failed to open " << foot
                << " (errno=" << errno << " - " << std::strerror(errno) << ")\n";
    }

    range_logfile_.open(range, std::ios::out | std::ios::app);
    if (!range_logfile_.is_open()) {
      std::cerr << "Failed to open " << range
                << " (errno=" << errno << " - " << std::strerror(errno) << ")\n";
    }
  }

  ~Custom()
  {
    if (highstate_logfile_.is_open()) highstate_logfile_.close();
    if (footpos_velocity_logfile_.is_open()) footpos_velocity_logfile_.close();
    if (range_logfile_.is_open()) range_logfile_.close();
  }

  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];

    std::cout << "initial position: x0: " << px0
              << ", y0: " << py0
              << ", yaw0: " << yaw0 << std::endl;

    sport_client.StaticWalk();
    std::cout << "Mode of the Go2 is: " << static_cast<int>(state.mode()) << std::endl;

    // Give the robot a moment to settle
    usleep(500000);
  }

  void RobotControl()
  {
    ct += dt;

    // Copy shared LiDAR values under mutex
    double front_local, left_local, right_local;
    bool lidar_valid_local;
    {
      std::lock_guard<std::mutex> lock(range_mutex_);
      front_local = front_range_;
      left_local  = left_range_;
      right_local = right_range_;
      lidar_valid_local = lidar_valid_;
    }

    std::cout << "Elapsed time: " << ct
              << " | front=" << front_local
              << " left=" << left_local
              << " right=" << right_local
              << std::endl;

    // If jump already triggered, do nothing more
    if (jump_triggered_) {
      return;
    }


    // Only evaluate if LiDAR data is valid
    if (!lidar_valid_local) {
      return;
    }
    
    // Start by moving forward
    if (ct <= 1.0) {
      if (!moving_started_) {
          moving_started_ = true;
          std::cout << "[INFO] Starting forward motion..." << std::endl;
      }
      sport_client.Move(0.5, 0.0, 0.0);
   }
   else{

      std::cout << "Did the robot move?" << moving_started_ << std::endl;
      std::cout << "Is the LiDAR data valid?" << lidar_valid_local << std::endl;
      std::cout << "Now checking the Jump condition blocks: " << std::endl;
      const double front_min = 0.50;
      const double front_max = 5.00;

      //commented out because I am currently focussing on front obstacle//
      // const double side_min  = 0.20;
      // const double side_max  = 10.00;

      bool front_ok = (front_local >= front_min && front_local <= front_max);
      // bool left_ok  = (left_local  >= side_min  && left_local  <= side_max);
      // bool right_ok = (right_local >= side_min  && right_local <= side_max);

    // if there are valid detections, increase the count. We require a few consecutive detections to trigger the jump to avoid noise.
      if (front_ok) {
        valid_detect_count_++;

        std::cout << "[DEBUG] Valid front detection count: " << valid_detect_count_ << std::endl;

        if (range_logfile_.is_open()) {
        range_logfile_
            << "Valid Detection Point is:" << ","
            << front_local << ","
            << left_local  << ","
            << right_local << "\n";
        range_logfile_ << std::flush;
       }
      } 

      // Require a few consecutive good detections to avoid noise
      const int required_consecutive = 3;

      if (valid_detect_count_ >= required_consecutive) {
        std::cout << "[INFO] Obstacle condition satisfied. Stopping and triggering FrontJump..."
                  << std::endl;

        std::cout << "jump..." << std::endl;
        jump_triggered_ = true;

        sport_client.Move(0.0, 0.0, 0.0); // stop before jumping
        usleep(1000000); 
        sport_client.StaticWalk(); // ensure stop command is processed
        usleep(5000000); // brief pause to ensure static walk is processed
        std::cout << "[INFO] Sending FrontJump command now..." << std::endl;
        sport_client.FrontJump();
        std::cout << "[INFO] FrontJump command sent." << std::endl;

        
      }


    }
  

    

  //   // Ignore inf or non-finite values
  //   //--- This part can be problematic, because front_local might be valid finite, but others might be infinite---//
  //   // if (!std::isfinite(front_local) ||
  //   //     !std::isfinite(left_local)  ||
  //   //     !std::isfinite(right_local)) {
  //   //   return;
  //   // }

  //   // -----------------------------
  //   // Threshold logic (meters)
  //   // Tune these carefully tomorrow
  //   // Example: obstacle ahead in a safe jump window
  //   // -----------------------------
  //   const double front_min = 0.10;
  //   const double front_max = 10.00;

  //   //commented out because I am currently focussing on front obstacle//
  //   // const double side_min  = 0.20;
  //   // const double side_max  = 10.00;

  //   bool front_ok = (front_local >= front_min && front_local <= front_max);
  //   // bool left_ok  = (left_local  >= side_min  && left_local  <= side_max);
  //   // bool right_ok = (right_local >= side_min  && right_local <= side_max);

  //  // if there are valid detections, increase the count. We require a few consecutive detections to trigger the jump to avoid noise.
  //   if (front_ok) {
  //     valid_detect_count_++;

  //     std::cout << "[DEBUG] Valid front detection count: " << valid_detect_count_ << std::endl;

  //     if (range_logfile_.is_open()) {
  //     range_logfile_
  //         << "Valid Detection Point is:" << ","
  //         << front_local << ","
  //         << left_local  << ","
  //         << right_local << "\n";
  //     range_logfile_ << std::flush;
  //    }
  //   } 

  //   // Require a few consecutive good detections to avoid noise
  //   const int required_consecutive = 2;

  //   if (valid_detect_count_ >= required_consecutive) {
  //     std::cout << "[INFO] Obstacle condition satisfied. Stopping and triggering FrontJump..."
  //               << std::endl;

  //     std::cout << "jump..." << std::endl;

  //     sport_client.Move(0.0, 0.0, 0.0); // stop before jumping
  //     usleep(300000); // brief pause to ensure stop command is processed
  //     sport_client.FrontJump();

  //     jump_triggered_ = true;
  //   }
  }

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::localtime(&now_time);
    std::ostringstream time_stream;
    time_stream << "[" << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "]";

    if (highstate_logfile_.is_open()) {
      highstate_logfile_ << time_stream.str() << " Position: "
                         << state.position()[0] << ", "
                         << state.position()[1] << ", "
                         << state.position()[2] << " | IMU RPY: "
                         << state.imu_state().rpy()[0] << ", "
                         << state.imu_state().rpy()[1] << ", "
                         << state.imu_state().rpy()[2] << " | Angular Vel: "
                         << state.imu_state().gyroscope()[0] << ", "
                         << state.imu_state().gyroscope()[1] << ", "
                         << state.imu_state().gyroscope()[2] << " | Acceleration: "
                         << state.imu_state().accelerometer()[0] << ", "
                         << state.imu_state().accelerometer()[1] << ", "
                         << state.imu_state().accelerometer()[2] << " | Quaternion: "
                         << state.imu_state().quaternion()[0] << ","
                         << state.imu_state().quaternion()[1] << ","
                         << state.imu_state().quaternion()[2] << ","
                         << state.imu_state().quaternion()[3] << " | Mode: "
                         << std::to_string(state.mode()) << " | BitMode: "
                         << std::bitset<8>(state.mode()).to_string() << " | Velocity: "
                         << state.velocity()[0] << ","
                         << state.velocity()[1] << ","
                         << state.velocity()[2]
                         << std::endl;
      highstate_logfile_ << std::flush;
    }

    if (footpos_velocity_logfile_.is_open()) {
      footpos_velocity_logfile_ << time_stream.str() << " Velocity:"
                                << " x=" << state.velocity()[0] << ","
                                << " y=" << state.velocity()[1] << ","
                                << " z=" << state.velocity()[2] << ","
                                << " FR(x,y,z)=" << state.foot_position_body()[0] << ","
                                                 << state.foot_position_body()[1] << ","
                                                 << state.foot_position_body()[2] << ","
                                << " FL(x,y,z)=" << state.foot_position_body()[3] << ","
                                                 << state.foot_position_body()[4] << ","
                                                 << state.foot_position_body()[5] << ","
                                << " RR(x,y,z)=" << state.foot_position_body()[6] << ","
                                                 << state.foot_position_body()[7] << ","
                                                 << state.foot_position_body()[8] << ","
                                << " RL(x,y,z)=" << state.foot_position_body()[9] << ","
                                                 << state.foot_position_body()[10] << ","
                                                 << state.foot_position_body()[11]
                                << std::endl;
      footpos_velocity_logfile_ << std::flush;
    }
  }

  void RangeInfoHandler(const void *message)
  {
    const geometry_msgs::msg::dds_::PointStamped_ *range_msg =
        (const geometry_msgs::msg::dds_::PointStamped_ *)message;

    double front = range_msg->point().x();
    double left  = range_msg->point().y();
    double right = range_msg->point().z();

    {
      std::lock_guard<std::mutex> lock(range_mutex_);
      front_range_ = front;
      left_range_  = left;
      right_range_ = right;
      lidar_valid_ = true;
    }

    if (range_logfile_.is_open()) {
      range_logfile_
          << range_msg->header().stamp().sec() << "."
          << range_msg->header().stamp().nanosec() << ","
          << range_msg->header().frame_id() << ","
          << front << ","
          << left  << ","
          << right << "\n";
      range_logfile_ << std::flush;
    }
  }

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;

  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
  unitree::robot::ChannelSubscriberPtr<geometry_msgs::msg::dds_::PointStamped_> range_suber;

  std::ofstream highstate_logfile_;
  std::ofstream footpos_velocity_logfile_;
  std::ofstream range_logfile_;

  // Shared LiDAR range data
  std::mutex range_mutex_;
  double front_range_ = std::numeric_limits<double>::infinity();
  double left_range_  = std::numeric_limits<double>::infinity();
  double right_range_ = std::numeric_limits<double>::infinity();
  bool lidar_valid_ = false;

  // Control state
  bool moving_started_ = false;
  bool jump_triggered_ = false;
  int valid_detect_count_ = 0;

  double px0 = 0.0, py0 = 0.0, yaw0 = 0.0;
  double ct = 0.0;
  float dt = 0.005;
};

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    return -1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  Custom custom;

  std::locale::global(std::locale::classic());
  std::cout.imbue(std::locale::classic());
  std::cerr.imbue(std::locale::classic());

  sleep(1);
  custom.GetInitState();

  // Run RobotControl periodically every 5 ms
  unitree::common::ThreadPtr threadPtr =
      unitree::common::CreateRecurrentThread(
          custom.dt * 1000000,
          std::bind(&Custom::RobotControl, &custom));

  while (true) {
    sleep(10);
  }

  return 0;
}