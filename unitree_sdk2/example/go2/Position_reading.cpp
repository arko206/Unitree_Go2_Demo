/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
/// [1] These include the Unitree Go2 SDK headers:

    //-- (a) sport_client handles motion commands

    //-- (b) channel_subscriber reads data like robot position, IMU

    //-- (c) SportModeState_ is the DDS message type for robot state

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>  
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
#include <thread>
#include <chrono>



// [2] Defines the topic name from which state feedback is subscribed.
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;


bool stopped = false;

void sigint_handler(int sig)
{
  if (sig == SIGINT)
  {
    stopped = true;
  }
}


class Custom
{
public:
  Custom()
  {
    // [4] Sets command timeout and initializes the sport client.
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    

    // [5] (a) Creates a subscriber to receive robot state on "rt/sportmodestate" topic

    // [5] (b) When a message is received, it calls the HighStateHandler method.
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

    

    namespace fs = std::filesystem;
    fs::path log_dir = "./Interface_Reading_Control/First_Interface_Front_Jump";
    std::error_code ec;
    fs::create_directories(log_dir, ec);   // ok if it already exists

    fs::path pos  = log_dir / "First_Interface_Front_Jump_log.txt";
    fs::path point_path  = log_dir / "Interface_path_point_log.txt";




    highstate_logfile_.open(pos,  std::ios::out | std::ios::app);
    if (!highstate_logfile_.is_open()){
      std::cerr << "Failed to open " << pos
                << " (errno=" << errno << " - " << std::strerror(errno) << ")\n";
    }

    path_point_logfile_.open(point_path,  std::ios::out | std::ios::app);
    if (!path_point_logfile_.is_open()){
      std::cerr << "Failed to open " << point_path
                << " (errno=" << errno << " - " << std::strerror(errno) << ")\n";
    }




  };

  std::ofstream highstate_logfile_;
  std::ofstream path_point_logfile_;


  ~Custom() 
  {
    if (highstate_logfile_.is_open())
        highstate_logfile_.close();

    if (path_point_logfile_.is_open())
        path_point_logfile_.close();

 
  }

 

  // Get initial position
  void GetInitState()
  {
    // [6] Stores the robot’s initial position and yaw (orientation)
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;

  };

  // [7] Called when a new message is received
  // Updates state and prints position and orientation (rpy)
  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::localtime(&now_time);
    std::ostringstream time_stream;
    time_stream << "[" << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << "]";


    if (highstate_logfile_.is_open())
    {
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
                          << state.imu_state().quaternion()[3] << "| Mode: "
                          << static_cast<int>(state.mode()) << "| Velocity:"
                          << "Velocity along x-axis:" << state.velocity()[0] << ","
                          << "Velocity along y-axis:" << state.velocity()[1] << ","
                          << "Velocity along z-axis:" << state.velocity()[2] << " | Gait Type:"
                          << static_cast<int>(state.gait_type()) << "| First Path Point:"
                          << state.path_point()[0].x() << ","
                          << state.path_point()[0].y() << ","
                          << state.path_point()[0].yaw() << std::endl;

                  
            

        highstate_logfile_ << std::flush;

    }


    std::cout << "Mode of the Go2 is: " 
          << static_cast<int>(state.mode()) 
          << " Position: "
          << state.position()[0] << ", "
          << state.position()[1] << ", "
          << state.position()[2]
          << std::endl;








    if (path_point_logfile_.is_open())
    {
        //const auto& path_points = state.path_point();

        
                
        path_point_logfile_ << time_stream.str() << "\n";

        for (int i = 0; i < state.path_point().size(); ++i) {
            path_point_logfile_ << "  [i=" << i
                                << ", t=" << state.path_point()[i].t_from_start()
                                << ", x=" << state.path_point()[i].x()
                                << ", y=" << state.path_point()[i].y()
                                << ", yaw=" << state.path_point()[i].yaw()
                                << ", vx=" << state.path_point()[i].vx()
                                << ", vy=" << state.path_point()[i].vy()
                                << ", vyaw=" << state.path_point()[i].vyaw()
                                << "]\n";
        }
        path_point_logfile_ << std::flush;

      }



    }




   







  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
  unitree::robot::go2::RobotStateClient rsc;

  double px0, py0, yaw0; // 初始状态的位置和偏航
  double ct = 0;         // 运行时间
  int flag = 0;          // 特殊动作执行标志
  float dt = 0.005;      // 控制步长0.001~0.01







};

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
 // Initializes communication and prepares the robot
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Custom custom;

  std::locale::global(std::locale::classic());
  std::cout.imbue(std::locale::classic());
  std::cerr.imbue(std::locale::classic());

  //sleep(1); // Wait for 1 second to obtain a stable state
  // Create a signal object to capture the signal for program exit
  signal(SIGINT, sigint_handler);



  custom.GetInitState(); // Get initial position

  // Keep program alive until Ctrl+C
  while (!stopped) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }



  
}
