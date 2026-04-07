/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <fstream>
#include <locale>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <unistd.h>
// [2] Defines the topic name from which state feedback is subscribed.
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

enum test_mode
{
  /*---Basic motion---*/
  normal_stand,
  balance_stand,
  velocity_move,
  stand_down,
  stand_up,
  damp,
  recovery_stand,
  /*---Special motion ---*/
  sit,
  rise_sit,
  stop_move = 99
};


class Custom
{
public:
  Custom()
  {
    // [4] Sets command timeout and initializes the sport client.
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    obstacles_client.SetTimeout(5.0f);
    obstacles_client.Init();
    

    // [5] (a) Creates a subscriber to receive robot state on "rt/sportmodestate" topic

    // [5] (b) When a message is received, it calls the HighStateHandler method.
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);

    

    highstate_logfile_.open("Movement_HighState_pos_log.txt", std::ios::out | std::ios::app);
    //highstate_logfile_.imbue(std::locale("en_US.UTF-8"));
    highstate_logfile_.imbue(std::locale::classic());

    if (!highstate_logfile_.is_open())
    {
        std::cerr << "Failed to open log file!" << std::endl;
    }



    footpos_velocity_logfile_.open( "Foot_log.txt", std::ios::out | std::ios::app);
    //footpos_velocity_logfile_.imbue(std::locale("en_US.UTF-8"));
    footpos_velocity_logfile_.imbue(std::locale::classic());

    if (!footpos_velocity_logfile_.is_open())
    {
        std::cerr << "Failed to open log file!" << std::endl;
    }





  };

  std::ofstream highstate_logfile_;
  std::ofstream footpos_velocity_logfile_;

  ~Custom() 
  {
    if (highstate_logfile_.is_open())
        highstate_logfile_.close();

    if (footpos_velocity_logfile_.is_open())
       footpos_velocity_logfile_.close();
        
  }


  /// --- Controlling the Robot---///
  void RobotControl()
  {
    ct += dt;

    std::cout << "Elapsed time: " << ct << std::endl;

    if (ct <= 6.0)
    {
      std::cout << "Moving forward with obstacle avoidance... " << ct << std::endl;

      if (obstacle_avoidance_enabled)
      {
        obstacles_client.Move(1.0, 0.0, 0.0);
      }
    }
    else
    {
      if (obstacle_avoidance_enabled)
      {
        std::cout << "Stopping motion and disabling obstacle avoidance..." << std::endl;

        sport_client.StopMove();  // stop robot motion first
        obstacles_client.UseRemoteCommandFromApi(false);
        obstacles_client.SwitchSet(false);

        obstacle_avoidance_enabled = false;
      }
    }
  };
  // Get initial position
  void GetInitState()
  {
    // [6] Stores the robot’s initial position and yaw (orientation)
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;

    /// have to add BalanceStand functionality and Static Walk Functionality
    // sport_client.BalanceStand();
    // std::cout << "Mode of the Go2 is: " << static_cast<int>(state.mode()) << std::endl;
    // sleep(5);
    sport_client.StaticWalk();
    //sport_client.FreeWalk();
    std::cout << "Mode of the Go2 is: " << static_cast<int>(state.mode()) << std::endl;


    obstacles_client.SwitchSet(true);
    usleep(1000000);
    obstacles_client.UseRemoteCommandFromApi(true);
    obstacle_avoidance_enabled = true;

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
                          << static_cast<int>(state.mode()) << std::endl;

        highstate_logfile_ << std::flush;

    }

    //to fp = state.foot_position_body();


    if (footpos_velocity_logfile_.is_open()){


        footpos_velocity_logfile_ << time_stream.str() << "Velocity:"
                                  << "Velocity along x-axis:" << state.velocity()[0] << ","
                                  << "Velocity along y-axis:" << state.velocity()[1] << ","
                                  << "Velocity along z-axis:" << state.velocity()[2] << ","
                                  << "FR(x,y,z)=" << state.foot_position_body()[0] << "," << state.foot_position_body()[1] << "," << state.foot_position_body()[2]<< ","
                                  << "FL(x,y,z)=" << state.foot_position_body()[3] << "," << state.foot_position_body()[4] << "," << state.foot_position_body()[5]<< ","
                                  << "RR(x,y,z)=" << state.foot_position_body()[6] << "," << state.foot_position_body()[7]<< "," << state.foot_position_body()[8]<< ","
                                  << "RL(x,y,z)=" << state.foot_position_body()[9] << "," << state.foot_position_body()[10]<< "," << state.foot_position_body()[11]
                                  << std::endl;

        
        footpos_velocity_logfile_ << std::flush;



    }

  };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  unitree::robot::go2::ObstaclesAvoidClient obstacles_client;
  bool obstacle_avoidance_enabled = false;

  double px0, py0, yaw0; // 初始状态的位置和偏航
  double ct = 0;         // 运行时间
  int flag = 0;          // 特殊动作执行标志
  float dt = 0.005;      // 控制步长0.001~0.01

  //custom addition to make the robot stop moving after 2 times of movement
  // int call_count = 0;
  // int TEST_MODE = velocity_move;
};

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  Custom custom;

  std::locale::global(std::locale::classic());
  std::cout.imbue(std::locale::classic());
  std::cerr.imbue(std::locale::classic());

  sleep(1);

  custom.GetInitState();
  
  // crete a recurrent thread that runs every  5 milliseconds and calls the RobotControl method
  unitree::common::ThreadPtr threadPtr =
      unitree::common::CreateRecurrentThread(
          custom.dt * 1000000,
          std::bind(&Custom::RobotControl, &custom));

  //Because RobotControl() is a method inside the Custom class, the thread needs to know 
  //which object it belongs to.

  while (1)
  {
    sleep(10);
  }

  return 0;
}
