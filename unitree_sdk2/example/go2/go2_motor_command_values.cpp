#include <iostream>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/MotorState_.hpp>
#include <unitree/idl/go2/BmsState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include <fstream> 
#include <chrono>
#include <iomanip>
#include <ctime>


using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"



// [3] Encapsulates logic for:

// (a) Subscribing to robot motor state

// (b) Periodically logging motor data
class Custom
{
public:
    explicit Custom() {}

    ~Custom() {

        if (logfile_.is_open()){
            logfile_.close();
        }
        
    }
    void Init();

private:
    void LowCommandMessageHandler(const void *messages);
    void ReportMotorCommand();
    std::ofstream logfile_;

    std::vector<std::string> motor_names = {
        "FR_hip", "FR_thigh", "FR_calf",
        "FL_hip", "FL_thigh", "FL_calf",
        "RR_hip", "RR_thigh", "RR_calf",
        "RL_hip", "RL_thigh", "RL_calf"
    };

private:
    
    unitree_go::msg::dds_::LowCmd_ low_cmd_{};  

    std::array<::unitree_go::msg::dds_::MotorCmd_, 20> motor_cmd_{};

    
    // [6]  DDS subscriber for LowState_
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_subscriber_;

    /*MotorState logging thread*/
    // [7] Thread that periodically logs motor data.
    ThreadPtr MotorCmdReportThreadPtr;
};

void Custom::Init()
{
    /*create subscriber*/

    lowcmd_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));

    
    lowcmd_subscriber_->InitChannel(std::bind(&Custom::LowCommandMessageHandler, this, std::placeholders::_1), 1);

    /*motor state logging thread*/
    MotorCmdReportThreadPtr = CreateRecurrentThreadEx("log_motor_cmd", UT_CPU_ID_NONE, 500000, &Custom::ReportMotorCommand, this);


    //setting up the File thread
    //logfile_.open("motor_log_new.txt", std::ios::out | std::ios::app);  // Append mode
    logfile_.open("Command_Forward_motor.txt", std::ios::out | std::ios::app);
    logfile_.imbue(std::locale("en_US.UTF-8"));

    if (!logfile_.is_open())
    {
        std::cerr << "Failed to open log file!" << std::endl;
    }

}

// [12] (a) Called every time new LowState_ data arrives.
// [12] (b) Copies the message into the low_state_ object.
void Custom::LowCommandMessageHandler(const void *message)
{
    low_cmd_ = *(unitree_go::msg::dds_::LowCmd_ *)message;
    //std::cout << "Message from Low Level Command is: " << low_state_ << std::endl;
}

void Custom::ReportMotorCommand()
{   
    // [13] Copies the 20 motor states from low_state_ into motor_state_[].
    for (int i = 0; i < 20; i++)
    {
        motor_cmd_[i] = low_cmd_.motor_cmd()[i]; // Get MotorCommand from LowCommand

    }

    //auto now = std::chrono::system_clock::now();

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::localtime(&now_time);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream time_stream;
    time_stream << "[" << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S")
                << "." << std::setw(3) << std::setfill('0') << ms.count() << "]";



    for (int i = 0; i < 12; i++)
    {
        std::ostringstream log_stream;

        log_stream << time_stream.str() << " Report for Motor Commands[" << i << " - " << motor_names[i] << "], "
                   << "Mode: " << motor_cmd_[i].mode() << ", "
                   << "Joint Target Position: " << motor_cmd_[i].q() << ", "
                   << "Joint Target Speed: " << motor_cmd_[i].dq() << ", "
                   << "Joint Target Torque: " << motor_cmd_[i].tau() << ", "
                   << "Joint Stiffness Coefficient: " << motor_cmd_[i].kp() << ", "
                   << "Joint Damping Coefficent: " << motor_cmd_[i].kd();

        std::cout << log_stream.str() << std::endl;

        if (logfile_.is_open())
            logfile_ << log_stream.str() << std::endl;
    }

}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}