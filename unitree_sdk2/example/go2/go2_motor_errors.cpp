#include <iostream>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/MotorState_.hpp>
#include <unitree/idl/go2/BmsState_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <fstream> 
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <clocale> 


using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWSTATE "rt/lowstate"

class GetDetailMotorErrors
{
private:
    struct MotorErrorState
    {
        std::string error_type;
        short int state;
    };

    std::vector<MotorErrorState> motor_errors = {
        {error_type : "Over current", state : 0},
        {error_type : "Over voltage", state : 0},
        {error_type : "Driver overheat", state : 0},
        {error_type : "Motor bus under voltage", state : 0},
        {error_type : "Winding overheat", state : 0},
        {error_type : "Encoder abnormal", state : 0},
        {error_type : "Reserved, Not used", state : 0},
        {error_type : "Reserved, Not used", state : 0},
        {error_type : "Communication interrupted", state : 0},
    };

public:
    GetDetailMotorErrors(){};
    ~GetDetailMotorErrors(){};
    void ReportErrors(std::string &motor_name, int error_code, const std::string &time_str, std::ofstream &log_stream)
    {
        for (int i = 0; i < motor_errors.size(); i++)
        {   
            // Each bit of the error_code corresponds to a motor error type
            // For example, error_code = 6, i. e. error_code = 0b000000110, means Over voltage and Driver overheat.
            motor_errors[i].state = (error_code & (1 << i)) >> i;



            //std::string error_message = time_str + " Motor Error: " + motor_errors[i].error_type;
            std::string error_message = time_str + " Motor [" + motor_name + "] Error: " + motor_errors[i].error_type + "Error State: " + std::to_string(motor_errors[i].state);
            std::cout << error_message << std::endl;

            if (log_stream.is_open())
            {
                log_stream << error_message << std::endl;
                log_stream.flush();
            }


            if (motor_errors[i].state > 0)
            {
                std::cout << motor_errors[i].error_type << ", ";
            }
        }
        std::cout << std::endl;
    };
};


// [3] Encapsulates logic for:

// (a) Subscribing to robot motor state

// (b) Periodically logging motor data
class Custom
{
public:
    explicit Custom() {}

    ~Custom() {
        
        //---- It saves the Motor Error Logs ----
        if (logfile_.is_open()){
            logfile_.close();
        }
        
        //-- It saves the temperature of the Go2 Robot Logs ----
        if (templogfile_.is_open()){
            templogfile_.close();
        }

        //-- It saves the Motor Error of the Fo2 Robot Logs--
        if(motor_error_logfile_.is_open()){
            motor_error_logfile_.close();
        }
        
        // It saves the battery management system temperature Logs---
        if(battery_temp_logfile_.is_open()){
            battery_temp_logfile_.close();
        }
        
        
    }
    void Init();

private:
    void LowStateMessageHandler(const void *messages);
    void ReportMotorState();
    std::ofstream logfile_;
    std::ofstream templogfile_;
    std::ofstream motor_error_logfile_;
    std::ofstream battery_temp_logfile_;



    std::vector<std::string> motor_names = {
        "FR_hip", "FR_thigh", "FR_calf",
        "FL_hip", "FL_thigh", "FL_calf",
        "RR_hip", "RR_thigh", "RR_calf",
        "RL_hip", "RL_thigh", "RL_calf"
    };

private:
    // [4] (a) Declares a variable named low_state_ of type LowState_ (from Unitree DDS message definition).
    // [4] (b) The {} at the end is uniform initialization, which value-initializes the struct (zeroes out all members).
    unitree_go::msg::dds_::LowState_ low_state_{};

    // [5] (a) Declares a fixed-size array motor_state_ of 20 elements of type MotorState_
    // [5] (b) Again, {} zero-initializes all 20 MotorState_ elements.
    std::array<::unitree_go::msg::dds_::MotorState_, 20> motor_state_{};

    
    // [6]  DDS subscriber for LowState_
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;

    /*MotorState logging thread*/
    // [7] Thread that periodically logs motor data.
    ThreadPtr MotorStateReportThreadPtr;

    // [8] Instance of the class that decodes and reports motor errors.
    GetDetailMotorErrors motor_errors_;

    ::unitree_go::msg::dds_::BmsState_ bms_state_;

    ::unitree_go::msg::dds_::IMUState_ imu_state_;

    int temperature_ntc1_;
    int temperature_ntc2_;
    float power_v_;
    float power_a_;
    float adc_reel_;
};

void Custom::Init()
{
    /*create subscriber*/

    // [9] (a) lowstate_subscriber_ is a smart pointer (ChannelSubscriberPtr) holding a subscriber to the DDS topic.
    // [9] (b) ChannelSubscriber<LowState_> is a template class that knows how to receive DDS messages of type LowState_.
    // [9] (c) TOPIC_LOWSTATE is the string "rt/lowstate" — the DDS topic name where the robot broadcasts LowState_ messages.
    lowstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));

    // [10] (a) InitChannel(...) tells the subscriber what to do every time a message is received.
    // [10] (b) It passes a callback function — i.e., a function to be automatically called when a new message arrives.
    lowstate_subscriber_->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*motor state logging thread*/
    // [11] (a) Spawns a thread named log_motor_state.
    // [11] (b) It runs ReportMotorState() every 500000 µs = 0.5 seconds.
    MotorStateReportThreadPtr = CreateRecurrentThreadEx("log_motor_state", UT_CPU_ID_NONE, 500000, &Custom::ReportMotorState, this);


    //setting up the File thread for recording motor errors
    //logfile_.open("motor_log_new.txt", std::ios::out | std::ios::app);  // Append mode
    logfile_.open("New_Forward_motor_log.txt", std::ios::out | std::ios::app);
    //logfile_.imbue(std::locale("en_US.UTF-8"));
    //logfile_.imbue(std::locale::classic());

    if (!logfile_.is_open())
    {
        std::cerr << "Failed to open log file!" << std::endl;
    }


    //for logging temperature of the motherboard
    templogfile_.open("New_Temperature_log.txt", std::ios::out | std::ios::app);
    //templogfile_.imbue(std::locale::classic());

    if (!templogfile_.is_open())
    {
        std::cerr << "Failed to open Temperature log file!" << std::endl;
    }


    motor_error_logfile_.open("New_Motor_Error_Log.txt", std::ios::out | std::ios::app);
    //motor_error_logfile_.imbue(std::locale::classic());

    if (!motor_error_logfile_.is_open())
    {
        std::cerr << "Failed to open Motor Error log file!" << std::endl;
    }


    battery_temp_logfile_.open("New_Battery_Temp_Log.txt", std::ios::out | std::ios::app);
    //battery_temp_logfile_.imbue(std::locale::classic());

    if (!battery_temp_logfile_.is_open())
    {
        std::cerr << "Failed to open Battery Temperature log file!" << std::endl;
    }



}

// [12] (a) Called every time new LowState_ data arrives.
// [12] (b) Copies the message into the low_state_ object.
void Custom::LowStateMessageHandler(const void *message)
{
    low_state_ = *(unitree_go::msg::dds_::LowState_ *)message;
    //std::cout << "Message from Low Level Command is: " << low_state_ << std::endl;
}

void Custom::ReportMotorState()
{   
    // [13] Copies the 20 motor states from low_state_ into motor_state_[].
    for (int i = 0; i < 20; i++)
    {
        motor_state_[i] = low_state_.motor_state()[i]; // Get MotorState from LowState

    }

    //retreiving the values of battery state
     bms_state_ = low_state_.bms_state();

     //retreiving the imu state of the robot
     imu_state_ = low_state_.imu_state();

     //retreiving the temperature of the motherboard
     temperature_ntc1_ = static_cast<int>(low_state_.temperature_ntc1());
     temperature_ntc2_ = static_cast<int>(low_state_.temperature_ntc2());
     power_v_ = low_state_.power_v();
     power_a_ = low_state_.power_a();
     adc_reel_ = low_state_.adc_reel();


    // Getting the Time Stream-----
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm *now_tm = std::localtime(&now_time);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream time_stream;
    time_stream << "[" << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S")
                << "." << std::setw(3) << std::setfill('0') << ms.count() << "]";

    std::ostringstream temp_stream;
    
    // Saving the Temperature of the Unitree Go2 Dog-----
    temp_stream << "Time of recording temperature is: " << time_stream.str() << ","
                << "Temperature Value of the Motherboard is: " << temperature_ntc1_<<","
                << "Automatic Charging temperature of the Motherboard is: " << temperature_ntc2_<<","
                << "Voltage value in the Motherboard is: " << power_v_ << ","
                << "Current value in the Motherboard is: " << power_a_ << ","
                << "Winder Current is: " << adc_reel_ ;

    std::cout << temp_stream.str() << std::endl;

    if (templogfile_.is_open()){
        templogfile_ << temp_stream.str() << std::endl;
        templogfile_.flush();
    }



    std::ostringstream battery_stream;
    
    /// Saving the Information about the Battery Management System for Unitree Go2------
    battery_stream << "Battery Report for time is:"<<  time_stream.str() << ","
               << "High Version of the Battery is:" << bms_state_.version_high() << ","
               << "Low Version of the Battery is:"  << bms_state_.version_low() <<","
               << "Level of the Battery is:"  << bms_state_.soc() <<","
               << "Charging and Discharging information is:"  << bms_state_.current() <<","
               << "Number of Charging Cycles is:"  << bms_state_.cycle() <<","
               << "Temperature of the First NTC inside the battery is:"  << static_cast<int>(bms_state_.bq_ntc()[0]) <<","
               << "Temperature of the Second NTC inside the battery is:"  << static_cast<int>(bms_state_.bq_ntc()[1]) <<","

               << "Temperature of the RES is:"  << static_cast<int>(bms_state_.mcu_ntc()[0]) <<","
               << "Temperature of the MOS is:"  << static_cast<int>(bms_state_.mcu_ntc()[1]) <<","

               << "First Cell Voltage is:"  << bms_state_.cell_vol()[0] <<","
               << "Second Cell Voltage is:"  << bms_state_.cell_vol()[1] <<","
                << "Third Cell Voltage is:"  << bms_state_.cell_vol()[2] <<","
                << "Fourth Cell Voltage is:"  << bms_state_.cell_vol()[3] <<","
                << "Fifth Cell Voltage is:"  << bms_state_.cell_vol()[4] <<","
                << "Sixth Cell Voltage is:"  << bms_state_.cell_vol()[5] <<","
                << "Seventh Cell Voltage is:"  << bms_state_.cell_vol()[6] <<","
                << "Eigth Cell Voltage is:"  << bms_state_.cell_vol()[7] <<","
                << "Nineth Cell Voltage is:"  << bms_state_.cell_vol()[8] <<","
                << "Tenth Cell Voltage is:"  << bms_state_.cell_vol()[9] <<","
                << "Eleventh Cell Voltage is:"  << bms_state_.cell_vol()[10] <<","
                << "Twelfth Cell Voltage is:"  << bms_state_.cell_vol()[11] <<","
                << "Thirteenth Cell Voltage is:"  << bms_state_.cell_vol()[12] <<","
                << "Fouteenth Cell Voltage is:"  << bms_state_.cell_vol()[13] <<","
                << "Fifteenth Cell Voltage is:"  << bms_state_.cell_vol()[14];



    std::cout << battery_stream.str() << std::endl;

    if (battery_temp_logfile_.is_open()){
        battery_temp_logfile_<< battery_stream.str() << std::endl;
        battery_temp_logfile_.flush();
    }


    




    /// Printing the information for each of 12 motors
    for (int i = 0; i < 12; i++)
    {
        std::ostringstream log_stream;

        log_stream << time_stream.str() << " Report for Motor [" << i << " - " << motor_names[i] << "], "
                   << "Mode: " << motor_state_[i].mode() << ", "
                   << "Position: " << motor_state_[i].q() << ", "
                   << "Velocity: " << motor_state_[i].dq() << ", "
                   << "Estimated torque: " << motor_state_[i].tau_est() << ", "
                   << "Motor packet loss: " << motor_state_[i].lost() << ", "
                   << "Error code: " << motor_state_[i].reserve()[0] << ","
                   << "Temperature of the motor is:" << static_cast<int>(motor_state_[i].temperature());

        std::cout << log_stream.str() << std::endl;

        if (logfile_.is_open()){
            logfile_ << log_stream.str() << std::endl;
            logfile_.flush();
        }

        //std::vector<MotorErrorState> Motor_Errors  = motor_errors_.ReportErrors(motor_state_[i].reserve()[0]);
        // motor_errors_.ReportErrors(motor_state_[i].reserve()[0]);
        motor_errors_.ReportErrors(motor_names[i], motor_state_[i].reserve()[0], time_stream.str(), motor_error_logfile_);


    }


    // std::cout << "Quarternion Report is:" << ","
    //            << "Quaternion Data is:" << imu_state_.quaternion()[4] << ","
    //            << "Angular Velocity Information is:"  << imu_state_.gyroscope()[3] <<","
    //            << "Acceleration Information is:"  << imu_state_.accelerometer()[3] <<","
    //            << "Euler Angle Information is:"  << imu_state_.rpy()[3]<<","
    //            << "IMU temperature is:"  << imu_state_.temperature() <<","
    //            << std::endl;


    

               




}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    ChannelFactory::Instance()->Init(0, argv[1]);


    std::setlocale(LC_ALL, "C");  
    std::locale::global(std::locale::classic());
    std::cout.imbue(std::locale::classic());
    std::cerr.imbue(std::locale::classic());

    Custom custom;


   
    
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}