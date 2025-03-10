#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <iostream>
#include <iomanip>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <serial/serial.h>
using namespace std;

enum class Mode
{
    ASYNC_MODE = 0x03,
    OPEN_LOOP_MODE = 0x04,
    CURRENT_MODE = 0x05,
    PROFILE_VELOCITY_MODE = 0x06,
    PROFILE_POSITION_MODE = 0x07,
    POSITION_MODE = 0x08,
    MIT_MODE = 0x09,
    BRAKE_CONTROL = 0x20,
    OPENLOOP_ACC_DEC_SET = 0x30,
    VELOCITY_ACC_DEC_SET = 0x31,
    CURRENT_PID_SET = 0x42,
    VELOCITY_PID_SET = 0x43,
    POSITION_PID_SET = 0x44,
    MIT_PID_SET = 0x45
};

struct MotorStatus
{
    int16_t voltage_value = 0;
    int16_t pwm_value = 0;
    int16_t current_value = 0;
    int16_t velocity_value = 0;

    int32_t motor_position_value = 0;
    int32_t actuator_position_value = 0;

    uint8_t mos_temp = 0;
    uint8_t mt_temp = 0;
    uint8_t g431_temp = 0;
    uint8_t warning_type = 0;
    uint8_t error_type = 0;
};

class MotorControl
{
public:
    MotorControl(const string& port, const uint32_t& baudrate, const uint8_t& motor_id);
    //~MotorControl();
    void closeport();
    void set_object_id(const uint8_t& new_id);
    void clearBuffer();
    vector<int> reset();
    void mode_selection(const Mode& mode, const uint8_t& Feedback_cycle1, const uint8_t& Feedback_cycle2, const uint8_t& Feedback_cycle3);
    void write_async(const int16_t& PWM_Value);
    void write_open_loop(const int16_t& PWM_Value);
    void write_current(const int16_t& Current_Value);
    void write_profile_velocity(const int16_t& Current_Value, const int16_t& Velocity_Value);
    void write_profile_position(const int16_t& Current_Value, const int16_t& Velocity_Value, const int32_t& Positin_Value, Mode mode = Mode::PROFILE_POSITION_MODE);
    void write_position(const int16_t& Current_Value, const int16_t& Velocity_Value, const int32_t& Positin_Value);
    void write_mit(const int16_t& Current_Value, const int16_t& Velocity_Value, const int32_t& Positin_Value, const float& KP, const float& KI);
    void write_openloop_acc_dec(const int16_t& Acc_Value, const int16_t& Dec_Value);
    void write_velocity_acc_dec(const int16_t& Acc_Value, const int16_t& Dec_Value);
    void write_current_pid(const float& KP, const float& KI);
    void write_velocity_pid(const float& KP, const float& KI);
    void write_position_pid(const float& KP, const float& KD);
    void brake_control(const int16_t& PWM_Value, const uint16_t& Time_Value);
    void insert_and_send_data();
    void send_data(const uint8_t* data, size_t size);
    MotorStatus get_motor_status(vector<uint8_t> header_types);

private:
    uint8_t motor_id;
    serial::Serial ser;

    vector<uint8_t> Header = { 0x88 };
    vector<uint8_t> ID = { 0x05, 0x06, 0x00, 0x00 };
    vector<uint8_t> Data = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55 };

    size_t find_start_of_feedback(const vector<uint8_t>& feedback, const vector<uint8_t>& header);
};

#endif
