#ifndef MULTI_MOTOR_CONTROL_H
#define MULTI_MOTOR_CONTROL_H
#include <memory>
#include <unordered_map>
#include <stdexcept>
#include <motor_control.h>

class MultiMotorControl:public MotorControl
{
public:
    MultiMotorControl(const string& port, const uint32_t& baudrate, const vector<uint8_t>& motor_ids);
    //~MultiMotorControl();
    void closeport();
    void set_object_id(const vector<uint8_t>& new_id);
    vector<vector<int>> reset();
    void mode_selection(const Mode& mode, const uint8_t& Feedback_cycle1, const uint8_t& Feedback_cycle2, const uint8_t& Feedback_cycle3);
    void write_async(const vector<int16_t>& PWM_Values);
    void write_open_loop(const vector<int16_t>& PWM_Values);
    void write_current(const vector<int16_t>& Current_Values);
    void write_profile_velocity(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values);
    void write_profile_position(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values,
        const vector<int32_t>& Position_Values);
    void write_position(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values,
        const vector<int32_t>& Position_Values);
    void write_mit(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values, const vector<int32_t>& Position_Values,
        const vector<float>& KPs, const vector<float>& KIs);
    void write_openloop_acc_dec(const vector<int16_t>& Acc_Value, const vector<int16_t>& Dec_Value);
    void write_velocity_acc_dec(const vector<int16_t>& Acc_Values, const vector<int16_t>& Dec_Values);
    void write_current_pid(const vector<float>& KPs, const vector<float>& KIs);
    void write_velocity_pid(const vector<float>& KPs, const vector<float>& KIs);
    void write_position_pid(const vector<float>& KPs, const vector<float>& KDs);
    void brake_control(const vector<int16_t>& PWM_Values, const vector<uint16_t>& Time_Values);
    vector<MotorStatus> get_motor_status(vector<uint8_t> header_types);

private:
    serial::Serial ser;
    vector<uint8_t> motor_ids_;

    template <typename T>
    void validate_param_size(const vector<T>& values, const string& param_name)
    {
        if (values.size() != motor_ids_.size() && values.size() != 1)
        {
            throw invalid_argument(param_name + " size must match the number of motors or be 1");
        }
    }
};

#endif