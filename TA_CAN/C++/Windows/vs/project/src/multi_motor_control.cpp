#include <multi_motor_control.h>

MultiMotorControl::MultiMotorControl(const string& port, const uint32_t& baudrate, const vector<uint8_t>& motor_ids)
    :MotorControl(port, baudrate, motor_ids[0]), motor_ids_(motor_ids)
{}

void MultiMotorControl::closeport()
{
    MotorControl::closeport();
}

void MultiMotorControl::set_object_id(const vector<uint8_t>& new_id)
{
    motor_ids_ = new_id;
}

vector<vector<int>> MultiMotorControl::reset()
{
    vector<vector<int>> results;
    for (uint8_t id : motor_ids_)
    {
        MotorControl::set_object_id(id);
        vector<int> result = MotorControl::reset();
        results.push_back(result);
    }
    return results;
}

void MultiMotorControl::mode_selection(const Mode& mode, const uint8_t& Feedback_cycle1, const uint8_t& Feedback_cycle2, const uint8_t& Feedback_cycle3)
{
    for (uint8_t id : motor_ids_)
    {
        MotorControl::set_object_id(id);
        MotorControl::mode_selection(mode, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3);
    }
}

void MultiMotorControl::write_async(const vector<int16_t>& PWM_Values)
{
    validate_param_size(PWM_Values, "PWM_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t PWM_Value = PWM_Values.size() == 1 ? PWM_Values[0] : PWM_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_async(PWM_Value);
    }
}

void MultiMotorControl::write_open_loop(const vector<int16_t>& PWM_Values)
{
    validate_param_size(PWM_Values, "PWM_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t PWM_Value = PWM_Values.size() == 1 ? PWM_Values[0] : PWM_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_open_loop(PWM_Value);
    }
}

void MultiMotorControl::write_current(const vector<int16_t>& Current_Values)
{
    validate_param_size(Current_Values, "Current_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t Current_Value = Current_Values.size() == 1 ? Current_Values[0] : Current_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_open_loop(Current_Value);
    }
}

void MultiMotorControl::write_profile_velocity(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values)
{
    validate_param_size(Current_Values, "Current_Values");
    validate_param_size(Velocity_Values, "Velocity_Values");

    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t Current_Value = Current_Values.size() == 1 ? Current_Values[0] : Current_Values[i];
        int16_t Velocity_Value = Velocity_Values.size() == 1 ? Velocity_Values[0] : Velocity_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_profile_velocity(Current_Value, Velocity_Value);
    }
}

void MultiMotorControl::write_profile_position(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values,
    const vector<int32_t>& Position_Values)
{
    validate_param_size(Current_Values, "Current_Values");
    validate_param_size(Velocity_Values, "Velocity_Values");
    validate_param_size(Position_Values, "Position_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t Current_Value = Current_Values.size() == 1 ? Current_Values[0] : Current_Values[i];
        int16_t Velocity_Value = Velocity_Values.size() == 1 ? Velocity_Values[0] : Velocity_Values[i];
        int32_t Positin_Value = Position_Values.size() == 1 ? Position_Values[0] : Position_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_profile_position(Current_Value, Velocity_Value, Positin_Value);
    }
}

void MultiMotorControl::write_position(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values,
    const vector<int32_t>& Position_Values)
{
    validate_param_size(Current_Values, "Current_Values");
    validate_param_size(Velocity_Values, "Velocity_Values");
    validate_param_size(Position_Values, "Position_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t Current_Value = Current_Values.size() == 1 ? Current_Values[0] : Current_Values[i];
        int16_t Velocity_Value = Velocity_Values.size() == 1 ? Velocity_Values[0] : Velocity_Values[i];
        int32_t Positin_Value = Position_Values.size() == 1 ? Position_Values[0] : Position_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_position(Current_Value, Velocity_Value, Positin_Value);
    }
}

void MultiMotorControl::write_mit(const vector<int16_t>& Current_Values, const vector<int16_t>& Velocity_Values, const vector<int32_t>& Position_Values,
    const vector<float>& KPs, const vector<float>& KDs)
{
    validate_param_size(Current_Values, "Current_Values");
    validate_param_size(Velocity_Values, "Velocity_Values");
    validate_param_size(Position_Values, "Positin_Values");
    validate_param_size(KPs, "KPs");
    validate_param_size(KDs, "KDs");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t Current_Value = Current_Values.size() == 1 ? Current_Values[0] : Current_Values[i];
        int16_t Velocity_Value = Velocity_Values.size() == 1 ? Velocity_Values[0] : Velocity_Values[i];
        int32_t Positin_Value = Position_Values.size() == 1 ? Position_Values[0] : Position_Values[i];
        float KP = KPs.size() == 1 ? KPs[0] : KPs[i];
        float KD = KDs.size() == 1 ? KDs[0] : KDs[i];

        MotorControl::set_object_id(id);
        MotorControl::write_mit(Current_Value, Velocity_Value, Positin_Value, KP, KD);
    }
}

void MultiMotorControl::write_openloop_acc_dec(const vector<int16_t>& Acc_Values, const vector<int16_t>& Dec_Values)
{
    validate_param_size(Acc_Values, "Acceleration_Values");
    validate_param_size(Dec_Values, "Deceleration_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t Acc_Value = Acc_Values.size() == 1 ? Acc_Values[0] : Acc_Values[i];
        int16_t Dec_Value = Dec_Values.size() == 1 ? Dec_Values[0] : Dec_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_openloop_acc_dec(Acc_Value, Dec_Value);
    }
}

void MultiMotorControl::write_velocity_acc_dec(const vector<int16_t>& Acc_Values, const vector<int16_t>& Dec_Values)
{
    validate_param_size(Acc_Values, "Acceleration_Values");
    validate_param_size(Dec_Values, "Deceleration_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t Acc_Value = Acc_Values.size() == 1 ? Acc_Values[0] : Acc_Values[i];
        int16_t Dec_Value = Dec_Values.size() == 1 ? Dec_Values[0] : Dec_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::write_velocity_acc_dec(Acc_Value, Dec_Value);
    }
}

void MultiMotorControl::write_current_pid(const vector<float>& KPs, const vector<float>& KIs)
{
    validate_param_size(KPs, "KPs");
    validate_param_size(KIs, "KIs");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        float KP = KPs.size() == 1 ? KPs[0] : KPs[i];
        float KI = KIs.size() == 1 ? KIs[0] : KIs[i];

        MotorControl::set_object_id(id);
        MotorControl::write_current_pid(KP, KI);
    }
}

void MultiMotorControl::write_velocity_pid(const vector<float>& KPs, const vector<float>& KIs)
{
    validate_param_size(KPs, "KPs");
    validate_param_size(KIs, "KIs");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        float KP = KPs.size() == 1 ? KPs[0] : KPs[i];
        float KI = KIs.size() == 1 ? KIs[0] : KIs[i];

        MotorControl::set_object_id(id);
        MotorControl::write_velocity_pid(KP, KI);
    }
}

void MultiMotorControl::write_position_pid(const vector<float>& KPs, const vector<float>& KDs)
{
    validate_param_size(KPs, "KPs");
    validate_param_size(KDs, "KIs");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        float KP = KPs.size() == 1 ? KPs[0] : KPs[i];
        float KD = KDs.size() == 1 ? KDs[0] : KDs[i];

        MotorControl::set_object_id(id);
        MotorControl::write_position_pid(KP, KD);
    }
}

void MultiMotorControl::brake_control(const vector<int16_t>& PWM_Values, const vector<uint16_t>& Time_Values)
{
    validate_param_size(PWM_Values, "PWM_Values");
    validate_param_size(Time_Values, "Time_Values");
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        uint8_t id = motor_ids_[i];
        int16_t PWM_Value = PWM_Values.size() == 1 ? PWM_Values[0] : PWM_Values[i];
        int16_t Time_Value = Time_Values.size() == 1 ? Time_Values[0] : Time_Values[i];

        MotorControl::set_object_id(id);
        MotorControl::brake_control(PWM_Value, Time_Value);
    }
}

vector<MotorStatus> MultiMotorControl::get_motor_status(vector<uint8_t> header_types)
{
    vector<MotorStatus> all_motor_status;
    for (size_t i = 0; i < motor_ids_.size(); ++i)
    {
        MotorStatus status;
        uint8_t id = motor_ids_[i];

        MotorControl::set_object_id(id);
        status = MotorControl::get_motor_status({ header_types });
        all_motor_status.push_back(status);
    }
    return all_motor_status;
}