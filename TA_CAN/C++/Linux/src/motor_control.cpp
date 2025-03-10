#include <motor_control.h>

MotorControl::MotorControl(const string& port, const uint32_t& baudrate, const uint8_t& motor_id)
:motor_id(motor_id)
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        if (ser.isOpen()) 
        {
            cout << port << " is opened" << endl;
        } 
        else
        {
            ser.open();
            cout << port << " open success" << endl;
        }

    }
    catch (std::exception& e) 
    {
        cout << "Error: " << e.what() << endl;
    }
}

void MotorControl::closeport()
{
    if (ser.isOpen())
    {
        ser.close();
        cout << "Serial port closed" << endl;
    }
    else
    {
        cout << "Serial port was not open" << endl;
    }
}

// MotorControl::~MotorControl()
// {
//     if (ser.isOpen())
//     {
//         ser.close();
//         cout << "Serial port closed" << endl;
//     }
//     else 
//     {
//         cout << "Serial port was not open" << endl;
//     }
// }

void MotorControl::set_object_id(const uint8_t& new_id)
{
    motor_id = new_id;
}

void MotorControl::clearBuffer()
{
    if (ser.isOpen())
    {
        ser.flush();
        ser.read(ser.available());
    }
}

vector<int> MotorControl::reset()
{
    clearBuffer();
    ID.back() = 0x00;
    ID[2] = motor_id;
    insert_and_send_data();
    size_t response_length = 13;
    vector<uint8_t> response_data(response_length);
    vector<int> results(2, 0);
    auto start_time = chrono::steady_clock::now();

    while (true)
    {
        size_t bytes_read = ser.read(response_data.data(), response_length);

        if (bytes_read > 0)
        {
            if (bytes_read >= 5 &&
                response_data[0] == 0x88 &&
                response_data[1] == 0x05 &&
                response_data[2] == 0x06 &&
                response_data[3] == motor_id &&
                response_data[4] == 0xB0)
            {
                for (uint8_t byte : response_data)
                {
                    cout << hex << setw(2) << uppercase << setfill('0') << static_cast<int>(byte) << " ";
                }
                cout << endl;
                vector<int> b(response_data.begin() + 5, response_data.begin() + 9);
                auto version = (b[0] << 24) + (b[1] << 16) + (b[2] << 8) + b[3];
                results[0] = version;
                vector<int> a(response_data.begin() + 9, response_data.begin() + 13);
                auto date = (a[0] << 24) | (a[1] << 16) | (a[2] << 8) | a[3];
                results[1] = date;
                //cout << hex << version << " " << dec << date << endl;
                return results;
            }
        }
        auto elapsed_time = chrono::steady_clock::now() - start_time;
        if (chrono::duration_cast<chrono::seconds>(elapsed_time).count() >= 0.2)
        {
            cout << "Timeout: No valid response received within 0.2 seconds." << endl;
            return results;
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }
}

void MotorControl::reset()
{
    ID.back() = 0x00;
    ID[2] = motor_id;
    insert_and_send_data();
}

void MotorControl::mode_selection(const Mode& mode, const uint8_t& Feedback_cycle1, const uint8_t& Feedback_cycle2, const uint8_t& Feedback_cycle3)
{
    ID.back() = 0x01;
    ID[2] = motor_id;
    Data[0] = static_cast<int>(mode);
    Data[1] = Feedback_cycle1;
    Data[2] = Feedback_cycle2;
    Data[3] = Feedback_cycle3;
    insert_and_send_data();
}

void MotorControl::write_async(const int16_t& PWM_Value)
{
    ID.back() = static_cast<int>(Mode::ASYNC_MODE);
    ID[2] = motor_id;
    Data[0] = PWM_Value >> 8 & 0xFF;
    Data[1] = PWM_Value & 0xFF;
    insert_and_send_data();
}

void MotorControl::write_open_loop(const int16_t& PWM_Value)
{
    ID.back() = static_cast<int>(Mode::OPEN_LOOP_MODE);
    ID[2] = motor_id;
    Data[0] = PWM_Value >> 8 & 0xFF;
    Data[1] = PWM_Value & 0xFF;
    insert_and_send_data();
}

void MotorControl::write_current(const int16_t& Current_Value)
{
    ID.back() = static_cast<int>(Mode::CURRENT_MODE);
    ID[2] = motor_id;
    Data[0] = Current_Value >> 8 & 0xFF;
    Data[1] = Current_Value & 0xFF;
    insert_and_send_data();
}

void MotorControl::write_profile_velocity(const int16_t& Current_Value, const int16_t& Velocity_Value)
{
    ID.back() = static_cast<int>(Mode::PROFILE_VELOCITY_MODE);
    ID[2] = motor_id;
    Data[0] = Current_Value >> 8 & 0xFF;
    Data[1] = Current_Value & 0xFF;
    Data[2] = Velocity_Value >> 8 & 0xFF;
    Data[3] = Velocity_Value & 0xFF;
    insert_and_send_data();
}

void MotorControl::write_profile_position(const int16_t& Current_Value, const int16_t& Velocity_Value, const int32_t& Positin_Value, Mode mode)
{
    ID.back() = static_cast<int>(mode);
    ID[2] = motor_id;
    Data[0] = Current_Value >> 8 & 0xFF;
    Data[1] = Current_Value & 0xFF;
    Data[2] = Velocity_Value >> 8 & 0xFF;
    Data[3] = Velocity_Value & 0xFF;
    Data[4] = Positin_Value >> 24 & 0xFF;
    Data[5] = Positin_Value >> 16 & 0xFF;
    Data[6] = Positin_Value >> 8 & 0xFF;
    Data[7] = Positin_Value & 0xFF;
    insert_and_send_data();
}

void MotorControl::write_position(const int16_t& Current_Value, const int16_t& Velocity_Value, const int32_t& Positin_Value)
{
    ID.back() = static_cast<int>(Mode::POSITION_MODE);
    ID[2] = motor_id;
    Data[0] = Current_Value >> 8 & 0xFF;
    Data[1] = Current_Value & 0xFF;
    Data[2] = Velocity_Value >> 8 & 0xFF;
    Data[3] = Velocity_Value & 0xFF;
    Data[4] = Positin_Value >> 24 & 0xFF;
    Data[5] = Positin_Value >> 16 & 0xFF;
    Data[6] = Positin_Value >> 8 & 0xFF;
    Data[7] = Positin_Value & 0xFF;
    insert_and_send_data();
}

void MotorControl::write_mit(const int16_t& Current_Value, const int16_t& Velocity_Value, const int32_t& Positin_Value, const float& KP, const float& KI)
{
    write_profile_position(Current_Value, Velocity_Value, Positin_Value, Mode:: MIT_MODE);

    ID.back() = static_cast<int>(Mode::MIT_PID_SET);
    union buf_kp_ki
    {
        float f[2];
        uint8_t u8[8];
    };
    buf_kp_ki kp_ki;
    kp_ki.f[0] = KP;
    kp_ki.f[1] = KI;

    Data[0] = kp_ki.u8[3];
    Data[1] = kp_ki.u8[2];
    Data[2] = kp_ki.u8[1];
    Data[3] = kp_ki.u8[0];
    Data[4] = kp_ki.u8[7];
    Data[5] = kp_ki.u8[6];
    Data[6] = kp_ki.u8[5];
    Data[7] = kp_ki.u8[4];
    insert_and_send_data();
}

void MotorControl::write_openloop_acc_dec(const int16_t& Acc_Value, const int16_t& Dec_Value)
{
    ID.back() = static_cast<int>(Mode::OPENLOOP_ACC_DEC_SET);
    ID[2] = motor_id;
    Data[0] = Acc_Value >> 24 & 0xff;
    Data[1] = Acc_Value >> 16 & 0xff;
    Data[2] = Acc_Value >> 8 & 0xff;
    Data[3] = Acc_Value & 0xff;
    Data[4] = Dec_Value >> 24 & 0xff;
    Data[5] = Dec_Value >> 16 & 0xff;
    Data[6] = Dec_Value >> 8 & 0xff;
    Data[7] = Dec_Value & 0xff;
    insert_and_send_data();
}

void MotorControl::write_velocity_acc_dec(const int16_t& Acc_Value, const int16_t& Dec_Value)
{
    ID.back() = static_cast<int>(Mode::VELOCITY_ACC_DEC_SET);
    ID[2] = motor_id;
    Data[0] = Acc_Value >> 24 & 0xff;
    Data[1] = Acc_Value >> 16 & 0xff;
    Data[2] = Acc_Value >> 8 & 0xff;
    Data[3] = Acc_Value & 0xff;
    Data[4] = Dec_Value >> 24 & 0xff;
    Data[5] = Dec_Value >> 16 & 0xff;
    Data[6] = Dec_Value >> 8 & 0xff;
    Data[7] = Dec_Value & 0xff;
    insert_and_send_data();
}

void MotorControl::write_current_pid(const float& KP, const float& KI)
{
    ID.back() = static_cast<int>(Mode::CURRENT_PID_SET);
    ID[2] = motor_id;
    union buf_kp_ki
    {
        float f[2];
        uint8_t u8[8];
    };
    buf_kp_ki kp_ki;
    kp_ki.f[0] = KP;
    kp_ki.f[1] = KI;

    Data[0] = kp_ki.u8[3];
    Data[1] = kp_ki.u8[2];
    Data[2] = kp_ki.u8[1];
    Data[3] = kp_ki.u8[0];
    Data[4] = kp_ki.u8[7];
    Data[5] = kp_ki.u8[6];
    Data[6] = kp_ki.u8[5];
    Data[7] = kp_ki.u8[4];
    insert_and_send_data();
}

void MotorControl::write_velocity_pid(const float& KP, const float& KI)
{
    ID.back() = static_cast<int>(Mode::VELOCITY_PID_SET);
    ID[2] = motor_id;
    union buf_kp_ki
    {
        float f[2];
        uint8_t u8[8];
    };
    buf_kp_ki kp_ki;
    kp_ki.f[0] = KP;
    kp_ki.f[1] = KI;

    Data[0] = kp_ki.u8[3];
    Data[1] = kp_ki.u8[2];
    Data[2] = kp_ki.u8[1];
    Data[3] = kp_ki.u8[0];
    Data[4] = kp_ki.u8[7];
    Data[5] = kp_ki.u8[6];
    Data[6] = kp_ki.u8[5];
    Data[7] = kp_ki.u8[4];
    insert_and_send_data();
}

void MotorControl::write_position_pid(const float& KP, const float& KD)
{
    ID.back() = static_cast<int>(Mode::POSITION_PID_SET);
    ID[2] = motor_id;
    union buf_kp_kd
    {
        float f[2];
        uint8_t u8[8];
    };
    buf_kp_kd kp_kd;
    kp_kd.f[0] = KP;
    kp_kd.f[1] = KD;

    Data[0] = kp_kd.u8[3];
    Data[1] = kp_kd.u8[2];
    Data[2] = kp_kd.u8[1];
    Data[3] = kp_kd.u8[0];
    Data[4] = kp_kd.u8[7];
    Data[5] = kp_kd.u8[6];
    Data[6] = kp_kd.u8[5];
    Data[7] = kp_kd.u8[4];
    insert_and_send_data();
}

void MotorControl::brake_control(const int16_t& PWM_Value, const uint16_t& Time_Value)
{
    ID.back() = static_cast<int>(Mode::BRAKE_CONTROL);
    ID[2] = motor_id;
    Data[0] = PWM_Value >> 8 & 0xFF;
    Data[1] = PWM_Value & 0xFF;
    Data[2] = Time_Value >> 8 & 0xFF;
    Data[3] = Time_Value & 0xFF;
    insert_and_send_data();
}

void MotorControl::insert_and_send_data()
{
    vector<uint8_t> data;
    data.insert(data.end(), Header.begin(), Header.end());
    data.insert(data.end(), ID.begin(), ID.end());
    data.insert(data.end(), Data.begin(), Data.end());

    send_data(data.data(), data.size());
    // for (size_t i = 0; i < data.size(); i++)
    // {
    //     cout << "0X" << hex << setw(2) << setfill('0') << static_cast<int>(data[i]) << " ";
    //     if (i % 16 == 15) cout << endl;
    // }
    // cout << endl;
}

void MotorControl::send_data(const uint8_t* data, size_t size)
{
    size_t num_send = ser.write(data, size);
    if (num_send != size)
    {
        cout<< "Data send failed. bytes be send: " << num_send << endl;
    }
}

MotorStatus MotorControl::get_motor_status(vector<uint8_t> header_types)
{
    auto NBytes = 39;
    switch (header_types.size())
    {
    case 1:
    case 2:
        NBytes = 117;
        break;
    case 3:
        NBytes = 156;
        break;
    default:
        break;
    }

    MotorStatus status;
    auto start_time = chrono::steady_clock::now();
    vector<uint8_t> feedback;

    while (ser.available() < NBytes && chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start_time).count() <= 3)
    {
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    if (ser.available() >= NBytes)
    {
        feedback.resize(ser.available());
        ser.read(feedback.data(), feedback.size());
    }
    for (const auto& header_type : header_types)
    {
        if (header_type == 0xB1 || header_type == 0xB2 || header_type == 0xB3)
        {
            if (header_type == 0xB1)
            {
                vector<uint8_t> header1 = { 0x88, 0x05, 0x06, motor_id, 0xB1 };
                size_t start_feedback1 = find_start_of_feedback(feedback, header1);
                if (start_feedback1 != string::npos && feedback.size() >= start_feedback1 + 13)
                {
                    vector<uint8_t> packet(feedback.begin() + start_feedback1, feedback.begin() + start_feedback1 + 13);
                    status.voltage_value = static_cast<int16_t>((packet[5] << 8) | packet[6]);
                    status.pwm_value = static_cast<int16_t>((packet[7] << 8) | packet[8]);
                    status.current_value = static_cast<int16_t>((packet[9] << 8) | packet[10]);
                    status.velocity_value = static_cast<int16_t>((packet[11] << 8) | packet[12]);
                }
            }
            if (header_type == 0xB2)
            {
                vector<uint8_t> header2 = { 0x88, 0x05, 0x06, motor_id, 0xB2 };
                size_t start_feedback2 = find_start_of_feedback(feedback, header2);
                if (start_feedback2 != string::npos && feedback.size() >= start_feedback2 + 13)
                {
                    vector<uint8_t> packet(feedback.begin() + start_feedback2, feedback.begin() + start_feedback2 + 13);
                    status.motor_position_value = static_cast<int32_t>((packet[5] << 24) | (packet[6] << 16) | (packet[7] << 8) | packet[8]);
                    status.actuator_position_value = static_cast<int32_t>((packet[9] << 24) | (packet[10] << 16) | (packet[11] << 8) | packet[12]);
                }
            }
            if (header_type == 0xB3)
            {
                vector<uint8_t> header3 = { 0x88, 0x05, 0x06, motor_id, 0xB3 };
                size_t start_feedback3 = find_start_of_feedback(feedback, header3);
                if (start_feedback3 != string::npos && feedback.size() >= start_feedback3 + 13)
                {
                    vector<uint8_t> packet(feedback.begin() + start_feedback3, feedback.begin() + start_feedback3 + 13);
                    status.mos_temp = packet[5];
                    status.mt_temp = packet[6];
                    status.g431_temp = packet[7];
                    status.warning_type = packet[8];
                    status.error_type = packet[9];
                }
            }

        }
        else
        {
            cout << "Only 0xB1, 0xB2, 0xB3 can be entered" << endl;
            return {};
        }
    }
    return status;
}

size_t MotorControl::find_start_of_feedback(const vector<uint8_t>& feedback, const vector<uint8_t>& header)
{
    for (size_t i = 0; i <= feedback.size() - header.size(); ++i)
    {
        if (equal(header.begin(), header.end(), feedback.begin() + i))
        {
            return i;
        }
    }
    return string::npos;
}
