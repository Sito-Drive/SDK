#include <motor_control.h>
#include <multi_motor_control.h>

vector<uint8_t>ids = {0x00, 0x01};
MultiMotorControl motors("COM3", 921600, ids);
//MotorControl motor("COM3", 921600, 0x01);

int main()
{

}
