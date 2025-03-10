#include <motor_control.h>
#include <multi_motor_control.h>

vector<uint8_t>ids = { 0x01, 0x17 };
MultiMotorControl motors("COM27", 921600, ids);
// MotorControl motor("Com27", 921600, 0x01);
int main()
{

}
