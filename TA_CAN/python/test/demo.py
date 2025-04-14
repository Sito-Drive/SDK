# import sys
# sys.path.append('C:/Users/Admin/Desktop/TA_CAN/python/src/motor_control')

from motor_control import *

# Selecting an operating mode
# port_name: Windows: "COM*"  Linux: "/dev/ttyUSB*"
# id: Executor ID
# Baudrate: The baud rate can only be 921600
port_name = "COM6"
id = [0x00]
Baudrate = 921600
motor = MotorControl(port_name, id, Baudrate)

# CURRENT_MODE
# Feedback_cycle1(ms): Feedback period of the first status packet，
#                      Including voltage, pwm, current, speed
# Feedback_cycle2(ms)：Including the position of the motor and the position of the actuator
# Feedback_cycle3(ms)：Including MOS tube, motor, chip temperature and warning information, 
#                      error information, current operation mode
# cur: current(mA)
Feedback_cycle1 = 10
Feedback_cycle2 = 10
Feedback_cycle3 = 10
cur = [1000]

reset(motor)
mode_selection(motor, CURRENT_MODE, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3)
write_current(motor, cur)

# PROFILE_VELOCITY_MODE
# cur(mA): limit current
# vel(rpm): target velocity of motor
cur = [10000]
vel = [1000]

reset(motor)
mode_selection(motor, PROFILE_VELOCITY_MODE, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3)
write_profile_velocity(motor, cur, vel)

# PROFILE_POSITION_MODE
# cur(mA): limit current 
# vel(rpm): the limit velocity of motor
# pos(count): target position, With a 16-bit encoder, the actuator counts 65536 per revolution.
cur = [10000]
vel = [1000]
pos = [0]
reset(motor)
mode_selection(motor, PROFILE_POSITION_MODE, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3)
write_profile_position(motor, cur, vel, pos)
write_profile_position(motor, cur, vel, {10000})

# POSITION_MODE
reset(motor)
mode_selection(motor, POSITION_MODE, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3)
write_position(motor, cur, vel, {0})
write_position(motor, cur, vel, {10000})

# MIT mode(Refer to the tutorial on how to use MIT mode)
cur = [1000]
vel = [0]
pos = [0]
kp = [0]
kd = [0]

reset(motor)
mode_selection(motor, MIT_MODE, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3)

# control current
write_mit(motor, cur, vel, pos, kp, kd)
write_mit(motor, {0}, vel, pos, kp, kd)

# control velocity
kp = [0]
kd = [13.1]
write_mit(motor, {500}, {600}, pos, kp, kd)
write_mit(motor, {0}, {0}, pos, kp, kd)

# control position
kp = [0.09]
kd = [13.1]
write_mit(motor, {0}, {0}, {0}, kp, kd)
write_mit(motor, {0}, {0}, {10000}, kp, kd)

# results1(list): Receive the content of the first status packet
# results1(list): Receive the content of the second status packet
# results1(list): Receive the content of the third status packet
results1 = get_v_p_c_v(motor)
results2 = get_pos(motor) 
results3 = get_temp_error(motor)