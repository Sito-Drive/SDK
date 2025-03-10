from motor_control import *
acc_dec = [200]
ids = [0x00]      # list of int
cur = [3000]      # list of int
vel = [1500]      # list of int
pos_0 = [0]          # list of int
pos = [65535]    # list of int

# motors = MotorControl('/dev/ttyUSB0', ids)                           # Params: serial port path (str), list of actuator ID
motors = MotorControl('COM3', ids) 
# a = reset(motors)
# print(a)
# mode_selection(motors, PROFILE_POSITION_MODE, 10, 10, 10)            # Params: MotorControl object, operation mode, feedback cycle 1, 2, 3
# time.sleep(0.1)
# write_profile_position(motors, cur, vel, pos)
# while (1):
#     num = 0
#     pos_results = get_pos(motors)
#     for i in range(0,2):
#         if (pos_results[i][2] >= pos[i]):
#             num += 1
#     if (num == len(pos_results)):
#         break
# time.sleep(0.5)
# write_profile_position(motors, cur, vel, pos_0)
# while (1):
#     num = 0
#     pos_results = get_pos(motors)
#     for i in range(0,2):
#         if (pos_results[i][2] <= pos_0[i]):
#             num += 1
#     if (num == len(pos_results)):
#         break
# time.sleep(0.3)
# reset(motors)



















# duty_valus = [500, 500] # list of int
# cur = [500, 500]        # list of int
# vel = [0, 0]            # list of int
# pos = [0, 0]            # list of int
# kps = [0.0, 0.0]        # list of float
# kds = [0.0, 0.0]        # list of float

# acc_dec = 0
# cur = [500, 500]        # list of int
# vel = [500, 500]        # list of int
# vel0 = [0, 0]
# pos = [65535, 65535]    # list of int
# kps = [0.0, 0.0]        # list of float
# kds = [0.0, 0.0]        # list of float
# reset(motor)                                                        # Parameter: MotorControl object
# write_async(motor, duty_valus)                                      # Params: MotorControl object, list of duty values
# write_open_loop(motor, duty_valus)                                  # Params: MotorControl object, list of duty values
# write_current(motor, cur)                                           # Params: MotorControl object, list of target current
# write_profile_velocity(motor, cur, vel)                             # Params: MotorControl object, list of current limit, list of target velocity 
# write_profile_position(motor, cur, vel, pos)                        # Params: MotorControl object, list of current limit, list of velocity limit, 
                                                                    #         list of target position
# write_position(motor, cur, vel, pos)                                # Params: MotorControl object, list of current limit, list of velocity limit, 
                                                                    #         list of target position
# write_mit(motor, cur, vel, pos, kps, kds)                           # Params: MotorControl object, list of target current, list of target velocity, 
                                                                    #         list of target position, list of KP, list of KD

# v_p_c_v_results = get_v_p_c_v(motor)
# print("Printing Results:")
# for i in range(0,len(ids)):
#     print(f"ID: {v_p_c_v_results[i][0]:<2} Voltage: {v_p_c_v_results[i][1]:<2} PWM: {v_p_c_v_results[i][2]:<2} "
#           f"Current: {v_p_c_v_results[i][3]:<2} Velocity: {v_p_c_v_results[i][4]:<2}")

# pos_results = get_pos(motor)
# print("Printing Results:")
# for i in range(0,len(ids)):
#     print(f"ID: {pos_results[i][0]:<2} Motor Position: {pos_results[i][1]:<2} Actuator Position: {pos_results[i][2]:<2} ")

# temp_error_results = get_temp_error(motor)
# print("Printing Results:")
# for i in range(0,len(ids)):
#     print(f"ID: {temp_error_results[i][0]:<2} Mos Temp: {temp_error_results[i][1]:<2} Motor Temp: {temp_error_results[i][2]:<2} "
#           f"G431 Temp: {temp_error_results[i][3]:<2} Warning Type: {temp_error_results[i][4]:<2} Error Type: {temp_error_results[i][4]:<2}")

# pwm = [-2000, -2000]    # list of int
# dur = [500, 500]        # list of int
# acc = [2000, 2000]      # list of int
# dec = [2000, 2000]      # list of int
# kp = [0.001, 0.001]     # list of float
# ki = [0.001, 0.001]     # list of float
# kd = [0.001, 0.001]     # list of float

# brake_control(motor, pwm, dur)                                      # Params: MotorControl object, list of PWM, list of durations
# openloop_acc_dec_set(motor, acc, dec)                               # Params: MotorControl object, list of acceleration, list of deceleration
# velocity_acc_dec_set(motor, acc, dec)                               # Params: MotorControl object, list of acceleration, list of deceleration

# current_pid_set(motor, kp, ki)                                      # Params: MotorControl object, list of KP, list of KI
# velocity_pid_set(motor, kp, ki)                                     # Params: MotorControl object, list of KP, list of KI
# position_pid_set(motor, kp, kd)                                     # Params: MotorControl object, list of KP, list of KD



motors.close_serial()