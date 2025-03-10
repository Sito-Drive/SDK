import time
from .motor_interface import MotorControl
from .constants import *

def reset(motor: MotorControl, flag = 0):
    return motor.Reset(flag)

def mode_selection(motor: MotorControl, mode, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3):
    motor.mode_selection(mode, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3)

def write_async(motor: MotorControl, pwm_values):
    motor.write_async(pwm_values)

def write_open_loop(motor: MotorControl, pwm_values):
    motor.write_open_loop(pwm_values)

def write_current(motor: MotorControl, current_values):
    motor.write_current(current_values)

def write_profile_velocity(motor: MotorControl, current_values, velocity_values):
    motor.write_profile_velocity(current_values, velocity_values)

def write_profile_position(motor: MotorControl, current_values, velocity_values, position_values):
    motor.write_profile_position(current_values, velocity_values, position_values)

def write_position(motor: MotorControl, current_values, velocity_values, position_values):
    motor.write_position(current_values, velocity_values, position_values)

def write_mit(motor: MotorControl, current_values, velocity_values, position_values, kps, kis):
    motor.write_mit(current_values, velocity_values, position_values, kps, kis)

def brake_control(motor: MotorControl, pwm_values, time_values):
    motor.brake_control(pwm_values, time_values)

def openloop_acc_dec_set(motor: MotorControl, acc_values, dec_values):
    motor.openloop_acc_dec_set(acc_values, dec_values)

def velocity_acc_dec_set(motor: MotorControl, acc_values, dec_values):
    motor.velocity_acc_dec_set(acc_values, dec_values)

def current_pid_set(motor: MotorControl, kps, kis):
    motor.current_pid_set(kps, kis)

def velocity_pid_set(motor: MotorControl, kps, kis):
    motor.velocity_pid_set(kps, kis)

def position_pid_set(motor: MotorControl, kps, kds):
    motor.position_pid_set(kps, kds)

def get_v_p_c_v(motor: MotorControl):
    return motor.read_v_p_c_v()

def get_pos(motor: MotorControl):
    return motor.read_pos()

def get_temp_error(motor: MotorControl):
    return motor.read_temp_error()