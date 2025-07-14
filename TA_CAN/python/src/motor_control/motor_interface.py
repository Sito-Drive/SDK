import time
import struct
import serial
from .constants import *

class MotorControl:
    def __init__(self, serial_port: str, id_values, baudrate = 921600):
        try:
            self.ser = serial.Serial(serial_port, baudrate)
            self.ser.timeout = 5
            self.ids = id_values
            self.is_connected = self.ser.is_open
        except serial.SerialException as e:
            print(f"Serial port connection failed: {e}")
            self.ser = None
            self.is_connected = False

    def close_serial(self):
        if self.ser:
            self.ser.close()
            self.is_connected = False
            print("Serial port closed")

    def Reset(self,flag):
        results = []
        self.ser.timeout = 1
        for motor_id in self.ids:
            id = 0x05060000 + (motor_id << 8)
            value = [0x55] * 8
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            if flag:
                time.sleep(0.2)
                start_time = time.time()
                while self.ser.in_waiting < 117 and time.time() - start_time <= 2:
                    time.sleep(0.01)
                feedback = self.ser.read(self.ser.in_waiting)
                header = bytes([0x88, 0x05, 0x06, motor_id, 0xB0])
                start_feedback = feedback.find(header)
                if start_feedback != -1 and len(feedback) >= start_feedback + 13:
                    packet = feedback[start_feedback: start_feedback + 13]
                    feedback1 = ''.join([f'{byte:02X}' for byte in packet[5:9]])
                    feedback2 = int.from_bytes(packet[9:13], byteorder='big', signed=True)
                    results.append([motor_id, feedback1, feedback2])
                else: results.append([motor_id])
            else:
                feedback = self.ser.read(13)
                if len(feedback) == 13 and feedback[4] == 0xB0:
                    feedback1 = ''.join([f'{byte:02X}' for byte in feedback[5:9]])
                    feedback2 = int.from_bytes(feedback[9:13], byteorder='big', signed=True)
                    results.append([motor_id, feedback1, feedback2])
                else: results.append([motor_id])
        self.ser.timeout = 5
        return results

    def mode_selection(self, serial_num, Feedback_cycle1, Feedback_cycle2, Feedback_cycle3):
        for motor_id in self.ids:
            serial_num = int(serial_num)
            if serial_num < 3:
                print('Please enter the correct parameters')
            else:
                id = 0x05060000 + (motor_id << 8) + 1
                value = [0x55] * 8
                value[0] = serial_num
                value[1] = Feedback_cycle1
                value[2] = Feedback_cycle2
                value[3] = Feedback_cycle3
                data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
                byte_data = bytes(data)
                self.ser.write(byte_data)
                print(pattern_phrases[serial_num])

    def write_async(self, pwm_values):
        if len(pwm_values) != len(self.ids):
            print("Error: The number of PWM values must match the number of motor IDs.")
            return
        for motor_id, pwm_value in zip(self.ids, pwm_values):
            id = 0x05060000 + (motor_id << 8) + ASYNC_MODE
            value = [0x55] * 8
            value[0] = (int(pwm_value) >> 8) & 0xFF
            value[1] = int(pwm_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            time.sleep(0.001)

    def write_open_loop(self, pwm_values):
        for motor_id, pwm_value in zip(self.ids, pwm_values):
            id = 0x05060000 + (motor_id << 8) + OPEN_LOOP_MODE
            value = [0x55] * 8
            value[0] = (int(pwm_value) >> 8) & 0xFF
            value[1] = int(pwm_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            time.sleep(0.001)

    def write_current(self, current_values):
        for motor_id, current_value in zip(self.ids, current_values):
            id = 0x05060000 + (motor_id << 8) + CURRENT_MODE
            value = [0x55] * 8
            value[0] = (int(current_value) >> 8) & 0xFF
            value[1] = int(current_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            time.sleep(0.001)

    def write_profile_velocity(self, current_values, velocity_values):
        for motor_id, current_value, velocity_value in zip(self.ids, current_values, velocity_values):
            id = 0x05060000 + (motor_id << 8) + PROFILE_VELOCITY_MODE
            value = [0x55] * 8
            value[0] = (int(current_value) >> 8) & 0xFF
            value[1] = int(current_value) & 0xFF
            value[2] = (int(velocity_value) >> 8) & 0xFF
            value[3] = int(velocity_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            time.sleep(0.001)

    def write_profile_position(self, current_values, velocity_values, position_values):
        for motor_id, current_value, velocity_value, position_value in zip(self.ids, current_values, velocity_values, position_values):
            id = 0x05060000 + (motor_id << 8) + PROFILE_POSITION_MODE
            value = [0x55] * 8
            value[0] = (int(current_value) >> 8) & 0xFF
            value[1] = int(current_value) & 0xFF
            value[2] = (int(velocity_value) >> 8) & 0xFF
            value[3] = int(velocity_value) & 0xFF
            value[4] = (int(position_value) >> 24) & 0xFF
            value[5] = (int(position_value) >> 16) & 0xFF
            value[6] = (int(position_value) >> 8) & 0xFF
            value[7] = int(position_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            time.sleep(0.001)

    def write_position(self, current_values, velocity_values, position_values):
        for motor_id, current_value, velocity_value, position_value in zip(self.ids, current_values, velocity_values, position_values):
            id = 0x05060000 + (motor_id << 8) + POSITION_MODE
            value = [0x00] * 8
            value[0] = (int(current_value) >> 8) & 0xFF
            value[1] = int(current_value) & 0xFF
            value[2] = (int(velocity_value) >> 8) & 0xFF
            value[3] = int(velocity_value) & 0xFF
            value[4] = (int(position_value) >> 24) & 0xFF
            value[5] = (int(position_value) >> 16) & 0xFF
            value[6] = (int(position_value) >> 8) & 0xFF
            value[7] = int(position_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            time.sleep(0.001)

    def write_mit(self, current_values, velocity_values, position_values, kps, kds):
        for motor_id, current_value, velocity_value, position_value, kp, kd in zip(self.ids, current_values, velocity_values, position_values, kps, kds):
            id = 0x05060000 + (motor_id << 8) + MIT_MODE
            value = [0x55] * 8
            value[0] = (int(current_value) >> 8) & 0xFF
            value[1] = int(current_value) & 0xFF
            value[2] = (int(velocity_value) >> 8) & 0xFF
            value[3] = int(velocity_value) & 0xFF
            value[4] = (int(position_value) >> 24) & 0xFF
            value[5] = (int(position_value) >> 16) & 0xFF
            value[6] = (int(position_value) >> 8) & 0xFF
            value[7] = int(position_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            kp_bytes = struct.pack('!f', kp)
            kd_bytes = struct.pack('!f', kd)
            id = 0x05060000 + (motor_id << 8) + MIT_PID_SET
            value = [0x55] * 8
            value[0] = kp_bytes[0]
            value[1] = kp_bytes[1]
            value[2] = kp_bytes[2]
            value[3] = kp_bytes[3]
            value[4] = kd_bytes[0]
            value[5] = kd_bytes[1]
            value[6] = kd_bytes[2]
            value[7] = kd_bytes[3]
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)
            time.sleep(0.001)

    def brake_control(self, pwm_values, time_values):
        for motor_id, pwm_value, time_value in zip(self.ids, pwm_values, time_values):
            id = 0x05060000 + (motor_id << 8) + BRAKE_CONTROL
            value = [0x55] * 8
            value[0] = (int(pwm_value) >> 8) & 0xFF
            value[1] = int(pwm_value) & 0xFF
            value[2] = (int(time_value) >> 8) & 0xFF
            value[3] = int(time_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)

    def openloop_acc_dec_set(self, acc_values, dec_values):
        for motor_id, acc_value, dec_value in zip(self.ids, acc_values, dec_values):
            id = 0x05060000 + (motor_id << 8) + OPENLOOP_ACC_DEC_SET
            value = [0x55] * 8
            value[0] = (int(acc_value) >> 24) & 0xFF
            value[1] = (int(acc_value) >> 16) & 0xFF
            value[2] = (int(acc_value) >> 8) & 0xFF
            value[3] = int(acc_value) & 0xFF
            value[4] = (int(dec_value) >> 24) & 0xFF
            value[5] = (int(dec_value) >> 16) & 0xFF
            value[6] = (int(dec_value) >> 8) & 0xFF 
            value[7] = int(dec_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)

    def velocity_acc_dec_set(self, acc_values, dec_values):
        for motor_id, acc_value, dec_value in zip(self.ids, acc_values, dec_values):
            id = 0x05060000 + (motor_id << 8) + VELOCITY_ACC_DEC_SET
            value = [0x55] * 8
            value[0] = (int(acc_value) >> 24) & 0xFF
            value[1] = (int(acc_value) >> 16) & 0xFF
            value[2] = (int(acc_value) >> 8) & 0xFF
            value[3] = int(acc_value) & 0xFF
            value[4] = (int(dec_value) >> 24) & 0xFF
            value[5] = (int(dec_value) >> 16) & 0xFF
            value[6] = (int(dec_value) >> 8) & 0xFF
            value[7] = int(dec_value) & 0xFF
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)

    def current_pid_set(self, kps, kis):
        for motor_id, kp, ki in zip(self.ids, kps, kis):
            kp_bytes = struct.pack('!f', kp)
            ki_bytes = struct.pack('!f', ki)
            id = 0x05060000 + (motor_id << 8) + CURRENT_PID_SET
            value = [0x55] * 8
            value[0] = kp_bytes[0]
            value[1] = kp_bytes[1]
            value[2] = kp_bytes[2]
            value[3] = kp_bytes[3]
            value[4] = ki_bytes[0]
            value[5] = ki_bytes[1]
            value[6] = ki_bytes[2]
            value[7] = ki_bytes[3]
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)

    def velocity_pid_set(self, kps, kis):
        for motor_id, kp, ki in zip(self.ids, kps, kis):
            kp_bytes = struct.pack('!f', kp)
            ki_bytes = struct.pack('!f', ki)
            id = 0x05060000 + (motor_id << 8) + VELOCITY_PID_SET
            value = [0x55] * 8
            value[0] = kp_bytes[0]
            value[1] = kp_bytes[1]
            value[2] = kp_bytes[2]
            value[3] = kp_bytes[3]
            value[4] = ki_bytes[0]
            value[5] = ki_bytes[1]
            value[6] = ki_bytes[2]
            value[7] = ki_bytes[3]
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)

    def position_pid_set(self, kps, kds):
        for motor_id, kp, kd in zip(self.ids, kps, kds):
            kp_bytes = struct.pack('!f', kp)
            kd_bytes = struct.pack('!f', kd)
            id = 0x05060000 + (motor_id << 8) + POSITION_PID_SET
            value = [0x55] * 8
            value[0] = kp_bytes[0]
            value[1] = kp_bytes[1]
            value[2] = kp_bytes[2]
            value[3] = kp_bytes[3]
            value[4] = kd_bytes[0]
            value[5] = kd_bytes[1]
            value[6] = kd_bytes[2]
            value[7] = kd_bytes[3]
            data = [0x88] + [(id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF] + value
            byte_data = bytes(data)
            self.ser.write(byte_data)

    def read_v_p_c_v(self):
        results = []
        for motor_id in self.ids:
            start_time = time.time()
            self.ser.reset_input_buffer()
            while self.ser.in_waiting < 117 and time.time() - start_time <= 3:
                time.sleep(0.01)
            feedback = self.ser.read(self.ser.in_waiting)
            header = bytes([0x88, 0x05, 0x06, motor_id, 0xB1])
            start_feedback = feedback.find(header)

            if start_feedback != -1 and len(feedback) >= start_feedback + 13:
                packet = feedback[start_feedback: start_feedback + 13]
                voltage_value = int.from_bytes(packet[5:7], byteorder='big', signed=True)
                pwm_value = int.from_bytes(packet[7:9], byteorder='big', signed=True)
                current_value = int.from_bytes(packet[9:11], byteorder='big', signed=True)
                velocity_value = int.from_bytes(packet[11:13], byteorder='big', signed=True)
                results.append([motor_id, voltage_value, pwm_value, current_value, velocity_value])
            else:
                print(f"Motor ID: {motor_id :<2} | The complete message was not found or the message length was insufficient")
        return results

    def read_pos(self):
        results = []
        for motor_id in self.ids:
            start_time = time.time()
            self.ser.reset_input_buffer()
            while self.ser.in_waiting < 117 and time.time() - start_time <= 3:
                time.sleep(0.01)
            feedback = self.ser.read(self.ser.in_waiting)
            header = bytes([0x88, 0x05, 0x06, motor_id, 0xB2])
            start_feedback = feedback.find(header)

            if start_feedback != -1 and len(feedback) >= start_feedback + 13:
                packet = feedback[start_feedback: start_feedback + 13]
                motor_position_value = int.from_bytes(packet[5:9], byteorder='big', signed=True)
                actuator_position_value = int.from_bytes(packet[9:13], byteorder='big', signed=True)
                results.append([motor_id, motor_position_value, actuator_position_value])
            else:
                print(f"Motor ID: {motor_id :<2} | The complete message was not found or the message length was insufficient")
        return results

    def read_temp_error(self):
        results = []
        for motor_id in self.ids:
            start_time = time.time()
            self.ser.reset_input_buffer()
            while self.ser.in_waiting < 117 and time.time() - start_time <= 3:
                time.sleep(0.01)
            feedback = self.ser.read(self.ser.in_waiting)
            header = bytes([0x88, 0x05, 0x06, motor_id, 0xB3])
            start_feedback = feedback.find(header)

            if start_feedback != -1 and len(feedback) >= start_feedback + 13:
                packet = feedback[start_feedback: start_feedback + 13]
                mos_temp = int.from_bytes(packet[5:6], byteorder='big', signed=True)
                mt_temp = int.from_bytes(packet[6:7], byteorder='big', signed=True)
                g431_temp = int.from_bytes(packet[7:8], byteorder='big', signed=True)
                warning_type = int.from_bytes(packet[8:9], byteorder='big', signed=True)
                error_type = int.from_bytes(packet[9:10], byteorder='big', signed=True)
                results.append([motor_id, mos_temp, mt_temp, g431_temp, warning_type, error_type])
            else:
                print(f"Motor ID: {motor_id :<2} | The complete message was not found or the message length was insufficient.")
        return results
