#!/usr/bin/python3

import serial
import time


def send_data(str, ser):
    for ch in str:
        ser.write(ch.encode("utf-8"))
        time.sleep(0.01)

def move_x(step, ser):
    send_data("stepx " + str(step), ser)

def move_y(step, ser):
    send_data("stepy " + str(step), ser)

def move_rot_servo(angle, ser):
    send_data("servo_rot " + str(angle), ser)

def move_lin_servo(pctg, ser):
    send_data("servp_lin " + str(pctg), ser)



# if __name__ == "__main__":
def motor_setup():
    ser = serial.Serial("/dev/ttyACM0", 115200)

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.flush()
    return ser


if __name__ == "__main__":
    ser = motor_setup()
    cmd = 'toggleled\r'
    while (1):
        send_data(cmd, ser)
        time.sleep(1)
