#!/usr/bin/python3

import serial
import time


def send_data(ser, str):
    for ch in str:
        ser.write(ch.encode("utf-8"))
        time.sleep(0.01)
    time.sleep(0.01)


def stm32_setup():
    ser = serial.Serial("/dev/ttyACM0", 115200)

    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.flush()
    return ser


if __name__ == "__main__":
    ser = stm32_setup()
    cmd = 'toggleled\r'
    while (1):
        send_data(cmd, ser)
        time.sleep(1)
