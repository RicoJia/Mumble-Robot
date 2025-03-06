#!/usr/bin/env python3
# Adopted from here: https://github.com/waveshareteam/ugv_rpi/blob/main/tutorial_en/base_ctrl.py

import json
import os
import queue
import threading
import time

import serial


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[: i + 1]
            self.buf = self.buf[i + 1 :]
            return r
        while True:
            # Check how much data is available in the buffer
            i = max(1, min(512, self.s.in_waiting or 1))
            data = self.s.read(i)

            # Handle timeout or no data received
            if not data:
                raise TimeoutError("No data received before timeout.")

            # Rico: the original function had a bug
            # Look for newline in the received data
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[: i + 1]
                self.buf[0:] = data[i + 1 :]
                return r
            else:
                self.buf.extend(data)


class BaseController:
    def __init__(self, uart_dev_set, buad_set, mock=False):
        self.command_queue = queue.Queue()
        self.mock = mock
        if mock:
            return
        self.ser = serial.Serial(uart_dev_set, buad_set, timeout=0.1)
        self.rl = ReadLine(self.ser)
        self.command_thread = threading.Thread(
            target=self.process_commands, daemon=True
        )
        self.command_thread.start()
        self.send_command({"T": 143, "cmd": 0})  # turn off echo mode
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def on_data_received(self):
        # firmware may require an acknowledgment from us, like a handshake
        line = self.rl.readline().decode("utf-8")
        data_read = json.loads(
            line
        )  # Rico Must have for data transactions. Otherwise OLED won't change
        self.ser.reset_input_buffer()
        return data_read

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            self.ser.write((json.dumps(data) + "\n").encode("utf-8"))

    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

    def base_speed_ctrl(self, input_left, input_right):
        # max speed: 0.5. It could be as fast as 2m/s on wood surfaces
        data = {"T": 1, "L": input_left, "R": input_right}
        self.send_command(data)

    def raw_imu_info(self):
        """
        Resp looks like:
        {'T': 1002, 'r': 0.024750011, 'p': 0.236115932, 'y': 93.8493042,
        'ax': -8.212890625, 'ay': -5.517578125, 'az': 994.8242188,
        'gx': 2.032500029, 'gy': -0.098750114, 'gz': -0.057500005,
        'mx': 6, 'my': 84, 'mz': -100, 'temp': 0}
        """
        self.send_command({"T": 126})
        resp = self.on_data_received()
        return resp

    def base_oled(self, input_line, input_text):
        data = {"T": 3, "lineNum": input_line, "Text": input_text}
        self.send_command(data)

    def base_default_oled(self):
        data = {"T": -3}
        self.send_command(data)


def test_oled(base):
    base.send_command({"T": 3, "lineNum": 0, "Text": "this is line0"})
    base.send_command({"T": 3, "lineNum": 1, "Text": "this is rico1"})
    base.send_command({"T": 3, "lineNum": 2, "Text": "this is rico2"})
    base.send_command({"T": 3, "lineNum": 3, "Text": "this is line3"})
    response = base.on_data_received()
    print("Response from UGV:", response)


def test_imu(base):
    # Adopted from: https://github.com/waveshareteam/ugv_rpi/blob/main/tutorial_en/08%20Microcontroller%20JSON%20Command%20Set.ipynb
    # two consecutive 126 will return the imu info
    while True:
        start = time.time()
        resp = base.raw_imu_info()
        print(f"time: {time.time() - start}, Response from UGV:", resp)


if __name__ == "__main__":
    base = BaseController("/dev/ttyS0", 115200)
    test_imu(base)
    # test_oled()
