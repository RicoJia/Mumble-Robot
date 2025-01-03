#!/usr/bin/env python3
# Adopted from here: https://github.com/waveshareteam/ugv_rpi/blob/main/tutorial_en/base_ctrl.py

import serial  
import json
import queue
import threading
import os
import time

curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)

class ReadLine:
	def __init__(self, s):
		self.buf = bytearray()
		self.s = s

	def readline(self):
		i = self.buf.find(b"\n")
		if i >= 0:
			r = self.buf[:i+1]
			self.buf = self.buf[i+1:]
			return r
		while True:
			i = max(1, min(512, self.s.in_waiting))
			data = self.s.read(i)
			i = data.find(b"\n")
			if i >= 0:
				r = self.buf + data[:i+1]
				self.buf[0:] = data[i+1:]
				return r
			else:
				self.buf.extend(data)


class BaseController:

	def __init__(self, uart_dev_set, buad_set):
		self.ser = serial.Serial(uart_dev_set, buad_set, timeout=1)
		self.rl = ReadLine(self.ser)
		self.command_queue = queue.Queue()
		self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
		self.command_thread.start()


	def on_data_received(self):
     # firmware may require an acknoledge from us, like a handshake
		data_read = json.loads(self.rl.readline().decode('utf-8')) # Rico TODO: Must have for data transactions. Otherwise OLED won't change
		self.ser.reset_input_buffer()
		return data_read


	def send_command(self, data):
		self.command_queue.put(data)


	def process_commands(self):
		while True:
			data = self.command_queue.get()
			self.ser.write((json.dumps(data) + '\n').encode("utf-8"))

	def base_json_ctrl(self, input_json):
		self.send_command(input_json)


	def gimbal_emergency_stop(self):
		data = {"T":0}
		self.send_command(data)


	def base_speed_ctrl(self, input_left, input_right):
     # max speed: 0.5. It could be as fast as 2m/s on wood surfaces
		data = {"T":1,"L":input_left,"R":input_right}
		self.send_command(data)


	def gimbal_ctrl(self, input_x, input_y, input_speed, input_acceleration):
		data = {"T":133,"X":input_x,"Y":input_y,"SPD":input_speed,"ACC":input_acceleration}
		self.send_command(data)


	def gimbal_base_ctrl(self, input_x, input_y, input_speed):
		data = {"T":141,"X":input_x,"Y":input_y,"SPD":input_speed}
		self.send_command(data)


	def base_oled(self, input_line, input_text):
		data = {"T":3,"lineNum":input_line,"Text":input_text}
		self.send_command(data)


	def base_default_oled(self):
		data = {"T":-3}
		self.send_command(data)

	def lights_ctrl(self, pwmA, pwmB):
		data = {"T":132,"IO4":pwmA,"IO5":pwmB}
		self.send_command(data)


	def gimbal_dev_close(self):
		self.ser.close()

base = BaseController('/dev/serial0', 115200)
base.ser.reset_input_buffer()
base.ser.reset_output_buffer()
base.base_default_oled()
# for i in range(12): # max speed: 0.5m/s
#     base.base_speed_ctrl(0.05*i,0.05*i)
#     time.sleep(1.0)
#     response = base.on_data_received()
#     print("Response from UGV:", response)
# base.base_speed_ctrl(0.5, 0.5)
# time.sleep(1.0)
# base.base_speed_ctrl(0, 0)
# # # Modifying the Display Content on the OLED Screen
# # base.send_command({"T":3,"lineNum":0,"Text":"this is rico0"})
# base.send_command({"T":3,"lineNum":0,"Text":"this is line0"})
# base.send_command({"T":3,"lineNum":1,"Text":"this is rico1"})
# base.send_command({"T":3,"lineNum":2,"Text":"this is rico2"})
# base.send_command({"T":3,"lineNum":3,"Text":"this is line3"})
# # Implement an infinite loop to continuously monitor serial port data.

response = base.on_data_received()
print("Response from UGV:", response)