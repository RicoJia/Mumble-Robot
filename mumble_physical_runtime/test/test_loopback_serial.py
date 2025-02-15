#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial("/dev/serial0", 115200, timeout=1)
    ser.flush()

    test_message = "hello\n"
    ser.write(test_message.encode("utf-8"))
    print(f"Sent: {test_message.strip()}")

    time.sleep(1)  # Wait for the data to loop back

    if ser.in_waiting > 0:
        received = ser.read(ser.in_waiting).decode("utf-8").rstrip()
        print(f"Received: {received}")
    else:
        print("No data received.")

    ser.close()
except serial.SerialException as e:
    print(f"Serial exception: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
