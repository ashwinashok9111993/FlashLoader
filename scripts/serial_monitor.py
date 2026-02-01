#!/usr/bin/env python3
"""
Simple serial monitor to verify UART communication
"""
import serial
import sys
import time

if len(sys.argv) < 2:
    print("Usage: python3 serial_monitor.py /dev/ttyUSB0 [baudrate]")
    sys.exit(1)

port = sys.argv[1]
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

print(f"Opening {port} at {baud} baud...")
print("Press Ctrl+C to exit\n")

try:
    with serial.Serial(port, baud, timeout=1) as ser:
        time.sleep(0.5)  # Let the board reset
        
        print("Listening for data...\n")
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                try:
                    text = data.decode('ascii', errors='replace')
                    print(text, end='', flush=True)
                except:
                    print(f"[HEX: {data.hex()}]", flush=True)
            time.sleep(0.01)
            
except KeyboardInterrupt:
    print("\n\nExiting...")
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)
