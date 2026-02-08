#!/usr/bin/env python3
"""
Simple Flash Loader for STM32F103C8 - Simplified Protocol

Protocol (Handshake-driven, 8-byte chunks):
1. Bootloader sends "BOOT\r\n" every 500ms
2. Host sends 'S' to start transfer
3. For each 8-byte chunk:
   - Host sends 'D' + 8 bytes + 1 checksum byte
   - Bootloader verifies checksum and sends ACK or NAK
   - Host waits for ACK before sending next chunk
   - Bootloader accumulates 128 chunks (1KB) before writing to flash
4. Host sends 'E' to end transfer (bootloader writes remaining data with 0xFF padding)
5. Bootloader jumps to application

Usage:
    python3 simple_flash.py -p /dev/ttyUSB0 -f application.bin
"""

import argparse
import serial
import time
import sys
from pathlib import Path

# Protocol commands
CMD_START = ord('S')
CMD_DATA = ord('D')
CMD_END = ord('E')
ACK = 0x06
NAK = 0x15

CHUNK_SIZE = 8       # 8 bytes per chunk
UART_TIMEOUT = 1.0   # 1 second timeout
MAX_RETRIES = 3


def simple_checksum(data):
    """Calculate simple checksum (sum of all bytes)"""
    return sum(data) & 0xFF


def send_firmware(port, firmware_file, verbose=False):
    """Send firmware to bootloader"""
    
    # Read firmware
    try:
        with open(firmware_file, 'rb') as f:
            firmware = f.read()
    except FileNotFoundError:
        print(f"Error: Firmware file '{firmware_file}' not found")
        return False
    
    if not firmware:
        print("Error: Firmware file is empty")
        return False
    
    print(f"Firmware size: {len(firmware)} bytes")
    
    # Pad to multiple of CHUNK_SIZE bytes
    if len(firmware) % CHUNK_SIZE:
        pad_size = CHUNK_SIZE - (len(firmware) % CHUNK_SIZE)
        firmware += b'\xFF' * pad_size
        if verbose:
            print(f"Padded firmware to {len(firmware)} bytes (added {pad_size} bytes)")
    
    # Open serial port
    try:
        ser = serial.Serial(port, 115200, timeout=UART_TIMEOUT)
        print(f"Connected to {port} at 115200 baud")
    except serial.SerialException as e:
        print(f"Error: Cannot open port '{port}': {e}")
        return False
    
    chunk_count = len(firmware) // CHUNK_SIZE
    success = False
    
    try:
        # Wait for bootloader heartbeat
        print("Waiting for bootloader...")
        
        line = bytearray()
        start_time = time.time()
        boot_found = False
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                c = ser.read(1)
                if c:
                    line.extend(c)
                    if b'BOOT' in bytes(line):
                        boot_found = True
                        break
            time.sleep(0.01)

        if not boot_found:
            print("Error: Bootloader not detected.")
            if verbose:
                print(f"  Received: {bytes(line).hex()}")
            return False
        print("Bootloader detected.")
        
        # Wait for any remaining heartbeat messages and clear buffer
        time.sleep(0.6)
        ser.reset_input_buffer()
        time.sleep(0.1)
        ser.reset_input_buffer()
        
        print("Sending START command...")
        ser.write(bytes([CMD_START]))
        ser.flush()
        
        # Wait for ACK
        response = ser.read(1)
        if response != bytes([ACK]):
            print(f"Error: Expected ACK after START, got 0x{response[0]:02x}" if response else "Error: No response to START")
            return False
        print("START acknowledged.")
        
        # Send firmware in 8-byte chunks
        print(f"Starting transfer ({chunk_count} chunks = {len(firmware)} bytes)...")
        
        for chunk_idx in range(chunk_count):
            offset = chunk_idx * CHUNK_SIZE
            chunk_data = firmware[offset:offset + CHUNK_SIZE]
            
            assert len(chunk_data) == CHUNK_SIZE, "Chunk size mismatch"
            
            # Calculate checksum
            checksum = simple_checksum(chunk_data)
            
            if verbose and chunk_idx % 128 == 0:  # Print every 1KB
                print(f"\nChunk {chunk_idx + 1} (offset 0x{offset:06x}):")
                print(f"  Checksum: 0x{checksum:02x}")
                print(f"  Data: {chunk_data.hex()}")
            
            # Send chunk with retries (handshake-driven)
            retry_count = 0
            while retry_count < MAX_RETRIES:
                # Send DATA command + 8 bytes + checksum
                ser.write(bytes([CMD_DATA]))
                ser.write(chunk_data)
                ser.write(bytes([checksum]))
                ser.flush()
                
                # Wait for ACK/NAK response
                # Use longer timeout every 128 chunks (1KB page write)
                if chunk_idx > 0 and chunk_idx % 128 == 0:
                    timeout = 3.0  # Flash page write takes longer
                else:
                    timeout = 1.0  # Normal chunk
                ser.timeout = timeout
                response = ser.read(1)
                ser.timeout = UART_TIMEOUT
                
                if response == bytes([ACK]):
                    if chunk_idx % 128 == 0 or chunk_idx == chunk_count - 1:
                        print(f"Chunk {chunk_idx + 1:5d}/{chunk_count} ({offset:5d} bytes): OK")
                    break
                    
                elif response == bytes([NAK]):
                    retry_count += 1
                    if verbose:
                        print(f"Chunk {chunk_idx + 1}: NAK received (retry {retry_count}/{MAX_RETRIES})")
                    
                    if retry_count >= MAX_RETRIES:
                        print(f"\nError: Chunk {chunk_idx + 1} failed after {MAX_RETRIES} retries")
                        return False
                    
                else:
                    retry_count += 1
                    if verbose:
                        if response:
                            print(f"Chunk {chunk_idx + 1}: Unexpected response: 0x{response[0]:02x} (retry {retry_count}/{MAX_RETRIES})")
                        else:
                            print(f"Chunk {chunk_idx + 1}: Timeout waiting for ACK (retry {retry_count}/{MAX_RETRIES})")
                    
                    if retry_count >= MAX_RETRIES:
                        print(f"\nError: Chunk {chunk_idx + 1} failed after {MAX_RETRIES} retries")
                        return False
        
        print(f"\nAll {chunk_count} chunks transferred successfully!")
        
        # Send END command
        print("Sending END command...")
        ser.write(bytes([CMD_END]))
        ser.flush()
        
        # Wait for final ACK
        response = ser.read(1)
        if response == bytes([ACK]):
            print("✓ Firmware transfer complete!")
            print("✓ Bootloader will now jump to application at 0x08002000")
            success = True
        else:
            if response:
                print(f"Warning: Expected ACK after END, got 0x{response[0]:02x}")
            else:
                print("Warning: No response after END.")

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")
    
    return success


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Simple Flash Loader for STM32F103C8")
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("-f", "--file", required=True, help="Firmware binary file")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")
    
    args = parser.parse_args()
    
    firmware_path = Path(args.file)
    if not firmware_path.is_file():
        print(f"Error: File not found: {firmware_path}")
        sys.exit(1)
        
    if send_firmware(args.port, firmware_path, args.verbose):
        print("Flash loading successful.")
        sys.exit(0)
    else:
        print("Flash loading failed.")
        sys.exit(1)


if __name__ == "__main__":
    main()
