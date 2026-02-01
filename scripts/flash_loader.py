#!/usr/bin/env python3
"""
UART Flash Loader for STM32F103C8

This script sends firmware to the bootloader using XModem protocol with CRC-16.

Usage:
    python3 flash_loader.py -p /dev/ttyUSB0 -f application.bin

Requirements:
    pip install pyserial
"""

import argparse
import serial
import time
import struct
import sys
from pathlib import Path

# XModem Control Characters
SOH = 0x01  # Start of Header
EOT = 0x04  # End of Transmission
ACK = 0x06  # Acknowledge
NAK = 0x15  # Negative Acknowledge
CAN = 0x18  # Cancel

FRAME_SIZE = 128
UART_TIMEOUT = 2.0
MAX_RETRIES = 3


def crc16_xmodem(data):
    """
    Calculate XModem CRC-16 (polynomial 0x1021)
    
    Args:
        data: bytes to calculate CRC for
        
    Returns:
        16-bit CRC value
    """
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc <<= 1
            if crc & 0x10000:
                crc ^= 0x1021
            crc &= 0xFFFF
    return crc


def send_firmware(port, firmware_file, verbose=False):
    """
    Send firmware to bootloader via XModem protocol
    
    Args:
        port: Serial port (e.g., '/dev/ttyUSB0' or 'COM3')
        firmware_file: Path to firmware binary file
        verbose: Print detailed debug information
        
    Returns:
        True on success, False on failure
    """
    
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
    
    # Pad to multiple of 128 bytes (XModem requirement)
    if len(firmware) % FRAME_SIZE:
        pad_size = FRAME_SIZE - (len(firmware) % FRAME_SIZE)
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
    
    block_num = 1
    frame_count = len(firmware) // FRAME_SIZE
    success = False
    
    try:
        # Clear any pending data in the buffer
        ser.reset_input_buffer()
        
        # Wait for the "BOOT" message
        print("Waiting for bootloader...")
        
        # Custom read_until with timeout for compatibility
        line = bytearray()
        start_time = time.time()
        boot_found = False
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                c = ser.read(1)
                if c:
                    line.extend(c)
                    # Look for BOOT anywhere in the received data
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

        # Clear any remaining garbage from the buffer
        time.sleep(0.2)
        ser.reset_input_buffer()

        # Initiate bootloader in CRC mode
        print("Initiating transfer (sending 'C')...")
        ser.write(b'C')
        ser.flush()
        time.sleep(0.1)  # Give bootloader time to process the 'C'
        
        # Send firmware blocks
        print(f"Starting transfer ({frame_count} blocks)...")
        
        for block_idx in range(frame_count):
            offset = block_idx * FRAME_SIZE
            block_data = firmware[offset:offset + FRAME_SIZE]
            
            assert len(block_data) == FRAME_SIZE, "Block size mismatch"
            
            # Calculate CRC
            crc = crc16_xmodem(block_data)
            
            # Build XModem frame: SOH BlockNum ~BlockNum Data CRC_H CRC_L
            frame = bytes([SOH, block_num, (~block_num) & 0xFF])
            frame += block_data
            frame += struct.pack('>H', crc)
            
            if verbose:
                print(f"\nBlock {block_num} (offset 0x{offset:06x}):")
                print(f"  Data CRC-16: 0x{crc:04x}")
                print(f"  Frame size: {len(frame)} bytes")
                print(f"  Frame hex: {frame[:20].hex()}... (first 20 bytes)")
                print(f"  Block field: SOH=0x01, BlockNum=0x{block_num:02x}, ~BlockNum=0x{(~block_num)&0xFF:02x}")
            
            # Send frame with retries
            retry_count = 0
            while retry_count < MAX_RETRIES:
                ser.write(frame)
                ser.flush()
                time.sleep(0.05)  # Give bootloader time to process
                
                # Wait for response
                response = ser.read(1)
                
                if response == bytes([ACK]):
                    print(f"Block {block_num:3d}/{frame_count}: OK", end='\r')
                    sys.stdout.flush()
                    block_num = (block_num + 1) & 0xFF
                    if block_num == 0:
                        block_num = 1 # Wrap around
                    break
                    
                elif response == bytes([NAK]):
                    retry_count += 1
                    if verbose:
                        print(f"Block {block_num}: NAK received (retry {retry_count}/{MAX_RETRIES})")
                    
                    if retry_count >= MAX_RETRIES:
                        print(f"\nError: Block {block_num} failed after {MAX_RETRIES} retries")
                        return False
                    
                elif response == bytes([CAN]):
                    print(f"\nError: Bootloader cancelled transfer at block {block_num}")
                    return False
                    
                else:
                    retry_count += 1
                    if verbose:
                        if response:
                            print(f"Block {block_num}: Unexpected response: 0x{response[0]:02x} (retry {retry_count}/{MAX_RETRIES})")
                        else:
                            print(f"Block {block_num}: Timeout (retry {retry_count}/{MAX_RETRIES})")
                    
                    if retry_count >= MAX_RETRIES:
                        print(f"\nError: Block {block_num} timeout after {MAX_RETRIES} retries")
                        return False
        
        print(f"\nBlock {frame_count:3d}/{frame_count}: OK")
        
        # Send End-of-Transmission
        print("Sending EOT (End of Transmission)...")
        ser.write(bytes([EOT]))
        ser.flush()
        
        # Wait for final ACK
        response = ser.read(1)
        if response == bytes([ACK]):
            print("✓ Firmware transfer complete!")
            print("✓ Bootloader will now jump to application at 0x08002000")
            success = True
        else:
            if response:
                print(f"Warning: Expected ACK after EOT, got 0x{ord(response):02x} instead")
            else:
                print("Warning: No response after EOT.")

    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")
    
    return success


def main():
    """
    Main function to parse arguments and start flash loader
    """
    parser = argparse.ArgumentParser(description="UART Flash Loader for STM32F103C8")
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


def main():
    """Parse arguments and run flash loader"""
    parser = argparse.ArgumentParser(
        description='UART Flash Loader for STM32F103C8 Bootloader',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 flash_loader.py -p /dev/ttyUSB0 -f application.bin
  python3 flash_loader.py -p COM3 -f firmware.bin -v
        """
    )
    
    parser.add_argument('-p', '--port', required=True,
                        help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-f', '--firmware', required=True,
                        help='Firmware binary file path')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose output for debugging')
    
    args = parser.parse_args()
    
    # Check firmware file exists
    if not Path(args.firmware).exists():
        print(f"Error: Firmware file not found: {args.firmware}")
        sys.exit(1)
    
    # Send firmware
    if send_firmware(args.port, args.firmware, args.verbose):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
