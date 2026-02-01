# STM32F103 UART Flash Loader

Simple UART-based bootloader for STM32F103C8 using handshake-driven 8-byte chunk protocol.

## Hardware
- **MCU**: STM32F103C8 (64KB Flash, 20KB RAM)
- **UART**: USART1 on PA9(TX)/PA10(RX), 115200 baud, 8N1
- **Programmer**: STLink V2
- **Serial**: /dev/ttyUSB0 (USB-UART adapter)

## Memory Map
```
0x08000000 - 0x08001FFF  (8 KB)   Bootloader
0x08002000 - 0x0800FFFF  (56 KB)  Application Space
```

## Features
- **5-second auto-boot timeout**: Automatically jumps to application if no flashing activity
- **Handshake-driven protocol**: ACK after each 8-byte chunk ensures reliable transfer
- **Application validation**: Checks vector table validity before jumping
- **Small footprint**: Bootloader ~6.3 KB, leaves 56 KB for application
- **Reliable flash writing**: 1KB page buffering before flash operations

## Quick Start

### 1. Install Dependencies
```bash
# Install Python serial library
sudo apt install python3-serial

# Or use pip (if not externally managed)
pip3 install pyserial
```

### 2. Build and Flash Bootloader
```bash
cd Simple_BootLoader
make clean && make
st-flash write build/Simple_BootLoader.bin 0x08000000
st-flash reset
```

Expected output: ~6.3 KB bootloader binary

### 3. Build Application
```bash
cd App1
make clean && make
```

Expected output: ~4.4 KB application binary at `build/Simple_BootLoader.bin`

### 4. Test Auto-Boot Feature
The bootloader has a 5-second timeout. If no START command is received, it automatically jumps to the application.

```bash
# Reset and watch the transition
st-flash reset
sleep 1
python3 scripts/serial_monitor.py /dev/ttyUSB0 115200

# You should see:
# BOOT      (repeated for ~5 seconds)
# BOOT
# === APP1 @ 0x08002000 ===
# APP1      (repeated every second)
```

### 5. Flash Application via UART Bootloader
```bash
# Reset device and flash within 5 seconds
st-flash reset
sleep 1
python3 scripts/simple_flash.py -p /dev/ttyUSB0 -f App1/build/Simple_BootLoader.bin

# Monitor application after flashing
sleep 2
python3 scripts/serial_monitor.py /dev/ttyUSB0 115200
```

## Protocol Details

### Handshake-Driven 8-Byte Chunk Protocol

#### Boot Sequence
1. Bootloader starts, sends "BOOT\r\n" every 500ms
2. After 5 seconds with no START command, jumps to application
3. Host sends 'S' (0x53) to start firmware transfer
4. Bootloader responds with ACK (0x06)

#### Data Transfer
For each 8-byte chunk:
```
Host:       'D' (0x44) + [8 data bytes] + [1 checksum byte]
Bootloader: ACK (0x06) or NAK (0x15)
```

Checksum: Simple sum of 8 bytes, masked to 8 bits: `sum(bytes) & 0xFF`

#### End Transfer
```
Host:       'E' (0x45)
Bootloader: ACK (0x06) then jump    # Bootloader (8 KB @ 0x08000000)
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h              # Main header
│   │   │   ├── usart.h             # UART definitions
│   │   │   └── gpio.h              # GPIO definitions
│   │   └── Src/
│   │       ├── main.c              # Bootloader logic
│   │       ├── usart.c             # UART initialization
│   │       └── gpio.c              # GPIO initialization
│   ├── STM32F103XX_FLASH_UNIFIED.ld # Linker (8 KB @ 0x08000000)
│   ├── Makefile                    # Build configuration
│   └── build/                      # Build outputs
│       └── Simple_BootLoader.bin   # Final binary
├── App1/                           # Application (56 KB @ 0x08002000)
│   ├── Core/Src/main.c             # Application with LED + UART
│   ├── STM32F103XX_APP1.ld         # Linker (56 KB @ 0x08002000)
│   └── build/
│       └── Simple_BootLoader.bin   # Application binary
├── scripts/
│   ├── simple_flash.py             # UART flash tool
│   └── serial_monitor.py           # UART monitor
└── README.md                       # This file
```

## Code Pitfalls and Solutions

### 1. HAL_Delay() Not Working in Application
**Problem**: After bootloader jumps to application, HAL_Delay() hangs forever.

**Cause**: SysTick interrupt not configured after vector table relocation.

**Solution**: Use busy-wait loops instead:
```c
// BAD - Will hang
HAL_Delay(1000);

// GOOD - Works reliably
volatile uint32_t i;
for (i = 0; i < 7200000; i++) {
    __NOP();  // ~1 second at 72 MHz
}
```

### 2. Debug Output on Same UART
**Problem**: Debug messages corrupt protocol data.

**Solution**: Use `#define DEBUG_OUTPUT 0` in bootloader:
```c
#define DEBUG_OUTPUT 0  // MUST be 0 for production

#if DEBUG_OUTPUT
static void debug_print(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
#else
#define debug_print(msg) /* No debug output */
#endif
```

### 3. Timeout Detection with 0xFF Data
**Problem**: Using `uart_recv_byte()` returning 0xFF on timeout conflicts with valid 0xFF data.

**Solution**: Use HAL status codes:
```c
// BAD - 0xFF is valid data
uint8_t byte = uart_recv_byte(1000);
if (byte == 0xFF) {  // Could be timeout OR data!
    // error handling
}

// GOOD - Check HAL status
if (HAL_UART_Receive(&huart1, chunk, 8, 5000) != HAL_OK) {
    // Definitely a timeout
    uart_send_byte(NAK);
}
```

### 4. Flash Page Alignment
**Problem**: STM32F103 requires 1 KB page alignment for erase operations.

**Solution**: Buffer chunks in RAM, write complete pages:
```c
static uint8_t page_buffer[1024];  // 1 KB in RAM
uint16_t page_idx = 0;

// Accumulate 8-byte chunks
memcpy(&page_buffer[page_idx], chunk, 8);
page_idx += 8;

// Write when full
if (page_idx >= 1024) {
    flash_write_page(flash_addr, page_buffer, 1024);
    flash_addr += 1024;
    page_idx = 0;
    memset(page_buffer, 0xFF, 1024);  // Reset for next page
}
```

### 5. Application Vector Table Validation
**Problem**: Jumping to corrupt/empty application region causes hard fault.

**Solution**: Always validate before jumping:
```c
static int check_application_valid(void) {
    uint32_t sp = *(volatile uint32_t*)(0x08002000 + 0);
    uint32_t reset = *(volatile uint32_t*)(0x08002000 + 4);
    
    // Check stack pointer in RAM
    if (sp < 0x20000000 || sp > 0x20005000) return 0;
    
    // Check reset vector in flash with Thumb bit
    if ((reset < 0x08002000) || ((reset & 0x1) == 0)) return 0;
    
    return 1;
}
```

### 6. UART Buffer Overflow
**Problem**: Bootloader heartbeat floods buffer, corrupting START command.

**Solution**: Clear RX buffer before and after sending START:
```c
// Host side (Python)
time.sleep(0.6)  # Wait for heartbeat cycle
ser.reset_input_buffer()
time.sleep(0.1)
ser.reset_input_buffer()  # Clear again

ser.write(bytes([CMD_START]))
```

## Development

### Bootloader Size
Current: 6,332 bytes / 8,192 bytes (77% used)

### Application Size
Current: 4,432 bytes / 57,344 bytes (7.7% used)

### Modifying Bootloader
```bash
cd Simple_BootLoader
# Edit Core/Src/main.c
make clean && make
st-flash write build/Simple_BootLoader.bin 0x08000000
st-flash reset
```

### Changing Timeout Period
Edit `BOOTLOADER_TIMEOUT` in main.c:
```c
#define BOOTLOADER_TIMEOUT  5000  // milliseconds
```

### Adding Debug Output
Only for development, disable for production:
```c
#define DEBUG_OUTPUT 1  // Enable
// Rebuild and reflash bootloader
```

## Testing Commands

### Monitor Bootloader Heartbeat
```bash
python3 scripts/serial_monitor.py /dev/ttyUSB0 115200
# Should see "BOOT" every 500ms for 5 seconds, then "APP1"
```

### Verify Auto-Boot
```bash
st-flash reset
sleep 7
python3 scripts/serial_monitor.py /dev/ttyUSB0 115200
# Should see only "APP1" (auto-booted)
```

### Flash New Firmware
```bash
# Quick flash (within 5-second window)
st-flash reset && sleep 1 && python3 scripts/simple_flash.py -p /dev/ttyUSB0 -f App1/build/Simple_BootLoader.bin
```

### Read Flash Contents
```bash
# Read bootloader
st-flash read /tmp/bootloader.bin 0x08000000 0x2000
hexdump -C /tmp/bootloader.bin | head -20

# Read application
st-flash read /tmp/app.bin 0x08002000 0x2000
hexdump -C /tmp/app.bin | head -20
```

### Erase Application (Force Bootloader Mode)
```bash
st-flash erase  # Erases entire flash including application
st-flash write Simple_BootLoader/build/Simple_BootLoader.bin 0x08000000
st-flash reset
# Bootloader will keep sending "BOOT" (no valid app)
```

## Performance

- **Transfer rate**: ~700 bytes/second (115200 baud, 8-byte chunks)
- **Flash time**: ~6 seconds for 4 KB application
- **Page write time**: ~100-200ms per 1 KB page
- **Bootloader startup**: <100ms
- **Auto-boot timeout**: 5 seconds
2. Timeout: Bootloader didn't receive all 8 bytes
3. Flash write failure: Application region locked

**Solutions**:
- Use shorter, shielded USB-UART cable
- Lower baud rate (modify both bootloader and script)
- Check for ground loops
- Verify flash not write-protected

### Application Doesn't Start After Flash
**Symptom**: Bootloader keeps sending "BOOT" heartbeat after flashing

**Causes**:
1. Application uses HAL_Delay() - SysTick won't work after bootloader jump
2. Invalid vector table
3. Application crash at startup

**Solution**: Use busy-wait loops instead of HAL_Delay():
```c
// DON'T USE: HAL_Delay(1000);

// USE THIS:
volatile uint32_t i;
for (i = 0; i < 7200000; i++) {
    __NOP();
}
```

### Build Errors

**Linker error "region FLASH overflowed"**:
- Bootloader: Check STM32F103XX_FLASH_UNIFIED.ld, max 8 KB
- Application: Check STM32F103XX_APP1.ld, max 56 KB at 0x08002000

**"arm-none-eabi-gcc not found"**:
```bash
sudo apt install gcc-arm-none-eabi
```

**Missing HAL drivers**:
Ensure `Drivers/STM32F1xx_HAL_Driver/` exists with source files

## Project Structure
```
FlashLoader/
├── Simple_BootLoader/          # Bootloader project
│   ├── Core/Src/main.c         # Bootloader logic
│   ├── Core/Src/usart.c        # UART configuration
│   ├── STM32F103XX_FLASH.ld    # Linker script (8KB)
│   ├── test_main.c             # Hardware test (LED + UART)
│   └── build/                  # Build outputs
├── App1/                       # Application project
│   ├── Core/Src/main.c         # Application code
│   ├── STM32F103XX_APP1.ld     # Linker script (56KB @ 0x08002000)
│   └── build/                  # Build outputs
├── scripts/
│   ├── flash_loader.py         # XModem flash tool
│   └── test_uart.py            # Bootloader test
└── README.md                   # This file
```

## Protocol Details

### XModem-CRC Frame Structure
```
[SOH] [BlkNum] [~BlkNum] [128 data bytes] [CRC_H] [CRC_L]
 0x01   0-255    255-0      payload         16-bit CRC
```

### Control Characters
- **SOH** = 0x01 (Start of Header)
- **ACK** = 0x06 (Acknowledge)
- **NAK** = 0x15 (Negative Acknowledge)
- **EOT** = 0x04 (End of Transmission)
- **CAN** = 0x18 (Cancel)

### Flash Programming
1. Bootloader waits for 'C' character (CRC mode)
2. Host sends firmware in 128-byte frames
3. Bootloader validates CRC-16 (polynomial 0x1021)
4. Erases 1KB pages, programs halfwords
5. Sends ACK for good frame, NAK for errors
6. Host sends EOT when complete
7. Bootloader jumps to application at 0x08002000

## Development

### Bootloader Size
Current: ~6.1 KB (limit 8 KB)

### Application Size  
Current: ~4.4 KB (limit 56 KB)

### Adding Features
- Edit `Simple_BootLoader/Core/Src/main.c` for bootloader changes
- Rebuild: `cd Simple_BootLoader && make`
- Reflash: `st-flash write build/Simple_BootLoader.bin 0x08000000`

### Debugging
- Use `test_uart.py` to verify bootloader responds
- Check CRC calculations match between Python and C
- Monitor with oscilloscope for baud rate verification
- Add debug markers: `uart_send_byte(0xAA);` in code

## License
Educational project - use freely.
