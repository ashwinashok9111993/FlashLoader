# STM32F103 UART Flash Loader

UART bootloader for STM32F103C8 with multiple flash methods: Python script (XModem-CRC) and ESP32 web interface.

## 🚀 Flash Methods

### Method 1: Python Script (USB/Serial)
Traditional USB-to-UART flashing using Python and XModem protocol.

### Method 2: ESP32 WiFi Flash Loader (NEW!)
**Web-based wireless flashing** - Upload firmware through a browser interface!
- 🌐 Beautiful web UI with drag & drop
- 📊 Real-time progress tracking  
- 📡 WiFi connectivity (AP or Station mode)
- ⚡ No USB cable needed after setup

👉 **See [esp32_FOTA_app/esp32_stm32_FlashLoader](esp32_FOTA_app/esp32_stm32_FlashLoader)** for complete documentation.

## Hardware
- **MCU**: STM32F103C8 (64KB Flash, 20KB RAM)
- **UART**: PA9(TX)/PA10(RX), 115200 baud, 8N1
- **Memory**: 8KB bootloader (0x08000000), 56KB app space (0x08002000)

## Quick Start

### Using Python Script
```bash
# 1. Build and flash bootloader
cd Simple_BootLoader && make && st-flash write build/Simple_BootLoader.bin 0x08000000

# 2. Build application  
cd App1 && make

# 3. Flash via UART
python3 scripts/flash_loader.py -p /dev/ttyUSB0 -f App1/build/app.bin
```

### Using ESP32 Web Interface
```bash
# 1. Setup ESP32 flash loader
cd esp32_FOTA_app/esp32_stm32_FlashLoader
pio run -t upload      # Upload ESP32 firmware
pio run -t uploadfs    # Upload web interface

# 2. Connect to WiFi "ESP32_FlashLoader" (password: flashloader123)

# 3. Open browser: http://192.168.4.1

# 4. Upload & flash your firmware!
```

## Protocol
- Boot: Sends "BOOT" every 500ms for 5 seconds, then jumps to app
- Transfer: 8-byte chunks with checksum validation
- Commands: 'S' (start), 'D'+data+checksum (chunk), 'E' (end)

## Code Pitfalls and Solutions

## Common Issues

### HAL_Delay() Hangs After Bootloader Jump
Use busy loops instead:
```c
// Replace HAL_Delay(1000) with:
for (volatile uint32_t i = 0; i < 7200000; i++) __NOP();
```

### Flash Transfer Errors  
- Check ground connections and cable quality
- Clear UART buffer before sending START command
- Verify baud rate matches (115200)

### Application Won't Start
- Validate vector table before jumping
- Avoid HAL_Delay() in application code

## Key Files
- **Bootloader**: [Simple_BootLoader/Core/Src/main.c](Simple_BootLoader/Core/Src/main.c)
- **Application**: [App1/Core/Src/main.c](App1/Core/Src/main.c)
- **Flash tool**: [scripts/simple_flash.py](scripts/simple_flash.py)

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
