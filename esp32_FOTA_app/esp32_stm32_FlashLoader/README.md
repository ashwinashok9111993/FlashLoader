# ESP32 WiFi-based STM32 Flash Loader

A web-based firmware flash loader for STM32 microcontrollers running on ESP32. Upload hex/bin files via a web interface and flash them wirelessly to your STM32 device using the XModem protocol over UART.

## Features

- 🌐 **Web Interface**: Beautiful, responsive web UI for file upload
- 📡 **WiFi Connectivity**: Works in both Access Point and Station modes
- 📁 **File Upload**: Support for .bin and .hex firmware files
- 📊 **Real-time Progress**: Live progress tracking during flash operation
- 🔄 **XModem Protocol**: Reliable firmware transfer with CRC-16 validation
- ⚡ **Async Operation**: Non-blocking web server for smooth performance

## Hardware Requirements

- ESP32 development board (DevKit-V1 or compatible)
- STM32 microcontroller with custom bootloader
- Connection wires

## Hardware Connections

```
ESP32 GPIO17 (TX) ──► STM32 RX (e.g., PA10)
ESP32 GPIO16 (RX) ◄── STM32 TX (e.g., PA9)
ESP32 GND        ──── STM32 GND
```

## Software Requirements

- [PlatformIO](https://platformio.org/) installed in VS Code
- Python 3.x (for filesystem upload)

## Installation & Setup

### 1. Clone/Open the Project

Open the project folder in VS Code with PlatformIO extension installed.

### 2. Configure WiFi Settings

Edit `src/main.cpp` to configure your WiFi settings:

```cpp
// For Access Point mode (default)
#define WIFI_MODE_AP true
const char* AP_SSID = "ESP32_FlashLoader";
const char* AP_PASSWORD = "flashloader123";

// For Station mode (connect to existing WiFi)
// Set WIFI_MODE_AP to false and configure:
const char* STA_SSID = "YourWiFiSSID";
const char* STA_PASSWORD = "YourWiFiPassword";
```

### 3. Build and Upload Firmware

```bash
# Build the project
pio run

# Upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor
```

### 4. Upload Filesystem (Web Interface)

The web interface HTML file needs to be uploaded to the ESP32's LittleFS filesystem:

```bash
# Upload filesystem
pio run -t uploadfs
```

**Note**: Make sure the ESP32 is connected and no serial monitor is running.

### 5. Access the Web Interface

After successful upload, the ESP32 will start and display connection information on the serial monitor:

#### Access Point Mode
- Connect to WiFi: `ESP32_FlashLoader` (password: `flashloader123`)
- Open browser: `http://192.168.4.1`

#### Station Mode
- ESP32 connects to your WiFi
- Check serial monitor for assigned IP address
- Open browser: `http://<ESP32_IP_ADDRESS>`

## Usage

### Flashing STM32 Firmware

1. **Prepare STM32**: Ensure your STM32 is running the bootloader and connected via UART to ESP32
2. **Reset STM32**: Put STM32 into bootloader mode (should send "BOOT" message)
3. **Open Web Interface**: Navigate to the ESP32's IP address in your browser
4. **Select Firmware**: Click the upload area or drag & drop your .bin/.hex file
5. **Flash**: Click "Flash Firmware" button
6. **Monitor Progress**: Watch real-time progress and status messages
7. **Complete**: Wait for success message indicating flash completion

### Web Interface Features

- **Drag & Drop**: Simply drag firmware files onto the upload area
- **Progress Tracking**: Real-time progress bar showing upload and flash status
- **Connection Status**: Visual indicator showing ESP32 connectivity
- **Status Messages**: Clear feedback for every operation
- **Reset Button**: Quickly reset the interface for a new flash operation

## API Endpoints

The ESP32 exposes these REST API endpoints:

### GET `/api/status`
Returns current flash operation status
```json
{
  "state": "idle|in_progress|success|failed",
  "progress": 0-100,
  "message": "Status message"
}
```

### POST `/api/upload`
Upload firmware file (multipart/form-data)
- Accepts .bin and .hex files
- Automatically starts flashing after upload

### POST `/api/reset`
Reset flash state to idle

## XModem Protocol Implementation

The flash loader uses XModem-CRC protocol for reliable data transfer:

- **Frame Size**: 128 bytes
- **CRC**: 16-bit CRC-XMODEM (polynomial 0x1021)
- **Retries**: Up to 3 retries per block
- **Flow Control**: ACK/NAK/EOT handshaking

### Protocol Flow

1. ESP32 waits for "BOOT" message from STM32 bootloader
2. ESP32 sends 'C' to initiate CRC mode
3. For each 128-byte block:
   - Send: SOH + BlockNum + ~BlockNum + Data[128] + CRC_H + CRC_L
   - Wait for: ACK (success) or NAK (retry)
4. Send EOT (End of Transmission)
5. Wait for final ACK

## Troubleshooting

### ESP32 Not Starting
- Check serial monitor for error messages
- Verify USB cable and connection
- Try pressing reset button on ESP32

### Cannot Connect to WiFi
- **AP Mode**: Verify SSID and password in code
- **Station Mode**: Check WiFi credentials and router connectivity
- Look for fallback AP mode in serial output

### Web Interface Not Loading
- Verify filesystem was uploaded: `pio run -t uploadfs`
- Check LittleFS initialization in serial monitor
- Clear browser cache or try incognito mode

### Flash Operation Fails
- Ensure STM32 bootloader is running and sending "BOOT"
- Verify UART connections (TX/RX not swapped, GND connected)
- Check baud rate matches (default: 115200)
- Verify STM32 is in bootloader mode (not running application)
- Check firmware file is valid and not corrupted

### Upload Timeout
- Check network connectivity
- Try smaller firmware files first
- Increase timeout values in code if needed

### "Bootloader not detected"
- Verify STM32 UART pins (default: GPIO16=RX, GPIO17=TX)
- Reset STM32 into bootloader mode
- Check if bootloader sends "BOOT" message on startup

## Configuration Options

### Change UART Pins
Edit in `src/main.cpp`:
```cpp
#define STM32_RX_PIN 16  // ESP32 RX (from STM32 TX)
#define STM32_TX_PIN 17  // ESP32 TX (to STM32 RX)
```

### Change Baud Rate
```cpp
#define STM32_BAUD 115200
```

### Adjust Retry Settings
```cpp
#define MAX_RETRIES 3  // Number of retries per block
```

## File Structure

```
esp32_stm32_FlashLoader/
├── src/
│   └── main.cpp              # Main ESP32 application
├── data/
│   └── index.html            # Web interface
├── include/
├── lib/
├── platformio.ini            # PlatformIO configuration
└── README.md                 # This file
```

## Technical Details

### Memory Usage
- Firmware buffer: Allocated dynamically based on upload size
- Web server: Async architecture for efficient memory use
- LittleFS: ~256KB reserved for filesystem

### Performance
- Upload speed: ~100KB/s (depends on network)
- Flash speed: ~10-15 seconds for typical 32KB firmware
- Progress updates: Every 500ms during flash operation

### Security Notes
- Default AP password should be changed for production
- No authentication implemented (add if needed)
- Files stored temporarily in RAM during upload

## Credits

Based on the original Python flash loader script with enhanced web-based functionality.

## License

This project is provided as-is for educational and development purposes.

## Support

For issues or questions:
1. Check serial monitor output for detailed debug information
2. Verify all hardware connections
3. Ensure STM32 bootloader is compatible with XModem-CRC protocol
4. Test with the original Python script first to verify bootloader functionality
