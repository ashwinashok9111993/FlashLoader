/*
 * ESP32 WiFi-based STM32 Flash Loader
 * 
 * This application creates a web server that allows uploading hex/bin files
 * and flashing them to an STM32 microcontroller via UART using XModem protocol.
 * 
 * Hardware connections:
 *   ESP32 TX (GPIO17) -> STM32 RX
 *   ESP32 RX (GPIO16) -> STM32 TX
 *   ESP32 GND -> STM32 GND
 * 
 * Access the web interface at: http://192.168.4.1 (AP mode) or configured IP (STA mode)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Configuration
#define WIFI_MODE_AP true  // Set to false for Station mode
const char* AP_SSID = "ESP32_FlashLoader";
const char* AP_PASSWORD = "flashloader123";

// Station mode credentials (if WIFI_MODE_AP is false)
const char* STA_SSID = "";
const char* STA_PASSWORD = "";

// UART configuration for STM32 communication
#define STM32_UART_NUM 2
#define STM32_RX_PIN 16
#define STM32_TX_PIN 17
#define STM32_BAUD 115200

#define CHUNK_SIZE 8
// Simple bootloader protocol constants (8-byte chunks)
#define CMD_START 'S'
#define CMD_DATA  'D'
#define CMD_END   'E'
// XModem protocol constants (kept for compatibility if needed)
#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18
#define FRAME_SIZE 128
#define MAX_RETRIES 8

// Global objects
AsyncWebServer server(80);
HardwareSerial STM32Serial(STM32_UART_NUM);

// Flash state
enum FlashState {
  IDLE,
  IN_PROGRESS,
  SUCCESS,
  FAILED
};

volatile FlashState flashState = IDLE;
String flashMessage = "";
volatile int flashProgress = 0;

// Function declarations
uint16_t crc16_xmodem(const uint8_t* data, size_t length);
bool flashSTM32(const uint8_t* firmware, size_t firmwareSize);
bool hexToBinary(const uint8_t* hexData, size_t hexSize, uint8_t* binBuffer, size_t* binSize);
uint8_t hexToNibble(char c);

// Flash job passed to background task
typedef struct {
  uint8_t* data;
  size_t size;
} FlashJob;

// Background task that runs the blocking flash operation
void flashTask(void* pvParameters) {
  FlashJob* job = (FlashJob*)pvParameters;
  if (job && job->data && job->size > 0) {
    flashSTM32(job->data, job->size);
    free(job->data);
  }
  if (job) free(job);
  vTaskDelete(NULL);
}
String getFlashStatusJSON();

// ============================================================================
// Intel HEX to Binary Converter
// ============================================================================
bool hexToBinary(const uint8_t* hexData, size_t hexSize, uint8_t* binBuffer, size_t* binSize) {
  *binSize = 0;
  uint32_t highAddr = 0;
  const char* hex = (const char*)hexData;
  
  for (size_t i = 0; i < hexSize; i++) {
    if (hex[i] == ':') {
      // Parse hex record
      if (i + 10 > hexSize) break;
      
      uint8_t byteCount = (hexToNibble(hex[i+1]) << 4) | hexToNibble(hex[i+2]);
      uint16_t addr = ((hexToNibble(hex[i+3]) << 12) | (hexToNibble(hex[i+4]) << 8) |
                       (hexToNibble(hex[i+5]) << 4) | hexToNibble(hex[i+6]));
      uint8_t recType = (hexToNibble(hex[i+7]) << 4) | hexToNibble(hex[i+8]);
      
      if (recType == 0x04) {
        // Extended linear address
        highAddr = ((hexToNibble(hex[i+9]) << 12) | (hexToNibble(hex[i+10]) << 8) |
                    (hexToNibble(hex[i+11]) << 4) | hexToNibble(hex[i+12])) << 16;
        i += 10 + byteCount * 2;
      } else if (recType == 0x00) {
        // Data record
        uint32_t fullAddr = highAddr | addr;
        for (int j = 0; j < byteCount; j++) {
          uint8_t byte = (hexToNibble(hex[i+9+j*2]) << 4) | hexToNibble(hex[i+10+j*2]);
          if (fullAddr + j < 256 * 1024) {
            binBuffer[fullAddr + j] = byte;
            if (fullAddr + j >= *binSize) {
              *binSize = fullAddr + j + 1;
            }
          }
        }
        i += 8 + byteCount * 2;
      } else if (recType == 0x01) {
        // EOF
        break;
      } else {
        i += 8 + byteCount * 2;
      }
    }
  }
  return true;
}

uint8_t hexToNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

// ============================================================================
// CRC16-XMODEM Calculation
// ============================================================================
uint16_t crc16_xmodem(const uint8_t* data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ============================================================================
// STM32 Flash Function using Simple Bootloader Protocol
// Protocol: Wait for BOOT -> Send 'S' -> Send chunks ('D' + 8 bytes + checksum) -> Send 'E'
// ============================================================================
bool flashSTM32(const uint8_t* firmware, size_t firmwareSize) {
  flashState = IN_PROGRESS;
  flashProgress = 0;
  flashMessage = "Starting flash process...";
  
  Serial.println("=== Starting STM32 Flash Process ===");
  Serial.printf("Firmware size: %d bytes\n", firmwareSize);
  
  // Pad firmware to multiple of CHUNK_SIZE (8 bytes for simple bootloader)
  size_t paddedSize = ((firmwareSize + CHUNK_SIZE - 1) / CHUNK_SIZE) * CHUNK_SIZE;
  uint8_t* paddedFirmware = (uint8_t*)malloc(paddedSize);
  
  if (!paddedFirmware) {
    flashMessage = "Error: Memory allocation failed";
    flashState = FAILED;
    return false;
  }
  
  memcpy(paddedFirmware, firmware, firmwareSize);
  memset(paddedFirmware + firmwareSize, 0xFF, paddedSize - firmwareSize);
  
  Serial.printf("Padded size: %d bytes\n", paddedSize);
  
  // Clear UART buffers
  while (STM32Serial.available()) {
    STM32Serial.read();
  }
  
  // Wait for "BOOT" message from bootloader
  flashMessage = "Waiting for bootloader...";
  Serial.println("Waiting for bootloader BOOT message...");
  
  unsigned long startTime = millis();
  bool bootFound = false;
  String bootBuffer = "";
  
  while (millis() - startTime < 5000) {
    if (STM32Serial.available()) {
      char c = STM32Serial.read();
      bootBuffer += c;
      if (bootBuffer.indexOf("BOOT") >= 0) {
        bootFound = true;
        break;
      }
    }
    yield();
    delay(10);
  }
  
  if (!bootFound) {
    flashMessage = "Error: Bootloader not detected. Make sure STM32 is in bootloader mode.";
    flashState = FAILED;
    free(paddedFirmware);
    Serial.println("Bootloader not detected!");
    return false;
  }
  
  Serial.println("Bootloader detected!");
  
  // Wait for any remaining heartbeat messages and clear buffer (matching Python: 0.6s wait + double clear)
  delay(600);
  while (STM32Serial.available()) {
    STM32Serial.read();
  }
  delay(100);
  while (STM32Serial.available()) {
    STM32Serial.read();
  }
  
  // Proceed directly to simple bootloader protocol (bootloader sends BOOT and expects 'S')
  flashMessage = "Initiating transfer...";
  Serial.println("Proceeding with simple bootloader protocol (send 'S')...");
  
    // Simple bootloader protocol: 8-byte chunks
    size_t chunkCount = paddedSize / CHUNK_SIZE;
    Serial.printf("Sending %d chunks...\n", chunkCount);
    flashMessage = "Transferring firmware...";

    // Send START command ('S') and wait for ACK
    Serial.println("Sending START ('S')...");
    STM32Serial.write((uint8_t)CMD_START);
    STM32Serial.flush();
    unsigned long startAckTimeout = millis();
    bool startAck = false;
    while (millis() - startAckTimeout < 3000) {
      if (STM32Serial.available()) {
        uint8_t r = STM32Serial.read();
        if (r == ACK) { 
          Serial.println("START acknowledged.");
          startAck = true; 
          break; 
        }
      }
      yield();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (!startAck) {
      flashMessage = "Error: No ACK after START";
      flashState = FAILED;
      free(paddedFirmware);
      Serial.println("Error: START not acknowledged by bootloader");
      return false;
    }

    for (size_t chunkIdx = 0; chunkIdx < chunkCount; chunkIdx++) {
      uint32_t offset = chunkIdx * CHUNK_SIZE;
      const uint8_t* chunkData = paddedFirmware + offset;

      // Calculate simple checksum (8-bit sum)
      uint8_t checksum = 0;
      for (int i = 0; i < CHUNK_SIZE; i++) checksum += chunkData[i];

      bool chunkOk = false;
      for (int retry = 0; retry < MAX_RETRIES; retry++) {
        // Send DATA command + 8 bytes + checksum
        if (retry > 0 || chunkIdx % 128 == 0) {
          Serial.printf("Chunk %d (retry %d)\n", (int)chunkIdx + 1, retry + 1);
        }
        STM32Serial.write((uint8_t)CMD_DATA);
        STM32Serial.write(chunkData, CHUNK_SIZE);
        STM32Serial.write(checksum);
        STM32Serial.flush();

        // Determine timeout: longer when a page just completed (every 128 chunks)
        // Match Python: 3.0s for page writes, 1.0s normal
        unsigned long respTimeout = (chunkIdx > 0 && (chunkIdx % 128) == 0) ? 3000 : 1000;
        unsigned long respStart = millis();
        while (millis() - respStart < respTimeout) {
          if (STM32Serial.available()) {
            uint8_t resp = STM32Serial.read();
            if (resp == ACK) { chunkOk = true; break; }
            if (resp == NAK) { 
              Serial.printf("Chunk %d: NAK (retry %d)\n", (int)chunkIdx + 1, retry + 1); 
              break; 
            }
          }
          yield();
          vTaskDelay(5 / portTICK_PERIOD_MS);
        }

        if (chunkOk) break;
      }

      if (!chunkOk) {
        flashMessage = String("Error: Chunk ") + String(chunkIdx + 1) + " failed";
        flashState = FAILED;
        free(paddedFirmware);
        Serial.printf("Chunk %d failed after retries\n", (int)chunkIdx + 1);
        return false;
      }

      // Update progress (print every 128 chunks like Python)
      flashProgress = ((chunkIdx + 1) * 100) / chunkCount;
      if ((chunkIdx + 1) % 128 == 0 || chunkIdx == chunkCount - 1) {
        Serial.printf("Chunk %5d/%d (%5d bytes): OK\n", (int)chunkIdx + 1, (int)chunkCount, (int)offset);
      }

      yield();
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    Serial.println("All chunks sent successfully!");

    // Send END command ('E') and wait for ACK
    Serial.println("Sending END ('E')...");
    STM32Serial.write((uint8_t)CMD_END);
    STM32Serial.flush();
    unsigned long endStart = millis();
    bool endAck = false;
    while (millis() - endStart < 3000) {
      if (STM32Serial.available()) {
        uint8_t r = STM32Serial.read();
        if (r == ACK) { 
          endAck = true; 
          break; 
        }
      }
      yield();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    free(paddedFirmware);

    if (endAck) {
      flashMessage = "✓ Firmware transfer complete! Bootloader will jump to application.";
      flashState = SUCCESS;
      flashProgress = 100;
      Serial.println("✓ Firmware transfer complete!");
      Serial.println("✓ Bootloader will now jump to application at 0x08002000");
      return true;
    } else {
      flashMessage = "Warning: Expected ACK after END, but flash may be complete";
      flashState = SUCCESS;
      flashProgress = 100;
      Serial.println("Warning: No response after END, but flash may be complete");
      return true;
    }
}

// ============================================================================
// Get Flash Status as JSON
// ============================================================================
String getFlashStatusJSON() {
  String json = "{";
  json += "\"state\":\"";
  switch (flashState) {
    case IDLE: json += "idle"; break;
    case IN_PROGRESS: json += "in_progress"; break;
    case SUCCESS: json += "success"; break;
    case FAILED: json += "failed"; break;
  }
  json += "\",";
  json += "\"progress\":" + String(flashProgress) + ",";
  json += "\"message\":\"" + flashMessage + "\"";
  json += "}";
  return json;
}

// ============================================================================
// Setup Function
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== ESP32 STM32 Flash Loader ===");
  
  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("Error: Failed to mount LittleFS");
  } else {
    Serial.println("✓ LittleFS mounted");
  }
  
  // Initialize STM32 UART
  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  Serial.printf("✓ STM32 UART initialized (RX:%d, TX:%d, Baud:%d)\n", 
                STM32_RX_PIN, STM32_TX_PIN, STM32_BAUD);
  
  // Setup WiFi
  if (WIFI_MODE_AP) {
    Serial.println("Starting Access Point mode...");
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("✓ AP IP Address: ");
    Serial.println(IP);
    Serial.printf("  SSID: %s\n", AP_SSID);
    Serial.printf("  Password: %s\n", AP_PASSWORD);
  } else {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(STA_SSID, STA_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✓ Connected to WiFi");
      Serial.print("  IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\n✗ WiFi connection failed, starting AP mode...");
      WiFi.softAP(AP_SSID, AP_PASSWORD);
      Serial.print("  AP IP Address: ");
      Serial.println(WiFi.softAPIP());
    }
  }
  
  // ========== Web Server Routes ==========
  
  // Serve main page from LittleFS or return 404
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/index.html")) {
      request->send(LittleFS, "/index.html", "text/html");
    } else {
      request->send(404, "text/plain", "index.html not found");
    }
  });
  
  // API: Get flash status
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", getFlashStatusJSON());
  });
  
  // API: Upload firmware file
  server.on("/api/upload", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      // This is called when upload completes
      if (flashState == IN_PROGRESS) {
        request->send(200, "application/json", "{\"success\":true,\"message\":\"Upload complete, flashing in progress...\"}");
      } else if (flashState == SUCCESS) {
        request->send(200, "application/json", "{\"success\":true,\"message\":\"Flash successful!\"}");
      } else {
        request->send(200, "application/json", "{\"success\":false,\"message\":\"" + flashMessage + "\"}");
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      // This is called for each chunk of uploaded data
      static uint8_t* firmwareBuffer = nullptr;
      static uint8_t* hexBuffer = nullptr;
      static size_t totalSize = 0;
      static size_t receivedSize = 0;
      static bool isHexFile = false;
      
      if (index == 0) {
        // First chunk
        Serial.printf("\n=== Upload started: %s ===\n", filename.c_str());
        Serial.printf("Content length: %d bytes\n", request->contentLength());
        
        totalSize = request->contentLength();
        receivedSize = 0;
        isHexFile = filename.endsWith(".hex");
        
        if (isHexFile) {
          // For .hex files, allocate space for hex data
          hexBuffer = (uint8_t*)malloc(totalSize + 1);
          if (!hexBuffer) {
            flashState = FAILED;
            flashMessage = "Error: Out of memory for hex data";
            return;
          }
        } else {
          // For .bin files, allocate directly
          firmwareBuffer = (uint8_t*)malloc(totalSize);
          if (!firmwareBuffer) {
            flashState = FAILED;
            flashMessage = "Error: Out of memory";
            return;
          }
        }
        
        flashState = IDLE;
        flashProgress = 0;
        flashMessage = "Uploading firmware...";
      }
      
      // Copy chunk
      if (isHexFile && hexBuffer && receivedSize + len <= totalSize) {
        memcpy(hexBuffer + receivedSize, data, len);
        receivedSize += len;
      } else if (!isHexFile && firmwareBuffer && receivedSize + len <= totalSize) {
        memcpy(firmwareBuffer + receivedSize, data, len);
        receivedSize += len;
      }
      
      int uploadProgress = (receivedSize * 50) / totalSize;
      if (uploadProgress != flashProgress) {
        flashProgress = uploadProgress;
        Serial.printf("Upload progress: %d%%\n", uploadProgress * 2);
      }
      
      if (final) {
        Serial.printf("Upload complete: %d bytes\n", receivedSize);
        
        if (isHexFile && hexBuffer && receivedSize > 0) {
          // Convert hex to binary
          hexBuffer[receivedSize] = '\0';
          firmwareBuffer = (uint8_t*)malloc(256 * 1024);
          if (!firmwareBuffer) {
            flashState = FAILED;
            flashMessage = "Error: Out of memory for binary";
            free(hexBuffer);
            hexBuffer = nullptr;
            return;
          }
          size_t binSize = 0;
          hexToBinary(hexBuffer, receivedSize, firmwareBuffer, &binSize);
          free(hexBuffer);
          hexBuffer = nullptr;
          Serial.printf("Hex converted: %d bytes\n", binSize);
          // Create flash job and run in background task
          FlashJob* job = (FlashJob*)malloc(sizeof(FlashJob));
          if (!job) {
            flashState = FAILED;
            flashMessage = "Error: Out of memory for job";
            free(firmwareBuffer);
            firmwareBuffer = nullptr;
            return;
          }
          job->data = firmwareBuffer;
          job->size = binSize;
          // Spawn task pinned to core 1 to avoid blocking async_tcp
          if (xTaskCreatePinnedToCore(flashTask, "flashTask", 16 * 1024, job, 5, NULL, 1) != pdPASS) {
            flashState = FAILED;
            flashMessage = "Error: Could not start flash task";
            free(job->data);
            free(job);
            firmwareBuffer = nullptr;
            return;
          }
          // firmwareBuffer ownership transferred to job
          firmwareBuffer = nullptr;
        } else if (!isHexFile && firmwareBuffer && receivedSize > 0) {
          // Create flash job for binary and run in background
          FlashJob* job = (FlashJob*)malloc(sizeof(FlashJob));
          if (!job) {
            flashState = FAILED;
            flashMessage = "Error: Out of memory for job";
            free(firmwareBuffer);
            firmwareBuffer = nullptr;
            return;
          }
          job->data = firmwareBuffer;
          job->size = receivedSize;
          if (xTaskCreatePinnedToCore(flashTask, "flashTask", 16 * 1024, job, 5, NULL, 1) != pdPASS) {
            flashState = FAILED;
            flashMessage = "Error: Could not start flash task";
            free(job->data);
            free(job);
            firmwareBuffer = nullptr;
            return;
          }
          firmwareBuffer = nullptr;
        } else {
          flashState = FAILED;
          flashMessage = "Error: Upload incomplete or no data";
          if (firmwareBuffer) {
            free(firmwareBuffer);
            firmwareBuffer = nullptr;
          }
          if (hexBuffer) {
            free(hexBuffer);
            hexBuffer = nullptr;
          }
        }
      }
    }
  );
  
  // API: Reset state
  server.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    flashState = IDLE;
    flashProgress = 0;
    flashMessage = "";
    request->send(200, "application/json", "{\"success\":true}");
  });
  
  // Start server
  server.begin();
  Serial.println("✓ Web server started");
  Serial.println("\n=== Ready ===");
  Serial.println("Open your browser and navigate to:");
  
  if (WIFI_MODE_AP) {
    Serial.println("  http://192.168.4.1");
  } else {
    Serial.print("  http://");
    Serial.println(WiFi.localIP());
  }
  Serial.println();
}

// ============================================================================
// Loop Function
// ============================================================================
void loop() {
  // Nothing to do here - everything is handled by async callbacks
  delay(100);
}
