#include <HardwareSerial.h>

// PWM
const int pwmPin = 5;
const int pwmFrequency = 5000;
const int pwmResolution = 8;
int motorSpeed = 200;
const int minMotorSpeed = 200;

// LiDAR UART
HardwareSerial LidarSerial(2);
const int lidarRxPin = 16;

// Packet buffer
#define MEAS_PACKET_SIZE 22
uint8_t buf[MEAS_PACKET_SIZE];
int idx = 0;

// LDS02RR produces ~86 packets per revolution
const int PACKETS_PER_REV = 86;

// First index seen after boot
int indexStart = -1;

// Checksum
uint16_t calcChecksum(uint8_t* b) {
  uint32_t checksum = 0;
  for (int i = 0; i < 20; i += 2)
    checksum = (checksum << 1) + (b[i] | (b[i+1] << 8));
  checksum = (checksum + (checksum >> 15)) & 0x7FFF;
  return (uint16_t)checksum;
}

// Setup
void setup() {
  Serial.begin(115200);
  Serial.println("=== LDS02RR FULL DECODER (INDEX ANGLE) ===");

  // Start PWM so the motor spins
  ledcAttach(pwmPin, pwmFrequency, pwmResolution);
  ledcWrite(pwmPin, motorSpeed);

  // Start lidar UART
  LidarSerial.begin(115200, SERIAL_8N1, lidarRxPin, -1);
  Serial.println("Motor + Lidar initialized.");
}

// Main Loop
void loop() {
  // Keep motor spinning
  ledcWrite(pwmPin, motorSpeed);

  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();

    // Sync on 0xFA
    if (idx == 0 && b != 0xFA) continue;
    buf[idx++] = b;

    // Full packet received
    if (idx == MEAS_PACKET_SIZE) {
      uint16_t recv = buf[20] | (buf[21] << 8);
      uint16_t calc = calcChecksum(buf);

      if (recv == calc) {
        uint8_t index = buf[1];

        // Initialize zero-angle reference
        if (indexStart < 0) {
          indexStart = index;
        }

        // Compute delta index with wrap
        int deltaIndex = index - indexStart;
        if (deltaIndex < 0) deltaIndex += 256;

        // Map to 0–PACKETS_PER_REV
        deltaIndex %= PACKETS_PER_REV;

        // Convert to angle
        float angleDeg = (deltaIndex / (float)PACKETS_PER_REV) * 360.0f;

        // Decode 4 samples
        for (int i = 0; i < 4; i++) {
          int base = 4 + i * 4;

          uint16_t dist = buf[base] | (buf[base + 1] << 8);
          uint16_t strength = buf[base + 2] | (buf[base + 3] << 8);

          if (dist == 0 || (dist & 0x8000)) continue;

          // Motor speed logic
          motorSpeed = map(dist, 0, 6000, 0, 255);
          if (motorSpeed < minMotorSpeed) motorSpeed = minMotorSpeed;

          Serial.printf(
            "Angle=%.2f deg, Distance=%u mm, Strength=%u, Index=%u, Sample=%d\n",
            angleDeg, dist, strength, index, i
          );
        }
      }

      idx = 0; // reset for next packet
    }
  }
}
