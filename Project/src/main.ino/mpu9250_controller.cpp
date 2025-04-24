#include "mpu9250_controller.hpp"

MPU9250Controller::MPU9250Controller()
  : _usb(nullptr), _bt(nullptr),
    _lastMs(0), _ledState(false)
{}

bool MPU9250Controller::begin(HardwareSerial* usb, BluetoothSerial* bt) {
  _usb = usb;
  _bt  = bt;

  // Initialize I2C on custom pins :contentReference[oaicite:9]{index=9}
  Wire.begin(I2C_SDA, I2C_SCL);

  // Setup sensor; returns false if not found :contentReference[oaicite:10]{index=10}
  if (!mpu.setup(0x68)) {
    _usb->println("MPU9250 setup failed");
    return false;
  }

  // Calibrate accel/gyro & magnetometer :contentReference[oaicite:11]{index=11}
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();

  // Use Madgwick AHRS; 10 iterations for stable yaw :contentReference[oaicite:12]{index=12}
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(10);

  pinMode(LED_PIN, OUTPUT);
  _lastMs = millis();
  return true;
}

void MPU9250Controller::update() {
  unsigned long now = millis();
  if (now - _lastMs < UPDATE_INTERVAL_MS) return;
  _lastMs = now;

  // Read + fuse; returns true if new data :contentReference[oaicite:13]{index=13}
  if (!mpu.update()) return;

  // Extract fused angles :contentReference[oaicite:14]{index=14}
  float roll  = mpu.getRoll();
  float pitch = mpu.getPitch();
  float yaw   = mpu.getYaw();

  // Stream JSON
  streamJSON(roll, pitch, yaw);

  // LED heartbeat
  _ledState = !_ledState;
  digitalWrite(LED_PIN, _ledState);
}

void MPU9250Controller::streamJSON(float roll, float pitch, float yaw) {
  StaticJsonDocument<256> doc;  // fits <1 KB :contentReference[oaicite:15]{index=15}

  // Fused orientation
  doc["roll"]  = roll;
  doc["pitch"] = pitch;
  doc["yaw"]   = yaw;

  // Raw sensor data :contentReference[oaicite:16]{index=16}
  doc["accel_x"] = mpu.getAccX();
  doc["accel_y"] = mpu.getAccY();
  doc["accel_z"] = mpu.getAccZ();
  doc["gyro_x"]  = mpu.getGyroX();
  doc["gyro_y"]  = mpu.getGyroY();
  doc["gyro_z"]  = mpu.getGyroZ();
  doc["mag_x"]   = mpu.getMagX();
  doc["mag_y"]   = mpu.getMagY();
  doc["mag_z"]   = mpu.getMagZ();

  // Transmit JSON over Bluetooth & USB :contentReference[oaicite:17]{index=17}
  serializeJson(doc, *_bt);
  _bt->println();
  serializeJson(doc, *_usb);
  _usb->println();
}
