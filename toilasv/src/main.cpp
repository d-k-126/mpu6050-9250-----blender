#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "Kalman.h"
#include <WiFi.h>
#include <WiFiUdp.h>


const char* ap_ssid = "ESP32";
const char* ap_password = "12345678";

// Thông tin UDP
const char* udpAddress = "192.168.4.2";
const int udpPortBlender = 12345; 
const int udpPortGraph = 12346;   

WiFiUDP udpBlender; 
WiFiUDP udpGraph;   

// MPU6050 objects
MPU6050 mpu68(0x68);
MPU6050 mpu69(0x69);

// Kalman filters
Kalman kalman68(4);
Kalman kalman69(4);

// MPU control status variables
uint8_t error_code = 0U;
bool dmp_ready = false;
uint16_t packet_size;

// Rotation/motion variables
Quaternion q68_raw, q69_raw;
Quaternion q68_filtered, q69_filtered;

// Average filter variables
const int WINDOW_SIZE = 5;
float q68w_window[WINDOW_SIZE] = {0};
float q68x_window[WINDOW_SIZE] = {0};
float q68y_window[WINDOW_SIZE] = {0};
float q68z_window[WINDOW_SIZE] = {0};
float q69w_window[WINDOW_SIZE] = {0};
float q69x_window[WINDOW_SIZE] = {0};
float q69y_window[WINDOW_SIZE] = {0};
float q69z_window[WINDOW_SIZE] = {0};
int window_index = 0;

// Timing variables
unsigned long last_update_time = 0;
const unsigned long SAMPLE_INTERVAL_MS = 10;

// Mix data
const float ALPHA = 0.98;

// Gait analysis variables
float knee_angle = 0.0;
float step_time = 0.0;
float walking_speed = 0.0;
float last_heel_strike = 0.0;
float accel_z_prev = 0.0;

// Function declarations
void initializeMPU(MPU6050 &mpu, const char* address);
void initializeKalmanFilters();
void applyKalmanFilter(Quaternion &q_raw, Quaternion &q_filtered, Kalman &kalman);
void applyMovingAverage(Quaternion &q, float* w_window, float* x_window, float* y_window, float* z_window);
void normalizeQuaternion(Quaternion &q);
void calculateGaitParameters(int16_t az68);
void sendUDP(WiFiUDP &udp, int port, const char* message);

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.begin(115200);
  while (!Serial);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  Serial.println("Đang chờ kết nối...");
  while (WiFi.softAPgetStationNum() == 0) {
    delay(500);
  }
  Serial.println("Kết nối thành công");

  udpBlender.begin(udpPortBlender);
  udpGraph.begin(udpPortGraph);

  initializeMPU(mpu68, "0x68");
  initializeMPU(mpu69, "0x69");
  initializeKalmanFilters();
  
  if (!mpu68.testConnection() || !mpu69.testConnection()) {
    ESP.restart();
  }
  
  packet_size = mpu68.dmpGetFIFOPacketSize();
  dmp_ready = true;
  
  Serial.println("Đã khởi tạo thành công");
  
  mpu68.resetFIFO();
  mpu69.resetFIFO();
  
  last_update_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_update_time < SAMPLE_INTERVAL_MS) {
    return;
  }
  
  if (!dmp_ready) {
    return;
  }
  
  uint16_t fifo_count68 = mpu68.getFIFOCount();
  uint16_t fifo_count69 = mpu69.getFIFOCount();
  
  if (fifo_count68 >= 1024 || fifo_count69 >= 1024) {
    mpu68.resetFIFO();
    mpu69.resetFIFO();
    last_update_time = current_time;
    return;
  }
  
  if (fifo_count68 < packet_size || fifo_count69 < packet_size) {
    return;
  }
  
  uint8_t fifo_buffer68[64];
  uint8_t fifo_buffer69[64];
  
  if (mpu68.dmpGetCurrentFIFOPacket(fifo_buffer68) && 
      mpu69.dmpGetCurrentFIFOPacket(fifo_buffer69)) {
    
    mpu68.dmpGetQuaternion(&q68_raw, fifo_buffer68);
    mpu69.dmpGetQuaternion(&q69_raw, fifo_buffer69);
    
    applyKalmanFilter(q68_raw, q68_filtered, kalman68);
    applyKalmanFilter(q69_raw, q69_filtered, kalman69);
    
    applyMovingAverage(q68_filtered, q68w_window, q68x_window, q68y_window, q68z_window);
    applyMovingAverage(q69_filtered, q69w_window, q69x_window, q69y_window, q69z_window);
    
    normalizeQuaternion(q68_filtered);
    normalizeQuaternion(q69_filtered);
    
    
    int16_t ax68, ay68, az68, gx68, gy68, gz68;
    mpu68.getMotion6(&ax68, &ay68, &az68, &gx68, &gy68, &gz68);
    
    
    calculateGaitParameters(az68);

    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "{\"data\": \"/quaternion/0\", \"value\": [%.6f, %.6f, %.6f, %.6f]}\n",
             q68_filtered.w, q68_filtered.x, q68_filtered.y, q68_filtered.z);
    sendUDP(udpBlender, udpPortBlender, buffer);
    
    snprintf(buffer, sizeof(buffer), "{\"data\": \"/quaternion/1\", \"value\": [%.6f, %.6f, %.6f, %.6f]}\n",
             q69_filtered.w, q69_filtered.x, q69_filtered.y, q69_filtered.z);
    sendUDP(udpBlender, udpPortBlender, buffer);

    
    snprintf(buffer, sizeof(buffer), 
             "{\"data\": \"/gait\", \"value\": {\"knee_angle\": %.2f, \"step_time\": %.2f, \"walking_speed\": %.2f}}\n",
             knee_angle, step_time, walking_speed);
    sendUDP(udpGraph, udpPortGraph, buffer);
    
    static unsigned long last_print_time = 0;
    if (current_time - last_print_time >= 1000) {
      Serial.println("Đang gửi thông tin");
      last_print_time = current_time;
    }
    
    window_index = (window_index + 1) % WINDOW_SIZE;
    last_update_time = current_time;
  }
}

void sendUDP(WiFiUDP &udp, int port, const char* message) {
  udp.beginPacket(udpAddress, port);
  udp.write((const uint8_t*)message, strlen(message));
  udp.endPacket();
}

void initializeMPU(MPU6050 &mpu, const char* address) {
  mpu.initialize();
  error_code = mpu.dmpInitialize();
  
  if (error_code != 0) {
    Wire.setClock(100000);
    mpu.initialize();
    error_code = mpu.dmpInitialize();
  }
  
  if (error_code != 0) {
    ESP.restart();
  }
  
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  
  mpu.CalibrateAccel(10);
  mpu.CalibrateGyro(10);
  mpu.setDMPEnabled(true);
}

void initializeKalmanFilters() {
  float processNoise = 0.001;
  float measurementNoise = 0.03;
  kalman68.setParameters(processNoise, measurementNoise);
  kalman69.setParameters(processNoise, measurementNoise);
}

void applyKalmanFilter(Quaternion &q_raw, Quaternion &q_filtered, Kalman &kalman) {
  float measurement[4] = {q_raw.w, q_raw.x, q_raw.y, q_raw.z};
  float filtered[4];
  kalman.updateEstimate(measurement, filtered);
  q_filtered.w = filtered[0];
  q_filtered.x = filtered[1];
  q_filtered.y = filtered[2];
  q_filtered.z = filtered[3];
}

void applyMovingAverage(Quaternion &q, float* w_window, float* x_window, float* y_window, float* z_window) {
  w_window[window_index] = q.w;
  x_window[window_index] = q.x;
  y_window[window_index] = q.y;
  z_window[window_index] = q.z;
  
  float w_sum = 0, x_sum = 0, y_sum = 0, z_sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    w_sum += w_window[i];
    x_sum += x_window[i];
    y_sum += y_window[i];
    z_sum += z_window[i];
  }
  
  q.w = ALPHA * q.w + (1 - ALPHA) * (w_sum / WINDOW_SIZE);
  q.x = ALPHA * q.x + (1 - ALPHA) * (x_sum / WINDOW_SIZE);
  q.y = ALPHA * q.y + (1 - ALPHA) * (y_sum / WINDOW_SIZE);
  q.z = ALPHA * q.z + (1 - ALPHA) * (z_sum / WINDOW_SIZE);
}

void normalizeQuaternion(Quaternion &q) {
  float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm > 0.0001f) {
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
  }
}

void calculateGaitParameters(int16_t az68) {
  float accel_z = az68 * (2.0 * 9.81 / 32768.0);
  
  float dot = q68_filtered.w * q69_filtered.w + q68_filtered.x * q69_filtered.x +
              q68_filtered.y * q69_filtered.y + q68_filtered.z * q69_filtered.z;
  knee_angle = acos(constrain(dot, -1.0, 1.0)) * 180.0 / PI;

  if (accel_z_prev > -2.0 && accel_z < -2.0) {
    if (last_heel_strike > 0) {
      step_time = (millis() - last_heel_strike) / 1000.0;
      walking_speed = 0.7 / step_time; 
    }
    last_heel_strike = millis();
  }
  accel_z_prev = accel_z;
}
