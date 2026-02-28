#include <Arduino.h>
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"

// === Libraries ===
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// === Network Settings ===
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* hostIP = "10.42.0.1"; // IP ของ Ubuntu Hotspot
const int udpPortTelemetry = 8889; // พอร์ตสำหรับส่งข้อมูลขึ้น ROS 2
const int udpPortCommand = 8888;   // พอร์ตสำหรับรับคำสั่งจาก ROS 2

WiFiUDP udpTelemetry;
WiFiUDP udpCommand;

// === โครงสร้างข้อมูล (Structs) ===
struct __attribute__((packed)) RobotDataPacket {
  float qI, qJ, qK, qReal; 
  float gX, gY, gZ;        
  float aX, aY, aZ;        
  float vel_L, vel_R;      
  float distance;          
};
RobotDataPacket robotData;

struct __attribute__((packed)) CommandPacket {
  float target_L;
  float target_R;
};
CommandPacket cmdPacket;

Adafruit_BNO08x bno08x;
TaskHandle_t TaskIMU_UDP;

// ==========================================
// 1. CONFIGURATION & PINS
// ==========================================
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

const float WHEEL_PPR = 16.0 * 99.5 ; 
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
const float MAX_SPEED_MS = 0.3; 
volatile float totalDistance = 0; 

unsigned long moveStartTime = 0;
float startWheelL = 0, startWheelR = 0;
float targetWheelL = 0, targetWheelR = 0;
const double RAMP_DURATION = 1000.0; 

PID PIDMotorL(-255, 255, 1000.0, 0.0, 0.0); 
PID PIDMotorR(-255, 255, 1000.0, 0.0, 0.0);

// === ตัวแปร Global สำหรับแชร์ระหว่าง Core ===
volatile float global_meas_v_L = 0;
volatile float global_meas_v_R = 0;
volatile float udp_target_L = 0;
volatile float udp_target_R = 0;

unsigned long lastCmdTime = 0;           
const unsigned long CMD_TIMEOUT_MS = 500; 

unsigned long prevPidTime = 0;
const long pidInterval = 3;

// ==========================================
// 2. HELPER FUNCTIONS
// ==========================================
double SCurve(double t, double start, double delta_vel, double duration) {
    if (t >= duration) return start + delta_vel;
    t /= duration / 2.0;
    if (t < 1) return delta_vel / 2.0 * t * t * t + start;
    t -= 2.0;
    return delta_vel / 2.0 * (t * t * t + 2.0) + start;
}

// ==========================================
// 3. TASK บน Core 0 (จัดการ Wi-Fi, IMU, และ UDP ขาเข้า/ขาออก)
// ==========================================
void imuUdpTask(void * pvParameters) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    // ผูกพอร์ตสำหรับรับคำสั่งจาก ROS 2
    udpCommand.begin(udpPortCommand);

    Wire.begin();
    Wire.setClock(400000);
    if (!bno08x.begin_I2C(0x4A)) { 
        while (1) { vTaskDelay(10 / portTICK_PERIOD_MS); } 
    }
  
    uint32_t reportIntervalUs = 9000; 
    bno08x.enableReport(SH2_ROTATION_VECTOR, reportIntervalUs);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, reportIntervalUs);
    bno08x.enableReport(SH2_ACCELEROMETER, reportIntervalUs);

    sh2_SensorValue_t sensorValue;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 5 / portTICK_PERIOD_MS; // 100Hz

    for(;;) {
        // --- 1. รับคำสั่ง Setpoint จาก ROS 2 (Non-blocking) ---
        int packetSize = udpCommand.parsePacket();
        if (packetSize == sizeof(CommandPacket)) {
            udpCommand.read((char*)&cmdPacket, sizeof(CommandPacket));
            udp_target_L = cmdPacket.target_L;
            udp_target_R = cmdPacket.target_R;
            lastCmdTime = millis(); // อัปเดต Watchdog
        }

        // --- 2. อ่านค่า IMU ---
        while (bno08x.getSensorEvent(&sensorValue)) {
            switch (sensorValue.sensorId) {
                case SH2_ROTATION_VECTOR:
                    robotData.qI = sensorValue.un.rotationVector.i;
                    robotData.qJ = sensorValue.un.rotationVector.j;
                    robotData.qK = sensorValue.un.rotationVector.k;
                    robotData.qReal = sensorValue.un.rotationVector.real;
                    break;
                case SH2_GYROSCOPE_CALIBRATED:
                    robotData.gX = sensorValue.un.gyroscope.x;
                    robotData.gY = sensorValue.un.gyroscope.y;
                    robotData.gZ = sensorValue.un.gyroscope.z;
                    break;
                case SH2_ACCELEROMETER:
                    robotData.aX = sensorValue.un.accelerometer.x;
                    robotData.aY = sensorValue.un.accelerometer.y;
                    robotData.aZ = sensorValue.un.accelerometer.z;
                    break;
            }
        }

        // --- 3. แพ็กข้อมูล Encoder ล่าสุดส่งขึ้น ROS 2 ---
        robotData.vel_L = global_meas_v_L;
        robotData.vel_R = global_meas_v_R;
        robotData.distance = totalDistance;

        udpTelemetry.beginPacket(hostIP, udpPortTelemetry);
        udpTelemetry.write((uint8_t*)&robotData, sizeof(RobotDataPacket));
        udpTelemetry.endPacket();

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// 4. MAIN LOOP บน Core 1 (ควบคุม Motor)
// ==========================================
void setup() {
    Serial.begin(115200);

    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 18, 19); 
    Encoder_Init(&encRight, 13, 14);

    xTaskCreatePinnedToCore(
        imuUdpTask, "IMU_UDP", 8192, NULL, 1, &TaskIMU_UDP, 0 
    );
}

void loop() {
    unsigned long currentMillis = millis();

    // 1. Watchdog: ถ้าการเชื่อมต่อ ROS 2 ขาดหายเกิน 0.5 วินาที ให้รถหยุด
    if (currentMillis - lastCmdTime > CMD_TIMEOUT_MS) {
        udp_target_L = 0;
        udp_target_R = 0;
    }

    // 2. ลูปควบคุม PID ที่ 50Hz (20ms)
    if (currentMillis - prevPidTime >= pidInterval) {
        double dt = (currentMillis - prevPidTime) / 1000.0;
        prevPidTime = currentMillis;

        // อัปเดต Target 
        if (udp_target_L != targetWheelL || udp_target_R != targetWheelR) {
            startWheelL = targetWheelL; 
            startWheelR = targetWheelR;
            targetWheelL = udp_target_L; 
            targetWheelR = udp_target_R;
            moveStartTime = currentMillis;
        }

        unsigned long elapsed = currentMillis - moveStartTime;
        float setpoint_L = SCurve((double)elapsed, startWheelL, targetWheelL - startWheelL, RAMP_DURATION);
        float setpoint_R = SCurve((double)elapsed, startWheelR, targetWheelR - startWheelR, RAMP_DURATION);

        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
        float stepL = delta_L * dist_per_tick;
        float stepR = delta_R * dist_per_tick;
        totalDistance += (stepL + stepR) / 2.0;

        float meas_v_L = stepL / dt;
        float meas_v_R = stepR / dt;

        // อัปเดตตัวแปร Global ให้ Core 0 นำไปส่ง
        global_meas_v_L = meas_v_L;
        global_meas_v_R = meas_v_R;

        int final_L = (int)PIDMotorL.compute(setpoint_L, meas_v_L);
        int final_R = (int)PIDMotorR.compute(setpoint_R, meas_v_R);

        if (abs(setpoint_L) < 0.001 && abs(setpoint_R) < 0.001 && abs(meas_v_L) < 0.01) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, final_L);
            BTS7960_SetSpeed(&motor_R, final_R);
        }
    }
}