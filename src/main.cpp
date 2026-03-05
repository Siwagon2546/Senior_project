#include <Arduino.h>
#include <WiFi.h>
#include "AsyncUDP.h"   // ใช้ AsyncUDP เพื่อรับข้อมูลแบบ Event-driven (Zero Latency)
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"
#include "driver/twai.h" 
#include <ESPmDNS.h>

// ==========================================
// 1. CONFIGURATION & PINS
// ==========================================
BTS7960_t motor_L = { 23, 22, 255 }; 
BTS7960_t motor_R = { 26, 25, 255 };
Encoder_t encLeft, encRight;

#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_ID 0x100 

const float WHEEL_PPR = 16.0 * 99.5; 
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH = 0.37;
const float MAX_SPEED_MS = 0.3; 
const float MAX_OMEGA = 1.2;
volatile float totalDistance = 0; // ใช้ volatile เพราะแชร์ข้าม Task

// S-Curve Settings
volatile unsigned long moveStartTime = 0;
volatile float startWheelL = 0, startWheelR = 0;
volatile float targetWheelL = 0, targetWheelR = 0;
const double RAMP_DURATION = 1000.0; 

// PID
PID PIDMotorL(-255, 255, 250.0, 0.0, 0.0); 
PID PIDMotorR(-255, 255, 250.0, 0.0, 0.0);

// Control States
volatile bool autoMode = false; 
volatile float Can_Target_L = 0, Can_Target_R = 0;
volatile bool isAutoDistance = false;
volatile float distanceTarget = 0;
volatile float startDistance = 0;

// Variables สำหรับการแสดงผล (แชร์จาก Control Task มาให้ Loop ปริ้นท์)
volatile float current_meas_v_L = 0;
volatile float current_meas_v_R = 0;
volatile float current_setpoint_L = 0;
volatile float current_setpoint_R = 0;

// ==========================================
// WIFI & UDP CONFIGURATION
// ==========================================
const char* WIFI_SSID = "TRACKING_YOUR_ADDRESS";
const char* WIFI_PASSWORD = "08092003";
const int UDP_PORT = 8888;

AsyncUDP udp;
volatile unsigned long lastUdpTime = 0;
const unsigned long UDP_TIMEOUT_MS = 500; 

#pragma pack(push, 1)
struct ControlPacket {
    float v_req;            
    float w_req;            
    uint8_t btn_dpad;       
    uint8_t btn_cancel;     
    uint8_t btn_toggle_mode;
    uint8_t btn_reset;      
};
#pragma pack(pop)

// ใช้ volatile เพื่อความปลอดภัยเวลารับค่าข้าม Core
volatile ControlPacket currentCommand = {0, 0, 0, 0, 0, 0};
bool lastToggleState = false;

// ==========================================
// 2. HELPER FUNCTIONS
// ==========================================
void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();
    Serial.println("CAN BUS Started");
}

void readCAN() {
    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) { // Non-blocking
        if (message.identifier == CAN_ID && message.data_length_code == 4) {
            int16_t raw_L = (message.data[1] << 8) | message.data[0];
            int16_t raw_R = (message.data[3] << 8) | message.data[2];
            Can_Target_L = (float)raw_L / 100.0f;
            Can_Target_R = (float)raw_R / 100.0f;
        }
    }
}

void sendDistanceCAN() {
    twai_message_t message;
    message.identifier = 0x101; 
    message.extd = 0;           
    message.data_length_code = 4; 

    int32_t dist_cm = (int32_t)(totalDistance * 100.0f);
    message.data[0] = dist_cm & 0xFF;
    message.data[1] = (dist_cm >> 8) & 0xFF;
    message.data[2] = (dist_cm >> 16) & 0xFF;
    message.data[3] = (dist_cm >> 24) & 0xFF;

    twai_transmit(&message, 0); // ไม่ต้องรอคิว ถ้ายุ่งให้ข้ามไปเลย
}

double SCurve(double t, double start, double delta_vel, double duration) {
    if (t >= duration) return start + delta_vel;
    t /= duration / 2.0;
    if (t < 1) return delta_vel / 2.0 * t * t * t + start;
    t -= 2.0;
    return delta_vel / 2.0 * (t * t * t + 2.0) + start;
}

void processWifiInput() {
    float v_req = currentCommand.v_req;
    float w_req = currentCommand.w_req;

    if (v_req > MAX_SPEED_MS) v_req = MAX_SPEED_MS;
    if (v_req < -MAX_SPEED_MS) v_req = -MAX_SPEED_MS;
    if (w_req > MAX_OMEGA) w_req = MAX_OMEGA;
    if (w_req < -MAX_OMEGA) w_req = -MAX_OMEGA;

    float newTargetL = v_req - (w_req * TRACK_WIDTH / 2.0);
    float newTargetR = v_req + (w_req * TRACK_WIDTH / 2.0);

    if (newTargetL != targetWheelL || newTargetR != targetWheelR) {
        startWheelL = targetWheelL; 
        startWheelR = targetWheelR;
        targetWheelL = newTargetL;
        targetWheelR = newTargetR;
        moveStartTime = millis();
    }
}

// ==========================================
// 3. 1000Hz CONTROL TASK (FreeRTOS)
// ==========================================
void ControlLoopTask(void *pvParameters) {
    // กำหนดความถี่ลูป 1ms (1000Hz)
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    unsigned long prevMicros = micros();
    
    // ตัวแปรสำหรับ Low-pass Filter ความเร็ว
    float filtered_v_L = 0;
    float filtered_v_R = 0;
    const float LPF_ALPHA = 0.05; // กรองสัญญาณ (ค่า 0.0 - 1.0 ยิ่งน้อยยิ่งนิ่ง)

    while (true) {
        unsigned long currentMicros = micros();
        double dt = (currentMicros - prevMicros) / 1000000.0;
        if (dt <= 0.0) dt = 0.001; // ป้องกันการหารด้วยศูนย์
        prevMicros = currentMicros;

        unsigned long currentMillis = millis();

        // เช็คระยะทาง (Auto Distance)
        if (isAutoDistance) {
            float traveled = totalDistance - startDistance;
            if (traveled >= distanceTarget) {
                isAutoDistance = false;
                targetWheelL = 0; targetWheelR = 0;
                startWheelL = MAX_SPEED_MS; startWheelR = MAX_SPEED_MS;
                moveStartTime = currentMillis;
            }
        }

        // อัปเดตเป้าหมายจาก CAN Mode
        if (autoMode) {
            if (Can_Target_L != targetWheelL || Can_Target_R != targetWheelR) {
                startWheelL = targetWheelL; startWheelR = targetWheelR;
                targetWheelL = Can_Target_L; targetWheelR = Can_Target_R;
                moveStartTime = currentMillis;
            }
        }

        // คำนวณ S-Curve
        unsigned long elapsed = currentMillis - moveStartTime;
        float setpoint_L = SCurve((double)elapsed, startWheelL, targetWheelL - startWheelL, RAMP_DURATION);
        float setpoint_R = SCurve((double)elapsed, startWheelR, targetWheelR - startWheelR, RAMP_DURATION);
        
        // บันทึกไว้ให้ภายนอกดู
        current_setpoint_L = setpoint_L;
        current_setpoint_R = setpoint_R;

        // คำนวณ Encoder
        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        float dist_per_tick = (2.0 * PI * WHEEL_RADIUS) / WHEEL_PPR;
        float stepL = delta_L * dist_per_tick;
        float stepR = delta_R * dist_per_tick;
        totalDistance += (stepL + stepR) / 2.0;

        // คำนวณความเร็ว Raw
        float raw_v_L = stepL / dt;
        float raw_v_R = stepR / dt;

        // ประยุกต์ใช้ Low-Pass Filter เพื่อลด Quantization Noise ที่ 1000Hz
        filtered_v_L = (LPF_ALPHA * raw_v_L) + ((1.0f - LPF_ALPHA) * filtered_v_L);
        filtered_v_R = (LPF_ALPHA * raw_v_R) + ((1.0f - LPF_ALPHA) * filtered_v_R);
        
        current_meas_v_L = filtered_v_L;
        current_meas_v_R = filtered_v_R;

        // คำนวณ PID
        int final_L = (int)PIDMotorL.compute(setpoint_L, filtered_v_L);
        int final_R = (int)PIDMotorR.compute(setpoint_R, filtered_v_R);

        // สั่งงาน Motor
        if (abs(setpoint_L) < 0.001 && abs(setpoint_R) < 0.001 && abs(filtered_v_L) < 0.01) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, final_L);
            BTS7960_SetSpeed(&motor_R, final_R);
        }

        // หน่วงเวลาให้ลูปนี้รันเป๊ะๆ ทุก 1ms (1000Hz)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==========================================
// 4. MAIN SETUP & LOOP
// ==========================================
void setup() {
    Serial.begin(115200);

    // Setup WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");

    if (!MDNS.begin("myrobot")) {  
        Serial.println("Error setting up MDNS!");
    } else {
        Serial.println("mDNS started: myrobot.local");
    }

    // --- ส่วนที่เปลี่ยนใหม่ (AsyncUDP Setup) ---
    if (udp.listen(UDP_PORT)) {
        Serial.printf("Listening on UDP port %d\n", UDP_PORT);
        
        // ฟังก์ชันนี้จะถูกเรียกอัตโนมัติ (บน Core 0) เมื่อมีข้อมูลเข้ามา
        udp.onPacket([](AsyncUDPPacket packet) {
            if (packet.length() == sizeof(ControlPacket)) {
                // ก๊อปปี้ข้อมูลลง Struct ทันที
                ControlPacket tempCmd;
                memcpy(&tempCmd, packet.data(), sizeof(ControlPacket));
                
                currentCommand.v_req = tempCmd.v_req;
                currentCommand.w_req = tempCmd.w_req;
                currentCommand.btn_dpad = tempCmd.btn_dpad;
                currentCommand.btn_cancel = tempCmd.btn_cancel;
                currentCommand.btn_toggle_mode = tempCmd.btn_toggle_mode;
                currentCommand.btn_reset = tempCmd.btn_reset;
                
                lastUdpTime = millis();
            }
        });
    }
    
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18); 
    Encoder_Init(&encRight, 13, 14);
    
    setupCAN();

    // เริ่มต้น Control Task บน Core 1 ด้วย Priority 10 (สูงกว่า loop ปกติ)
    xTaskCreatePinnedToCore(
        ControlLoopTask,   // ชื่อฟังก์ชัน Task
        "ControlTask",     // ชื่อ Task (สำหรับ Debug)
        8192,              // Stack Size
        NULL,              // Parameters
        10,                // Priority (ยิ่งมากยิ่งสำคัญ)
        NULL,              // Task Handle
        1                  // รันบน Core 1
    );
}

void loop() {
    // --- จัดการ I/O ทั้งหมดในลูปนี้ เพื่อไม่ให้กวน PID ---
    readCAN(); 

    // Safety Timeout
    if (millis() - lastUdpTime > UDP_TIMEOUT_MS) {
        currentCommand.v_req = 0; currentCommand.w_req = 0;
        currentCommand.btn_dpad = 0; currentCommand.btn_cancel = 0;
        currentCommand.btn_toggle_mode = 0; currentCommand.btn_reset = 0;
    }

    // --- BUTTON LOGIC ---
    if (currentCommand.btn_dpad != 0) {
        autoMode = false;      
        isAutoDistance = true; 
        startDistance = totalDistance; 
        
        if (currentCommand.btn_dpad == 1) distanceTarget = 5.0;
        else if (currentCommand.btn_dpad == 2) distanceTarget = 10.0;
        else if (currentCommand.btn_dpad == 3) distanceTarget = 15.0;
        else if (currentCommand.btn_dpad == 4) distanceTarget = 20.0;

        startWheelL = targetWheelL; startWheelR = targetWheelR;
        targetWheelL = MAX_SPEED_MS; targetWheelR = MAX_SPEED_MS;
        moveStartTime = millis();
        
        currentCommand.btn_dpad = 0; 
    }

    if (abs(currentCommand.v_req) > 0.05 || abs(currentCommand.w_req) > 0.05 || currentCommand.btn_cancel) {
        if (isAutoDistance) {
            isAutoDistance = false;
        }
    }

    bool currentToggle = currentCommand.btn_toggle_mode > 0;
    if (currentToggle && !lastToggleState) {
        autoMode = !autoMode;
        isAutoDistance = false; 
        startWheelL = targetWheelL; startWheelR = targetWheelR;
        moveStartTime = millis();
    }
    lastToggleState = currentToggle;

    if (currentCommand.btn_reset) {
        totalDistance = 0;
        isAutoDistance = false;
        currentCommand.btn_reset = 0; 
    }

    if (!isAutoDistance && !autoMode) {
        processWifiInput(); 
    }

    // --- ส่งข้อมูลออก (จำกัดความถี่ เพื่อไม่ให้บล็อกระบบ) ---
    unsigned long currentMillis = millis();
    static unsigned long lastCanTxTime = 0;
    static unsigned long lastSerialPrintTime = 0;

    // ส่ง CAN ที่ 20Hz (ทุกๆ 50ms)
    if (currentMillis - lastCanTxTime >= 50) {
        lastCanTxTime = currentMillis;
        sendDistanceCAN();
    }

    // ปริ้นท์ Serial ที่ 10Hz (ทุกๆ 100ms)
    if (currentMillis - lastSerialPrintTime >= 100) {
        lastSerialPrintTime = currentMillis;
        const char* modeStr = isAutoDistance ? "DIST" : (autoMode ? "AUTO" : "MAN");
        if (Serial.availableForWrite() > 0) {
        Serial.printf("[%s] TgtL:%.2f TgtR:%.2f RealL:%.2f RealR:%.2f Dist:%.2f\n", 
                      modeStr, current_setpoint_L, current_setpoint_R, 
                      current_meas_v_L, current_meas_v_R, totalDistance);
        }
    }
}