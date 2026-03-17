#include <Arduino.h>
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"

// ==========================================
// 1. CONFIGURATION & PINS
// ==========================================
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

// ค่า PID (แนะนำให้จูน Kp ลงหากเปลี่ยนเป็นความละเอียด x4 ในอนาคต)
PID PIDMotorL(-255, 255, 200.0, 0.0, 0.0);
PID PIDMotorR(-255, 255, 200.0, 0.0, 0.0);

const float WHEEL_RADIUS = 0.05;
const float TRACK_WIDTH  = 0.37;
const float WHEEL_PPR    = 16.0 * 99.5; 
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

// ==========================================
// 2. COMMUNICATION STRUCTURE (Update for Localization)
// ==========================================
#pragma pack(push, 1)
struct FeedbackPacket {
    uint8_t h1 = 0xAA;
    uint8_t h2 = 0x55;
    float v_measured;      // 4 bytes
    float omega_measured;  // 4 bytes
    int32_t left_ticks;    // 4 bytes (สำหรับ ROS2 Localization)
    int32_t right_ticks;   // 4 bytes (สำหรับ ROS2 Localization)
    uint8_t checksum;      // 1 byte
};

struct CommandPacket {
    float v_target;
    float omega_target;
    uint8_t checksum;
};
#pragma pack(pop)

// ==========================================
// 3. SHARED STATE (Thread-safe exchange)
// ==========================================
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
volatile float target_v = 0.0f, target_w = 0.0f;
volatile float curr_v = 0.0f, curr_w = 0.0f;
volatile int32_t share_left_ticks = 0, share_right_ticks = 0;

// ==========================================
// 4. CONTROL LOOP (Core 1 — 200 Hz)
// ==========================================
void ControlTask(void * pv) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long prevMicros = micros();
    float fL = 0.0f, fR = 0.0f;

    while(1) {
        unsigned long cur = micros();
        float dt = (cur - prevMicros) / 1000000.0f;
        if (dt <= 0) dt = 0.005f;
        prevMicros = cur;

        float tv, tw;
        portENTER_CRITICAL(&gMux);
        tv = target_v; tw = target_w;
        portEXIT_CRITICAL(&gMux);

        // ดึงค่า Absolute Ticks และ Delta
        // หมายเหตุ: Encoder_GetDelta จะรีเซ็ตค่าภายในตัวมันเองเพื่อหาความต่าง
        long dL = Encoder_GetDelta(&encLeft);
        long dR = Encoder_GetDelta(&encRight);
        
        // ดึงค่าสะสมล่าสุดสำหรับ Localization
        int32_t absoluteL = (int32_t)Encoder_GetCount(&encLeft);
        int32_t absoluteR = (int32_t)Encoder_GetCount(&encRight);

        float sL = dL * DIST_PER_TICK;
        float sR = dR * DIST_PER_TICK;
        
        // Low-pass Filter สำหรับความเร็ว
        fL = (0.2f * (sL/dt)) + 0.8f * fL; 
        fR = (0.2f * (sR/dt)) + 0.8f * fR;

        if (abs(tv) < 0.001f && abs(tw) < 0.001f) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
            fL = 0.0f; fR = 0.0f;
        } 
        else {
            float tL = tv - (tw * TRACK_WIDTH / 2.0f);
            float tR = tv + (tw * TRACK_WIDTH / 2.0f);
            BTS7960_SetSpeed(&motor_L, (int)PIDMotorL.compute(tL, fL));
            BTS7960_SetSpeed(&motor_R, (int)PIDMotorR.compute(tR, fR));
        }

        // อัปเดตข้อมูลสถานะทั้งหมดให้ Core 0
        portENTER_CRITICAL(&gMux);
        curr_v = (fR + fL) / 2.0f;
        curr_w = (fR - fL) / TRACK_WIDTH;
        share_left_ticks = absoluteL;
        share_right_ticks = absoluteR;
        portEXIT_CRITICAL(&gMux);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); 
    }
}

// ==========================================
// 5. MAIN SETUP & LOOP (Core 0 — Communication)
// ==========================================
void setup() {
    Serial.begin(115200); 
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18);
    Encoder_Init(&encRight, 13, 14);
    
    xTaskCreatePinnedToCore(ControlTask, "Control", 4096, NULL, 10, NULL, 1);
}

void loop() {
    // 1. รับข้อมูลจาก Python
    if (Serial.available() >= sizeof(CommandPacket) + 2) {
        if (Serial.read() == 0xAA && Serial.peek() == 0x55) {
            Serial.read(); // ข้าม 0x55
            CommandPacket cmd;
            Serial.readBytes((uint8_t*)&cmd, sizeof(CommandPacket));
            
            
            uint8_t sum = 0;
            uint8_t* ptr = (uint8_t*)&cmd;
            for(int i = 0; i < 8; i++) sum += ptr[i]; 

            if (sum == cmd.checksum) {
                portENTER_CRITICAL(&gMux);
                target_v = cmd.v_target;
                target_w = cmd.omega_target;
                portEXIT_CRITICAL(&gMux);
            }
        }
    }

    // 2. ส่งข้อมูลกลับไป Python (Localization Feedback)
    static unsigned long lastTX = 0;
    if (millis() - lastTX >= 10) { // ส่งที่ 50Hz ก็เพียงพอสำหรับ Localization
        lastTX = millis();
        
        FeedbackPacket pkt;
        portENTER_CRITICAL(&gMux);
        pkt.v_measured = curr_v;
        pkt.omega_measured = curr_w;
        pkt.left_ticks = share_left_ticks;
        pkt.right_ticks = share_right_ticks;
        portEXIT_CRITICAL(&gMux);

        // สร้าง Checksum (รวม 16 bytes: v(4), w(4), L(4), R(4))
        uint8_t* ptr = (uint8_t*)&pkt.v_measured;
        pkt.checksum = 0;
        for(int i = 0; i < 16; i++) pkt.checksum += ptr[i];

        Serial.write((uint8_t*)&pkt, sizeof(FeedbackPacket));
    }
}