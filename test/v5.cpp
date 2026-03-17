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

// ค่า PID สำหรับควบคุมล้อ
PID PIDMotorL(-255, 255, 200.0, 0.0, 0.0);
PID PIDMotorR(-255, 255, 200.0, 0.0, 0.0);

const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH  = 0.37;
const float WHEEL_PPR    = 16.0 * 99.5;
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

// ==========================================
// 2. COMMUNICATION STRUCTURE (Binary Payload)
// ==========================================
#pragma pack(push, 1)
struct FeedbackPacket {
    uint8_t h1 = 0xAA;
    uint8_t h2 = 0x55;
    float v_measured;
    float omega_measured;
    float total_dist;
    uint8_t checksum;
};

struct CommandPacket {
    float v_target;
    float omega_target;
    uint8_t checksum;
};
#pragma pack(pop)

// ==========================================
// 3. SHARED STATE (Core 0 ↔ Core 1)
// ==========================================
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
volatile float target_v = 0.0f, target_w = 0.0f;
volatile float curr_v = 0.0f, curr_w = 0.0f, total_d = 0.0f;

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
        if (dt <= 0) dt = 0.005f; // ค่าเริ่มต้นสำหรับ 200Hz
        prevMicros = cur;

        // รับค่า Target จาก Core 0
        float tv, tw;
        portENTER_CRITICAL(&gMux);
        tv = target_v; tw = target_w;
        portEXIT_CRITICAL(&gMux);

        // ดึงค่าการหมุนจาก Encoder
        float sL = Encoder_GetDelta(&encLeft) * DIST_PER_TICK;
        float sR = Encoder_GetDelta(&encRight) * DIST_PER_TICK;
        
        // กรองสัญญาณความเร็ว (Low-pass Filter) ให้ไวขึ้นรับกับ 200Hz
        fL = (0.2f * (sL/dt)) + 0.8f * fL; 
        fR = (0.2f * (sR/dt)) + 0.8f * fR;

        // ตรวจสอบ Deadband: ถ้าเป้าหมายเป็น 0 ให้หยุดจ่ายไฟและรีเซ็ต PID
        if (abs(tv) < 0.001f && abs(tw) < 0.001f) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
            
            // สำคัญ: ล้างค่าสะสมใน PID เพื่อป้องกันการกระชากเมื่อเริ่มวิ่งใหม่
            // **หมายเหตุ: ต้องมั่นใจว่าในไฟล์ pid.h ของคุณมีฟังก์ชัน reset()**
            fL = 0.0f; 
            fR = 0.0f;
        } 
        else {
            // Inverse Kinematics (Target v,w -> Target vL, vR)
            float tL = tv - (tw * TRACK_WIDTH / 2.0f);
            float tR = tv + (tw * TRACK_WIDTH / 2.0f);

            // Motor Drive ผ่าน PID
            BTS7960_SetSpeed(&motor_L, (int)PIDMotorL.compute(tL, fL));
            BTS7960_SetSpeed(&motor_R, (int)PIDMotorR.compute(tR, fR));
        }

        // อัปเดตข้อมูลสถานะ (Feedback) ให้ Core 0 นำไปส่ง
        portENTER_CRITICAL(&gMux);
        curr_v = (fR + fL) / 2.0f;
        curr_w = (fR - fL) / TRACK_WIDTH;
        total_d += (sL + sR) / 2.0f;
        portEXIT_CRITICAL(&gMux);

        // รอจนครบ 5ms (200Hz)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5)); 
    }
}

// ==========================================
// 5. MAIN SETUP & LOOP (Core 0 — Communication)
// ==========================================
void setup() {
    // ใช้ Baud rate 115200 (สามารถปรับเป็น 921600 ได้ถ้าต้องการลด Latency เพิ่มอีก)
    Serial.begin(115200); 
    
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18);
    Encoder_Init(&encRight, 13, 14);
    
    xTaskCreatePinnedToCore(ControlTask, "Control", 4096, NULL, 10, NULL, 1);
}

void loop() {
    // 1. รับข้อมูลจาก Python (Command)
    if (Serial.available() >= sizeof(CommandPacket) + 2) {
        if (Serial.read() == 0xAA && Serial.peek() == 0x55) {
            Serial.read(); // ข้าม 0x55
            
            CommandPacket cmd;
            Serial.readBytes((uint8_t*)&cmd, sizeof(CommandPacket));
            
            // Checksum Simple Validation
            uint8_t* ptr = (uint8_t*)&cmd;
            uint8_t sum = 0;
            for(int i = 0; i < 8; i++) sum += ptr[i]; 

            if (sum == cmd.checksum) {
                portENTER_CRITICAL(&gMux);
                target_v = cmd.v_target;
                target_w = cmd.omega_target;
                portEXIT_CRITICAL(&gMux);
            }
        }
    }

    // 2. ส่งข้อมูลกลับไป Python (Feedback) ทุกๆ 5ms (200Hz)
    static unsigned long lastTX = 0;
    unsigned long now = millis();
    
    if (now - lastTX >= 5) { // ใช้ >= เพื่อให้เวลาลงล็อคเป๊ะขึ้น
        lastTX = now;
        
        FeedbackPacket pkt;
        portENTER_CRITICAL(&gMux);
        pkt.v_measured = curr_v;
        pkt.omega_measured = curr_w;
        pkt.total_dist = total_d;
        portEXIT_CRITICAL(&gMux);

        // สร้าง Checksum ขาออก
        uint8_t* ptr = (uint8_t*)&pkt.v_measured;
        pkt.checksum = 0;
        for(int i = 0; i < 12; i++) pkt.checksum += ptr[i];

        // ส่งออก UART
        Serial.write((uint8_t*)&pkt, sizeof(FeedbackPacket));
    }
}