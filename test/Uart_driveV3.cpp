#include <Arduino.h>
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"

// --- Configuration ---
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

PID PIDMotorL(-255, 255, 100.0, 0.0, 0.0);
PID PIDMotorR(-255, 255, 100.0, 0.0, 0.0);

const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH  = 0.37;
const float WHEEL_PPR    = 16.0 * 99.5 * 4;
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

// --- Communication Structure ---
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

// --- Shared Variables ---
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
volatile float target_v = 0, target_w = 0;
volatile float curr_v = 0, curr_w = 0, total_d = 0;

// --- Task: Control Loop (Core 1) ---
void ControlTask(void * pv) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long prevMicros = micros();
    float fL = 0, fR = 0;

    while(1) {
        unsigned long cur = micros();
        float dt = (cur - prevMicros) / 1000000.0f;
        if (dt <= 0) dt = 0.005; // สำหรับ 200Hz
        prevMicros = cur;

        float tv, tw;
        portENTER_CRITICAL(&gMux);
        tv = target_v; tw = target_w;
        portEXIT_CRITICAL(&gMux);

        // 1. ตรวจสอบ Deadband (แก้ปัญหาเสียงวี๊ด)
        // ถ้า Target เป็น 0 ให้หยุดจ่ายไฟและล้างค่า PID ทันที
        if (abs(tv) < 0.005f && abs(tw) < 0.005f) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
            fL = 0; fR = 0; // ล้างค่า Filter ด้วย
        } 
        else {
            // Inverse Kinematics
            float tL = tv - (tw * TRACK_WIDTH / 2.0f);
            float tR = tv + (tw * TRACK_WIDTH / 2.0f);

            // Measured
            float sL = Encoder_GetDelta(&encLeft) * DIST_PER_TICK;
            float sR = Encoder_GetDelta(&encRight) * DIST_PER_TICK;
            fL = (0.2f * (sL/dt)) + 0.8f * fL; // ปรับ Filter ให้ไวขึ้นตามความถี่
            fR = (0.2f * (sR/dt)) + 0.8f * fR;

            BTS7960_SetSpeed(&motor_L, (int)PIDMotorL.compute(tL, fL));
            BTS7960_SetSpeed(&motor_R, (int)PIDMotorR.compute(tR, fR));
        }

        // Update Global State สำหรับ Feedback
        portENTER_CRITICAL(&gMux);
        curr_v = (fR + fL) / 2.0f;
        curr_w = (fR - fL) / TRACK_WIDTH;
        total_d += ( (Encoder_GetDelta(&encLeft) + Encoder_GetDelta(&encRight))/2.0f ) * DIST_PER_TICK;
        portEXIT_CRITICAL(&gMux);

        // 200 Hz = 5ms
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18,PCNT_UNIT_0);
    Encoder_Init(&encRight, 13, 14,PCNT_UNIT_1);
    xTaskCreatePinnedToCore(ControlTask, "Control", 4096, NULL, 10, NULL, 1);
}

void loop() {
    // 1. รับข้อมูลจาก Python (Command)
    if (Serial.available() >= sizeof(CommandPacket) + 2) {
        if (Serial.read() == 0xAA && Serial.read() == 0x55) {
            CommandPacket cmd;
            Serial.readBytes((uint8_t*)&cmd, sizeof(CommandPacket));
            
            // Checksum Simple Validation
            uint8_t* ptr = (uint8_t*)&cmd;
            uint8_t sum = 0;
            for(int i=0; i<8; i++) sum += ptr[i]; 

            if (sum == cmd.checksum) {
                portENTER_CRITICAL(&gMux);
                target_v = cmd.v_target;
                target_w = cmd.omega_target;
                portEXIT_CRITICAL(&gMux);
            }
        }
    }

    // 2. ส่งข้อมูลกลับไป Python (Feedback) ทุก 20ms
    static unsigned long lastTX = 0;
    if (millis() - lastTX >= 5) {
        lastTX = millis();
        FeedbackPacket pkt;
        portENTER_CRITICAL(&gMux);
        pkt.v_measured = curr_v;
        pkt.omega_measured = curr_w;
        pkt.total_dist = total_d;
        portEXIT_CRITICAL(&gMux);

        uint8_t* ptr = (uint8_t*)&pkt.v_measured;
        pkt.checksum = 0;
        for(int i=0; i<12; i++) pkt.checksum += ptr[i];

        Serial.write((uint8_t*)&pkt, sizeof(FeedbackPacket));
    }
}