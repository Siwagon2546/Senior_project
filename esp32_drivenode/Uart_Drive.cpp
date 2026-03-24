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

PID PIDMotorL(-255, 255, 200.0, 0.01, 10);
PID PIDMotorR(-255, 255, 200.0, 0.01, 10);

const float WHEEL_RADIUS  = 0.05;
const float TRACK_WIDTH   = 0.37;
const float WHEEL_PPR     = 16.0 * 99.5;
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

// ==========================================
// 2. COMMUNICATION STRUCTURES
// ==========================================
#pragma pack(push, 1)
struct FeedbackPacket {
    uint8_t  h1 = 0xAA;
    uint8_t  h2 = 0x55;
    float    v_measured;      // 4 bytes
    float    omega_measured;  // 4 bytes
    int32_t  left_ticks;      // 4 bytes
    int32_t  right_ticks;     // 4 bytes
    uint8_t  checksum;        // 1 byte
    // รวม: 2+4+4+4+4+1 = 19 bytes
};

struct CommandPacket {
    float   v_target;      // 4 bytes
    float   omega_target;  // 4 bytes
    uint8_t checksum;      // 1 byte
    // รวม payload (ไม่นับ checksum): 8 bytes
};
#pragma pack(pop)

// ==========================================
// 3. SHARED STATE
// ==========================================
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

volatile float    target_v          = 0.0f;
volatile float    target_w          = 0.0f;
volatile float    curr_v            = 0.0f;
volatile float    curr_w            = 0.0f;
volatile int32_t  share_left_ticks  = 0;
volatile int32_t  share_right_ticks = 0;

// ✅ แก้ไข: เพิ่ม timestamp สำหรับ software watchdog
volatile unsigned long lastCmdMillis = 0;
const unsigned long CMD_TIMEOUT_MS   = 300;  // หยุดมอเตอร์ถ้าไม่มี cmd > 500ms

// ==========================================
// 4. CONTROL LOOP (Core 1 — 200 Hz)
// ==========================================
void ControlTask(void *pv) {
    TickType_t    xLastWakeTime = xTaskGetTickCount();
    unsigned long prevMicros    = micros();
    float         fL = 0.0f, fR = 0.0f;

    while (1) {
        unsigned long cur = micros();
        float dt = (cur - prevMicros) / 1000000.0f;
        if (dt <= 0.0f) dt = 0.005f;
        prevMicros = cur;

        // อ่าน target velocity
        float tv, tw;
        portENTER_CRITICAL(&gMux);
        tv = target_v;
        tw = target_w;
        portEXIT_CRITICAL(&gMux);

        // ✅ แก้ไข: Software watchdog — บังคับหยุดถ้าไม่มี cmd ใหม่
        if (millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
            tv = 0.0f;
            tw = 0.0f;
            portENTER_CRITICAL(&gMux);
            target_v = 0.0f;
            target_w = 0.0f;
            portEXIT_CRITICAL(&gMux);
        }

        // อ่าน encoder
        int32_t absoluteL = (int32_t)Encoder_GetCount(&encLeft);
        int32_t absoluteR = (int32_t)Encoder_GetCount(&encRight);

        static int32_t prev_L = 0, prev_R = 0;
        long dL = absoluteL - prev_L;
        long dR = absoluteR - prev_R;
        prev_L = absoluteL;
        prev_R = absoluteR;

        float sL = dL * DIST_PER_TICK;
        float sR = dR * DIST_PER_TICK;

        // Low-pass filter
        fL = (0.2f * (sL / dt)) + 0.8f * fL;
        fR = (0.2f * (sR / dt)) + 0.8f * fR;

        if (fabsf(tv) < 0.001f && fabsf(tw) < 0.001f) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
            fL = 0.0f;
            fR = 0.0f;
        } else {
            float tL = tv - (tw * TRACK_WIDTH / 2.0f);
            float tR = tv + (tw * TRACK_WIDTH / 2.0f);
            BTS7960_SetSpeed(&motor_L, (int)PIDMotorL.compute(tL, fL));
            BTS7960_SetSpeed(&motor_R, (int)PIDMotorR.compute(tR, fR));
        }

        // อัปเดตสถานะให้ Core 0
        portENTER_CRITICAL(&gMux);
        curr_v            = (fR + fL) / 2.0f;
        curr_w            = (fR - fL) / TRACK_WIDTH;
        share_left_ticks  = absoluteL;
        share_right_ticks = absoluteR;
        portEXIT_CRITICAL(&gMux);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

// ==========================================
// 5. SETUP & LOOP (Core 0 — Communication)
// ==========================================
void setup() {
    Serial.begin(115200);
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft,  19, 18);
    Encoder_Init(&encRight, 13, 14);

    xTaskCreatePinnedToCore(ControlTask, "Control", 4096, NULL, 10, NULL, 1);
}

void loop() {
    // ---- รับ CommandPacket จาก Python ----
    // CommandPacket = 9 bytes (v:4 + w:4 + checksum:1)
    // Header = AA 55 (2 bytes) → รวม 11 bytes ต่อ frame
    if (Serial.available() >= (int)(sizeof(CommandPacket) + 2)) {
        if (Serial.read() == 0xAA) {
            if (Serial.read() == 0x55) {
                CommandPacket cmd;
                Serial.readBytes((uint8_t*)&cmd, sizeof(CommandPacket));

                // ✅ แก้ไข: คำนวณ checksum เฉพาะ payload bytes เท่านั้น
                //    (8 bytes = v_target + omega_target, ไม่รวม checksum field)
                uint8_t  sum = 0;
                uint8_t* ptr = (uint8_t*)&cmd;
                for (int i = 0; i < (int)(sizeof(CommandPacket) - 1); i++) {
                    sum += ptr[i];
                }

                if (sum == cmd.checksum) {
                    portENTER_CRITICAL(&gMux);
                    target_v = cmd.v_target;
                    target_w = cmd.omega_target;
                    portEXIT_CRITICAL(&gMux);

                    // ✅ แก้ไข: อัปเดต watchdog timestamp
                    lastCmdMillis = millis();
                }
                // ถ้า checksum ไม่ตรง ก็แค่ ignore packet นี้
            }
        }
    }

    // ---- ส่ง FeedbackPacket กลับ Python ที่ 50 Hz ----
    static unsigned long lastTX = 0;
    if (millis() - lastTX >= 10) {
        lastTX = millis();

        FeedbackPacket pkt;
        portENTER_CRITICAL(&gMux);
        pkt.v_measured     = curr_v;
        pkt.omega_measured = curr_w;
        pkt.left_ticks     = share_left_ticks;
        pkt.right_ticks    = share_right_ticks;
        portEXIT_CRITICAL(&gMux);

        // checksum ครอบคลุม 16 bytes: v(4)+w(4)+L(4)+R(4)
        uint8_t* ptr = (uint8_t*)&pkt.v_measured;
        pkt.checksum = 0;
        for (int i = 0; i < 16; i++) pkt.checksum += ptr[i];

        Serial.write((uint8_t*)&pkt, sizeof(FeedbackPacket));
    }
}