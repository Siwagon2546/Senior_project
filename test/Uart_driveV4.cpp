#include <Arduino.h>
#include <math.h>
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"

// --- Configuration ---
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

PID PIDMotorL(-255, 255, 200.0, 0.000, 0.00);
PID PIDMotorR(-255, 255, 200.0, 0.000, 0.00);

const float WHEEL_RADIUS = 0.055;
const float TRACK_WIDTH  = 0.37;
const float WHEEL_PPR    = 16.0 * 99.5 * 4.0; // 4X Resolution
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

// --- Communication Structures ---
#pragma pack(push, 1)
struct FeedbackPacket {
    uint8_t h1 = 0xAA;
    uint8_t h2 = 0x55;
    float v_measured;      
    float omega_measured;  
    int32_t left_ticks;    
    int32_t right_ticks;   
    uint8_t checksum;      
};

struct CommandPacket {
    float v_target;
    float omega_target;
    uint8_t checksum;
};
#pragma pack(pop)

// --- Shared Variables (ใช้แลกเปลี่ยนระหว่าง Core) ---
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
volatile float target_v_shared = 0, target_w_shared = 0;
volatile float curr_v_shared = 0, curr_w_shared = 0;
volatile int32_t left_ticks_shared = 0, right_ticks_shared = 0;

// --- Task: Control Loop (Core 1) ---
void ControlTask(void * pv) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float fL = 0, fR = 0;
    int32_t last_L = 0, last_R = 0; 

    while(1) {
        // 1. ดึงค่า Target จาก Shared Variables
        portENTER_CRITICAL(&gMux);
        float tv = target_v_shared; 
        float tw = target_w_shared;
        portEXIT_CRITICAL(&gMux);

        // 2. อัปเดต Encoder (อ่านที่นี่ที่เดียวเท่านั้น!)
        int32_t cur_L = Encoder_GetCount(&encLeft);
        int32_t cur_R = Encoder_GetCount(&encRight);
        
        float dt = 0.005; // 200Hz
        float sL = (cur_L - last_L) * DIST_PER_TICK;
        float sR = (cur_R - last_R) * DIST_PER_TICK;
        last_L = cur_L; last_R = cur_R;

        if (abs(tv) < 0.005f && abs(tw) < 0.005f) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
            fL = 0; fR = 0;
        } else {
            float tL = tv - (tw * TRACK_WIDTH / 2.0f);
            float tR = tv + (tw * TRACK_WIDTH / 2.0f);
            
            fL = (0.2f * (sL/dt)) + 0.8f * fL;
            fR = (0.2f * (sR/dt)) + 0.8f * fR;

            BTS7960_SetSpeed(&motor_L, (int)PIDMotorL.compute(tL, fL));
            BTS7960_SetSpeed(&motor_R, (int)PIDMotorR.compute(tR, fR));
        }

        // 3. อัปเดตค่าสถานะลง Shared Variables เพื่อให้ loop() เอาไปส่ง
        portENTER_CRITICAL(&gMux);
        curr_v_shared = (fR + fL) / 2.0f;
        curr_w_shared = (fR - fL) / TRACK_WIDTH;
        left_ticks_shared = cur_L;
        right_ticks_shared = cur_R;
        portEXIT_CRITICAL(&gMux);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);
    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    // สำคัญ: ต้องระบุ PCNT Unit ให้ต่างกัน
    Encoder_Init(&encLeft, 19, 18, PCNT_UNIT_0);
    Encoder_Init(&encRight, 13, 14, PCNT_UNIT_1);
    
    // สร้าง Task ไว้ที่ Core 1
    xTaskCreatePinnedToCore(ControlTask, "Control", 4096, NULL, 10, NULL, 1);
}

void loop() {
    // 1. รับข้อมูลจาก Python (Command)
    if (Serial.available() >= sizeof(CommandPacket) + 2) {
        if (Serial.read() == 0xAA && Serial.read() == 0x55) {
            CommandPacket cmd;
            Serial.readBytes((uint8_t*)&cmd, sizeof(CommandPacket));
            
            uint8_t sum = 0;
            uint8_t* ptr = (uint8_t*)&cmd;
            for(int i=0; i<8; i++) sum += ptr[i]; 

            if (sum == cmd.checksum) {
                portENTER_CRITICAL(&gMux);
                target_v_shared = cmd.v_target;
                target_w_shared = cmd.omega_target;
                portEXIT_CRITICAL(&gMux);
            }
        }
    }

    // 2. ส่งข้อมูลกลับไป Python (Feedback) ทุก 20ms
    static unsigned long lastTX = 0;
    if (millis() - lastTX >= 20) { 
        lastTX = millis();
        FeedbackPacket pkt;
        
        // ดึงค่าจาก Shared Variables มาใส่ Packet (ใช้เวลาสั้นมาก ไม่ขวาง WDT)
        portENTER_CRITICAL(&gMux);
        pkt.v_measured = curr_v_shared;
        pkt.omega_measured = curr_w_shared;
        pkt.left_ticks = left_ticks_shared;
        pkt.right_ticks = right_ticks_shared;
        portEXIT_CRITICAL(&gMux);

        // คำนวณ Checksum ของ 16 bytes (v + w + l_ticks + r_ticks)
        uint8_t* data_ptr = (uint8_t*)&pkt.v_measured;
        pkt.checksum = 0;
        for(int i = 0; i < 16; i++) { 
            pkt.checksum += data_ptr[i];
        }

        Serial.write((uint8_t*)&pkt, sizeof(FeedbackPacket));
    }
}