#include <Arduino.h>
#include <math.h>
// เอา esp_task_wdt.h ออกไปแล้ว
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"

BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

// 💡 หมายเหตุ: ถ้าสั่งแล้วล้อมีเสียงครางวี้ดๆ แต่ไม่หมุน ให้ลองเพิ่มเลข 200.0 เป็น 400.0 หรือ 500.0 ดูนะครับ (PWM อาจจะน้อยไปเอาชนะน้ำหนักหุ่นไม่ไหว)
PID PIDMotorL(-255, 255, 200.0, 0.0, 0.0);
PID PIDMotorR(-255, 255, 200.0, 0.0, 0.0);

const float WHEEL_PPR    = 16.0 * 99.5;
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH  = 0.37;
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

const unsigned long CMD_TIMEOUT_MS = 500;
const float MAX_V_MPS = 0.3f;            
const float MAX_OMEGA_RADPS = 1.2f;     

#pragma pack(push, 1)
struct CmdVelPayload {
    uint8_t header1;
    uint8_t header2;
    uint8_t type;
    float v;
    float omega;
    uint8_t checksum;
};

struct OdomPayload {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    uint8_t type = 0x12;
    float measured_v;
    float measured_omega;
    float total_dist;
    uint8_t checksum;
} odomData;
#pragma pack(pop)

portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

volatile float g_totalDist    = 0.0f;
volatile float g_vel_left     = 0.0f;
volatile float g_vel_right    = 0.0f;
volatile float target_v       = 0.0f;
volatile float target_omega   = 0.0f;
volatile unsigned long g_last_cmd_time = 0;

uint8_t calculateChecksum(uint8_t* ptr, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 2; i < length - 1; i++) sum ^= ptr[i];
    return sum;
}

void ControlLoopTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 
    
    unsigned long prevMicros = micros();
    float filtered_v_L = 0.0f;
    float filtered_v_R = 0.0f;
    const float LPF_ALPHA = 0.05f;
    float local_total_dist = 0.0f;

    while (1) {
        unsigned long currentMicros = micros();
        double dt = (currentMicros - prevMicros) / 1000000.0;
        if (dt <= 0.0) dt = 0.001;
        prevMicros = currentMicros;

        float tv, tom;
        unsigned long last_cmd;
        
        portENTER_CRITICAL(&gMux);
        tv  = target_v;
        tom = target_omega;
        last_cmd = g_last_cmd_time;
        portEXIT_CRITICAL(&gMux);

        if (millis() - last_cmd > CMD_TIMEOUT_MS) {
            tv = 0.0f;
            tom = 0.0f;
        }

        float left_target_ms  = tv - (tom * TRACK_WIDTH / 2.0f);
        float right_target_ms = tv + (tom * TRACK_WIDTH / 2.0f);

        long delta_L = Encoder_GetDelta(&encLeft);
        long delta_R = Encoder_GetDelta(&encRight);
        float stepL = delta_L * DIST_PER_TICK;
        float stepR = delta_R * DIST_PER_TICK;
        
        local_total_dist += (stepL + stepR) / 2.0f;

        float raw_v_L = stepL / dt;
        float raw_v_R = stepR / dt;
        filtered_v_L = (LPF_ALPHA * raw_v_L) + ((1.0f - LPF_ALPHA) * filtered_v_L);
        filtered_v_R = (LPF_ALPHA * raw_v_R) + ((1.0f - LPF_ALPHA) * filtered_v_R);

        portENTER_CRITICAL(&gMux);
        g_vel_left   = filtered_v_L;
        g_vel_right  = filtered_v_R;
        g_totalDist  = local_total_dist;
        portEXIT_CRITICAL(&gMux);

        int pwm_L = (int)PIDMotorL.compute(left_target_ms, filtered_v_L);
        int pwm_R = (int)PIDMotorR.compute(right_target_ms, filtered_v_R);

        if (abs(left_target_ms) < 0.001f && abs(right_target_ms) < 0.001f && abs(filtered_v_L) < 0.01f) {
            BTS7960_Stop(&motor_L);
            BTS7960_Stop(&motor_R);
        } else {
            BTS7960_SetSpeed(&motor_L, pwm_L);
            BTS7960_SetSpeed(&motor_R, pwm_R);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void SerialCommTask(void * pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz
    
    // 🌟 ขยาย Buffer ให้ใหญ่ขึ้น รองรับข้อมูลที่มาเป็นก้อน
    uint8_t rxBuffer[128];
    int rxIndex = 0;

    while (1) {
        // --- 📥 ส่วนรับข้อมูล (cmd_vel) แบบกวาดรวดเดียว ---
        while (Serial.available() > 0) {
            // ดูดข้อมูลทั้งหมดเข้า Buffer
            if (rxIndex < 128) {
                rxBuffer[rxIndex++] = Serial.read();
            } else {
                // Buffer ล้น (ไม่ควรเกิด) เคลียร์ทิ้งเลย
                rxIndex = 0;
            }
        }

        // --- 🔍 เริ่มค้นหาแพ็กเกจที่ซ่อนอยู่ใน Buffer ---
        if (rxIndex >= 12) { // 12 คือขนาดของ CmdVelPayload
            // ค้นหา Header [AA] [BB]
            for (int i = 0; i <= rxIndex - 12; i++) {
                if (rxBuffer[i] == 0xAA && rxBuffer[i+1] == 0xBB) {
                    uint8_t msg_type = rxBuffer[i+2];

                    if (msg_type == 0x11) {
                        // เจอแพ็กเกจคำสั่ง!
                        CmdVelPayload cmd;
                        memcpy(&cmd, &rxBuffer[i], sizeof(CmdVelPayload));
                        
                        // คำนวณ Checksum ตั้งแต่ไบต์ที่ 2 (Type) ถึง 10
                        uint8_t calc_cs = calculateChecksum((uint8_t*)&cmd, sizeof(CmdVelPayload));
                        
                        if (calc_cs == cmd.checksum) {
                            float rx_v = constrain(cmd.v, -MAX_V_MPS, MAX_V_MPS);
                            float rx_omega = constrain(cmd.omega, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS);

                            portENTER_CRITICAL(&gMux);
                            target_v = rx_v;
                            target_omega = rx_omega;
                            g_last_cmd_time = millis();
                            portEXIT_CRITICAL(&gMux);
                        }
                        
                        // เคลียร์ Buffer ทั้งหมดหลังจากเจอแพ็กเกจแล้ว
                        rxIndex = 0;
                        break; // ออกจากลูปค้นหา
                    }
                }
            }
            // ถ้าค้นจนสุดแล้วไม่เจออะไรเลย หรือข้อมูลเก่าเกินไป ก็ล้างไพ่ทิ้ง
            if (rxIndex >= 64) rxIndex = 0; 
        }

        // --- 📤 ส่วนส่งข้อมูล (Odometry) ---
        float vl, vr, td;
        portENTER_CRITICAL(&gMux);
        vl = g_vel_left;
        vr = g_vel_right;
        td = g_totalDist;
        portEXIT_CRITICAL(&gMux);

        odomData.measured_v = (vr + vl) / 2.0f;
        odomData.measured_omega = (vr - vl) / TRACK_WIDTH;
        odomData.total_dist = td;
        odomData.checksum = calculateChecksum((uint8_t*)&odomData, sizeof(OdomPayload));
        
        Serial.write((uint8_t*)&odomData, sizeof(OdomPayload));

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.setRxBufferSize(256);
    Serial.setTxBufferSize(256);
    Serial.begin(921600);
    delay(1000); // รอให้ Serial พร้อม

    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18);
    Encoder_Init(&encRight, 13, 14);

    g_last_cmd_time = millis(); 

    // เอา WDT ออกแล้ว รัน Task ลง Core เลย
    xTaskCreatePinnedToCore(ControlLoopTask, "Control", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(SerialCommTask, "Comm", 4096, NULL, 4, NULL, 0);
    
    vTaskDelete(NULL); 
}

void loop() {}