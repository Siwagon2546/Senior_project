#include <Arduino.h>
#include <math.h>
#include <esp_task_wdt.h> // เพิ่มไลบรารี Hardware Watchdog
#include "bts7960.h"
#include "encoder.h"
#include "pid.h"
#include "driver/twai.h"

// ==========================================
// 1. CONFIGURATION & PINS & SAFETY LIMITS
// ==========================================
BTS7960_t motor_L = { 22, 23, 255 }; 
BTS7960_t motor_R = { 25, 26, 255 };
Encoder_t encLeft, encRight;

PID PIDMotorL(-255, 255, 200.0, 0.0, 0.0);
PID PIDMotorR(-255, 255, 200.0, 0.0, 0.0);

const float WHEEL_PPR    = 16.0 * 99.5;
const float WHEEL_RADIUS = 0.04;
const float TRACK_WIDTH  = 0.37;
const float DIST_PER_TICK = (2.0 * M_PI * WHEEL_RADIUS) / WHEEL_PPR;

// --- SAFETY CONFIGURATION ---
const int WDT_TIMEOUT_SECONDS = 2;       // Watchdog Timeout (วินาที)
const unsigned long CMD_TIMEOUT_MS = 500; // เวลาสูงสุดที่ยอมให้ขาดการเชื่อมต่อ (0.5 วินาที)
const float MAX_V_MPS = 0.3f;            // ความเร็วเส้นตรงสูงสุด (m/s) ปรับตามจริง
const float MAX_OMEGA_RADPS = 1.2f;     // ความเร็วหมุนสูงสุด (rad/s) ปรับตามจริง

// ==========================================
// 2. SHARED STATE (Core 0 ↔ Core 1)
// ==========================================
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

volatile float g_totalDist    = 0.0f;
volatile float g_vel_left     = 0.0f;
volatile float g_vel_right    = 0.0f;
volatile float target_v       = 0.0f;
volatile float target_omega   = 0.0f;

volatile unsigned long g_last_cmd_time = 0; // เก็บเวลาล่าสุดที่ได้รับคำสั่ง

String inputString = "";
bool stringComplete = false;

// ==========================================
// 3. CONTROL LOOP (Core 1 — 1000 Hz)
// ==========================================
void ControlLoopTask(void * pvParameters) {
    // ลงทะเบียน Task นี้เข้าสู่ระบบ Watchdog
    esp_task_wdt_add(NULL); 

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    unsigned long prevMicros = micros();
    float filtered_v_L = 0.0f;
    float filtered_v_R = 0.0f;
    const float LPF_ALPHA = 0.05f;
    float local_total_dist = 0.0f;

    while (1) {
        // รายงานตัวกับ Watchdog ว่า Task นี้ยังทำงานปกติ
        esp_task_wdt_reset(); 

        unsigned long currentMicros = micros();
        double dt = (currentMicros - prevMicros) / 1000000.0;
        if (dt <= 0.0) dt = 0.001;
        prevMicros = currentMicros;

        float tv, tom;
        unsigned long last_cmd;
        
        portENTER_CRITICAL(&gMux);
        tv  = target_v;
        tom = target_omega;
        last_cmd = g_last_cmd_time; // ดึงเวลาล่าสุดมาเช็ค
        portEXIT_CRITICAL(&gMux);

        // --- SAFETY: DEADMAN SWITCH ---
        // ถ้าไม่ได้รับคำสั่งใหม่เกิน CMD_TIMEOUT_MS ให้บังคับหยุดรถ
        if (millis() - last_cmd > CMD_TIMEOUT_MS) {
            tv = 0.0f;
            tom = 0.0f;
        }

        // Inverse Kinematics
        float left_target_ms  = tv - (tom * TRACK_WIDTH / 2.0f);
        float right_target_ms = tv + (tom * TRACK_WIDTH / 2.0f);

        // Forward Kinematics
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

        // PID Control
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

// ==========================================
// 4. SETUP & LOOP (Core 0)
// ==========================================
void setup() {
    Serial.begin(921600);
    inputString.reserve(50);
    
    // --- ตั้งค่า Hardware Watchdog ---
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true); // true = ถ้านับถอยหลังหมด ให้ Restart บอร์ดทันที
    esp_task_wdt_add(NULL); // เอา Core 0 (Main Loop) เข้าระบบ Watchdog

    BTS7960_Init(&motor_L);
    BTS7960_Init(&motor_R);
    Encoder_Init(&encLeft, 19, 18);
    Encoder_Init(&encRight, 13, 14);

    // เซ็ตเวลาเริ่มต้นให้ Deadman switch ป้องกันการวิ่งตอนเพิ่งเปิดเครื่อง
    g_last_cmd_time = millis(); 

    xTaskCreatePinnedToCore(ControlLoopTask, "Control", 8192, NULL, 10, NULL, 1);
}

void loop() {
    // รายงานตัวกับ Watchdog ว่า Main Loop ยังไม่แฮงค์
    esp_task_wdt_reset(); 

    // --- 1. รับข้อมูลจาก Serial ---
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else if (inChar != '\r') { 
            inputString += inChar;
        }
    }

    // --- 2. ประมวลผล ---
    if (stringComplete) {
        inputString.trim();
        
        if (inputString == "WHO_ARE_YOU") {
            Serial.println("I_AM_DRIVE_NODE");
        } 
        else {
            int commaIndex = inputString.indexOf(',');
            if (commaIndex > 0) {
                float rx_v = inputString.substring(0, commaIndex).toFloat();
                float rx_omega = inputString.substring(commaIndex + 1).toFloat();

                // --- SAFETY: CLAMPING จำกัดความเร็วสูงสุด ---
                rx_v = constrain(rx_v, -MAX_V_MPS, MAX_V_MPS);
                rx_omega = constrain(rx_omega, -MAX_OMEGA_RADPS, MAX_OMEGA_RADPS);

                portENTER_CRITICAL(&gMux);
                target_v = rx_v;
                target_omega = rx_omega;
                g_last_cmd_time = millis(); // อัปเดตเวลาล่าสุดว่าเพิ่งได้รับคำสั่ง
                portEXIT_CRITICAL(&gMux);
            }
        }
        
        inputString = "";
        stringComplete = false;
    }

    // --- 3. ส่งข้อมูลกลับไปยัง Raspberry Pi (50Hz) ---
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish >= 20) {
        lastPublish = millis();
        
        float vl, vr, td;
        portENTER_CRITICAL(&gMux);
        vl = g_vel_left;
        vr = g_vel_right;
        td = g_totalDist;
        portEXIT_CRITICAL(&gMux);

        float measured_v = (vr + vl) / 2.0f;
        float measured_omega = (vr - vl) / TRACK_WIDTH;

        Serial.printf("%.3f,%.3f,%.3f\n", measured_v, measured_omega, td);
    }
}