#include <math.h>
#include "bts7960.h"
#include "pid.h"
#include "encoder.h"
#include <ps5Controller.h>

Encoder_t encLeft;
Encoder_t encRight;

const int N_WHEEL = 2;
const int encA[N_WHEEL] = {13, 19};
const int encB[N_WHEEL] = {14, 18};

BTS7960_t Motor_L = {
    .RPWM_Pin = 25,  // ขา D5 (PWM)
    .LPWM_Pin = 26,  // ขา D6 (PWM)
    .max_pwm = 255  // สำหรับ Arduino 8-bit PWM
};
BTS7960_t Motor_R = {
    .RPWM_Pin = 22,  // ขา D5 (PWM)
    .LPWM_Pin = 23,  // ขา D6 (PWM)
    .max_pwm = 255  // สำหรับ Arduino 8-bit PWM
};

// สร้าง Object Array สำหรับ Encoder


uint16_t TPR = 600;
double wheelRPM[2] = {0,0};

// void computeWheelKinematics(float dt) {
//     float sum_v = 0.0f;
//     for (int i = 0; i < N_WHEEL; ++i) {
//         // ดึงค่า dcnt ออกมาและ reset ในตัวเดียว เพื่อลดความคลาดเคลื่อนทางเวลา
//         // long dcnt = encoders[i].getCountAndReset();
        
//         // wheelRPM[i]   = (dcnt / TPR) * (60.0f / dt);
//         // Serial.printf("RPM1: %.2f , RPM2: %d \n", wheelRPM[0], wheelRPM[1]);
//         Serial.printf("dcnt: %.2f , i: %d \n", dcnt, i);
//         // wheelOmega[i] = wheelRPM[i] * (2*PI / 60.0f);
//         // wheelV[i]     = wheelOmega[i] * R;
        
//     }
// }

// timer 
unsigned long previousMillis = 0;
const long interval = 10;

//Pid parameter ssss
float Setpoint[3]={0,0};
float Input[3]={0,0};
float Output[3]={0,0};

double   Kp1=100, Ki1=0.001, Kd1=0.01;
double   Kp2=100, Ki2=0.001, Kd2=0.01;

PID PIDMortorL(-255, 255, Kp1, Ki1, Kd1);
PID PIDMortorR(-255, 255, Kp2, Ki2, Kd2);

float TRACK_WIDTH = 0.37;
void Drive_equation(int target_v ,int target_w,float &out1, float &out2){
    if (abs(target_v) < 10) target_v = 0;
    if (abs(target_w) < 10) target_w = 0;
    // if (abs(W) < 10) W = 0;
    // if (abs(x) < 100) x = x/2;
    // if (abs(y) < 100) y = y/2;
    // if (abs(W) < 100) W = W/2;
    target_v = map(target_v,-127,127,-2,2);
    target_w = map(target_w,-127,127,-2,2);
    out1 = target_v - (target_w * TRACK_WIDTH / 2.0f);
    out2 = target_v + (target_w * TRACK_WIDTH / 2.0f);
}

// float target_v = 0, target_w = 0; // ตัวแปรรับค่าจาก Serial
unsigned long lastSerialTime = 0;

void setup() {
    Serial.begin(115200);
    // ps5.begin("10:18:49:ac:28:82");
    ps5.begin("10:18:49:ac:28:82");
    
    Encoder_Init(&encLeft, encA[0], encB[0]);  // ขา A=12, B=14
    Encoder_Init(&encRight, encA[1], encB[1]); // ขา A=26, B=27
    Serial.println("Encoders Initialized");

    while (ps5.isConnected() == false)  
    { 
    Serial.println("PS5 controller not found");
    delay(300);
    }
}

void loop(){

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
    // save the current time as the last time the action happened
    previousMillis = currentMillis;
    // --- 1. การอ่าน Serial Command ---
    // if (Serial.available() > 0) {
    //     // รับรูปแบบ: "v,w\n" เช่น "0.5,0.2\n"
    //     target_v = Serial.parseFloat(); 
    //     if (Serial.read() == ',') {
    //         target_w = Serial.parseFloat();
    //     }
    //     Serial.printf("target_v: %ld, target_w: %ld\n", target_v, target_w);
    //     while (Serial.available()) Serial.read(); // Clear buffer
    //     lastSerialTime = currentMillis;
    // }
    // Safety: ถ้าไม่มีคำสั่งส่งมาเกิน 1 วินาที ให้หยุดหุ่นยนต์
    if (currentMillis - lastSerialTime > 1000) {
        // target_v = 0; target_w = 0;
    }

    long countL = Encoder_GetCountAndReset(&encLeft);
    long countR = Encoder_GetCountAndReset(&encRight);
    Drive_equation(ps5.LStickY(),ps5.LStickX(),Setpoint[0],Setpoint[1]);
    Output[0] = PIDMortorL.compute(Setpoint[0],float(countL));
    Output[1] = PIDMortorR.compute(Setpoint[1],float(countR));
    
    //  (target_v, target_w, Setpoint[0], Setpoint[1]);
    // BTS7960_SetSpeed(&Motor_L,int(Setpoint[0]));
    // BTS7960_SetSpeed(&Motor_R,int(Setpoint[1]));
    BTS7960_SetSpeed(&Motor_L,int(Output[0]));
    BTS7960_SetSpeed(&Motor_R,int(Output[1]));
    // computeWheelKinematics(interval);
    Serial.printf("encoder L: %ld, R: %ld\n", countL, countR);

  }
  if (Serial.availableForWrite() > 20){
      Serial.print("rX: "); Serial.print(int(ps5.LStickX())); Serial.print("rY: ");Serial.println(int(ps5.LStickY()));
      Serial.print("Setpoint[0]: "); Serial.print(Setpoint[0],2);Serial.print("Setpoint[1]: "); Serial.println(Setpoint[1],2);
      Serial.print("Output[0]: "); Serial.print(Output[0],2);Serial.print("Output[1]: "); Serial.println(Output[1],2);
  }
//   Serial.printf("RPM1: %.2f , RPM2: %d \n", wheelRPM[0], wheelRPM[1]);
}