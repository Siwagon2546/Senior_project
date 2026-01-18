#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

typedef struct {
    int pinA;
    int pinB;
    volatile long count;  // ตัวแปรสะสมค่าจริงจาก ISR (ไม่ถูก Reset ใน Loop)
    long prevCount;       // ตัวแปรจำค่าเก่า เพื่อใช้คำนวณผลต่าง (Delta)
} Encoder_t;

// ฟังก์ชันเริ่มต้นการทำงาน
void Encoder_Init(Encoder_t *enc, int pinA, int pinB);

// อ่านค่าตำแหน่งสะสมทั้งหมด (Absolute Position)
long Encoder_GetCount(Encoder_t *enc);

// อ่านค่าความเปลี่ยนแปลงเทียบกับรอบที่แล้ว (Velocity/Delta)
long Encoder_GetDelta(Encoder_t *enc);

// รีเซ็ตค่าทั้งหมดเป็น 0
void Encoder_Reset(Encoder_t *enc);

#endif