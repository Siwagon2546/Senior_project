#include "encoder.h"

// ฟังก์ชัน ISR (Interrupt Service Routine) ที่ทำงานใน RAM
void IRAM_ATTR encoder_isr(void* arg) {
    Encoder_t* enc = (Encoder_t*)arg;
    
    // ตรวจสอบทิศทางจากขา B
    if (digitalRead(enc->pinB)) {
        enc->count++;
    } else {
        enc->count--;
    }
}

void Encoder_Init(Encoder_t *enc, int a, int b) {
    enc->pinA = a;
    enc->pinB = b;
    enc->count = 0;

    // ตั้งค่า Mode ของ Pin (จัดการขา Input Only ของ ESP32)
    if (a < 34) pinMode(a, INPUT_PULLUP);
    else pinMode(a, INPUT);
    
    if (b < 34) pinMode(b, INPUT_PULLUP);
    else pinMode(b, INPUT);

    // เชื่อมต่อ Interrupt โดยส่ง pointer 'enc' เป็น Argument
    attachInterruptArg(digitalPinToInterrupt(enc->pinA), encoder_isr, (void*)enc, RISING);
}

long Encoder_GetCount(Encoder_t *enc) {
    return enc->count;
}

long Encoder_GetCountAndReset(Encoder_t *enc) {
    // ปิด Interrupt ชั่วคราวเพื่อให้อ่านค่าและ Reset ได้อย่างแม่นยำ (Atomic Operation)
    noInterrupts();
    long temp = enc->count;
    enc->count = 0;
    interrupts();
    return temp;
}

void Encoder_Reset(Encoder_t *enc) {
    enc->count = 0;
}