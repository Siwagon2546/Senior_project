#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// โครงสร้างสำหรับเก็บสถานะของ Encoder แต่ละตัว
typedef struct {
    int pinA;
    int pinB;
    volatile long count;
} Encoder_t;

/**
 * @brief เริ่มต้นตั้งค่า Encoder
 * @param enc Pointer ของโครงสร้าง Encoder
 * @param a ขา Phase A
 * @param b ขา Phase B
 */
void Encoder_Init(Encoder_t *enc, int a, int b);

/**
 * @brief อ่านค่า Ticks ปัจจุบัน
 */
long Encoder_GetCount(Encoder_t *enc);

/**
 * @brief อ่านค่า Ticks และ Reset เป็น 0 ทันที (Atomic)
 */
long Encoder_GetCountAndReset(Encoder_t *enc);

/**
 * @brief รีเซ็ตค่า Ticks เป็น 0
 */
void Encoder_Reset(Encoder_t *enc);

#ifdef __cplusplus
}
#endif

#endif