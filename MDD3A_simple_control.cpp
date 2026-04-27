#include <Arduino.h>

// ── 1. ใส่ PIN ของคุณตรงนี้ ─────────────────────────
const int PWM_LEFT_A  = 5;   // ขาล้อซ้าย
const int PWM_LEFT_B  = 4;

void setup() {
  pinMode(PWM_LEFT_A, OUTPUT); pinMode(PWM_LEFT_B, OUTPUT);
}

void loop() {
  // --- 1. เดินหน้า (Forward) ---
  analogWrite(PWM_LEFT_A, 100);    // ความเร็ว 100 (0-255)
  analogWrite(PWM_LEFT_B, 0);
  delay(2000);                   // หมุน 2 วินาที

  // --- 2. หยุด (Stop) ---
  analogWrite(PWM_LEFT_A, 0);
  analogWrite(PWM_LEFT_B, 0);
  delay(1000);                   // หยุด 1 วินาที

  // --- 3. ถอยหลัง (Backward) ---
  analogWrite(PWM_LEFT_B, 100);
  analogWrite(PWM_LEFT_A, 0);
  delay(2000);                   // หมุน 2 วินาที

  // --- 4. หยุด (Stop) ---
  analogWrite(PWM_LEFT_B, 0);
  analogWrite(PWM_LEFT_A, 0);
  delay(1000);
}
