#pragma once
#include <stdint.h>
void esc_servo_init(void);               // sets 50 Hz on 14,15 and 16
void esc_write_pct(int esc_id, int pct); // 1..2 => 0..100 %
void gear_speed_pct(int pct);            // -100..100 (CR servo)    