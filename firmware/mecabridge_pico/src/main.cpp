#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "term.h"
#include <stdio.h>
#include "motors.h"
#include "esc_servo.h"
#include "enc.h"

int main()
{
  stdio_init_all();

  motors_init();
  // esc_servo_init();
  // enc_init();
  term_init();   // This will now wait for the first keypress

  while (1) {
    term_poll();
    sleep_ms(10);     // Small delay to prevent busy-waiting
  }
}
