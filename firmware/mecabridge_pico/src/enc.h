#pragma once
#include <stdint.h>
void enc_init(void);           // attach IRQs A/B for 4 wheels
void enc_read(int32_t out[4]); // atomic copy
void enc_reset(void);
