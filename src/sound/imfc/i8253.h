#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t reload;     // programmed divisor
    uint16_t counter;    // current counter
    uint8_t  mode;       // mode 2 (rate) / mode 3 (square)
    bool     out;        // output pin
    bool     irq;        // IRQ flag
    double   accum;      // fractional PIT clocks
} timer_chan_t;

typedef struct {
    timer_chan_t ch[3];  // 0=A, 1=B, 2=prescaler for B
    uint8_t tcwr;        // last control word written
    double  clk_ratio;   // PIT input clock / Z80 clock
} imfc_timer_t;

void imfc_timer_init(imfc_timer_t *t, double pit_clk_hz, double cpu_clk_hz);
void imfc_timer_write(imfc_timer_t *t, uint16_t port, uint8_t data);
uint8_t imfc_timer_read(imfc_timer_t *t, uint16_t port);
void imfc_timer_tick(imfc_timer_t *t, int tstates);

/* Helper: check and clear IRQ flags */
bool imfc_timer_check_irq(imfc_timer_t *t, int chan);
