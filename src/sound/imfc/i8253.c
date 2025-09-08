#include "i8253.h"
#include <string.h>
#include <stdio.h>

static void timer_reload(timer_chan_t *ch, uint16_t val) {
    if (val == 0) val = 0x10000; // 8253 interprets 0 as 65536
    ch->reload = val;
    ch->counter = val;
    ch->irq = false;
}

void imfc_timer_init(imfc_timer_t *t, double pit_clk_hz, double cpu_clk_hz) {
    memset(t, 0, sizeof(*t));
    t->clk_ratio = pit_clk_hz / cpu_clk_hz;
    for (int i = 0; i < 3; i++) {
        timer_reload(&t->ch[i], 0x10000);
        t->ch[i].mode = 2; // default to rate generator
    }
}

void imfc_timer_write(imfc_timer_t *t, uint16_t port, uint8_t data) {
    switch (port & 0x07) {
        case 0x04: // CNTR0
            timer_reload(&t->ch[0], data);
            break;
        case 0x05: // CNTR1
            timer_reload(&t->ch[1], data);
            break;
        case 0x06: // CNTR2 (prescaler for B)
            timer_reload(&t->ch[2], data);
            break;
        case 0x07: // TCWR
            t->tcwr = data;
            /* Very simplified decode: bits 1..0 = mode */
            for (int i = 0; i < 3; i++) {
                t->ch[i].mode = (data >> (i*2)) & 0x03;
            }
            break;
        default:
            break;
    }
}

uint8_t imfc_timer_read(imfc_timer_t *t, uint16_t port) {
    switch (port & 0x07) {
        case 0x04: return (uint8_t)(t->ch[0].counter & 0xFF);
        case 0x05: return (uint8_t)(t->ch[1].counter & 0xFF);
        case 0x06: return (uint8_t)(t->ch[2].counter & 0xFF);
        case 0x07: return t->tcwr;
        default:   return 0xFF;
    }
}

void imfc_timer_tick(imfc_timer_t *t, int tstates) {
    double pit_ticks = t->clk_ratio * tstates;

    for (int i = 0; i < 3; i++) {
        timer_chan_t *ch = &t->ch[i];
        ch->accum += pit_ticks;
        while (ch->accum >= 1.0) {
            ch->accum -= 1.0;

            if (--ch->counter == 0) {
                ch->irq = true;
                if (ch->mode == 3) {
                    ch->out = !ch->out; // square wave toggle
                } else {
                    ch->out = true; // pulse high
                }
                ch->counter = ch->reload; // auto reload
            }
        }
    }

    /* special case: CNTR2 is prescaler for CNTR1 */
    if (t->ch[2].irq) {
        t->ch[2].irq = false;
        if (--t->ch[1].counter == 0) {
            t->ch[1].irq = true;
            t->ch[1].counter = t->ch[1].reload;
        }
    }
}

bool imfc_timer_check_irq(imfc_timer_t *t, int chan) {
    if (t->ch[chan].irq) {
        t->ch[chan].irq = false;
        return true;
    }
    return false;
}
