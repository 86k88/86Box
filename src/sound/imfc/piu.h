#ifndef PIU_H
#define PIU_H

#include <stdint.h>
#include <stdbool.h>
#include "ppi_v2.h"

/* Which side raised/changed the interrupt */
typedef enum {
    PIU_SIDE_Z80 = 0,
    PIU_SIDE_PC  = 1
} piu_side_t;

/* Top-level PIU IRQ callback. Fired whenever the PPI's intr_changed fires. */
typedef void (*piu_irq_cb_t)(piu_side_t side, bool level, void *user);

typedef struct {
    // Z80 side
    ppi8255_t z80_ppi;

    // PC/ISA side
    ppi8255_t pc_ppi;

    // Shadow handshake lines from PPI callbacks
    bool z80_obf, z80_ibf, z80_intr;
    bool pc_obf,  pc_ibf,  pc_intr;

    // Pulses generated on clock tick
    bool z80_stb, z80_ack;
    bool pc_stb,  pc_ack;

    // Edge detection
    bool last_z80_obf, last_pc_obf;
    bool last_z80_ibf, last_pc_ibf;

    // ---- NEW: bubble-up interrupt callback ----
    piu_irq_cb_t irq_cb;
    void        *irq_user;
} piu_t;

void piu_init(piu_t *piu);
void piu_clock_tick(piu_t *piu);

/* Register (or clear) the top-level IRQ callback.
 * Pass cb=NULL to clear. 'user' is forwarded back to you unchanged. */
static inline void piu_set_irq_callback(piu_t *piu, piu_irq_cb_t cb, void *user)
{
    piu->irq_cb  = cb;
    piu->irq_user = user;
}

#endif // PIU_H
