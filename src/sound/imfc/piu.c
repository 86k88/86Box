#include "piu.h"
#include <string.h>
#include <stdio.h>


/* --- Internal callback shims --- */
static bool piu_mirror_guard = false;  /* prevent recursion while mirroring */
/* Mirror PC Port C bits 6/7 -> Z80 Port C bits 4/5 */
static void piu_mirror_pc_portc(piu_t *piu, uint8_t z80_val)
{
    uint8_t pc_val = ppi8255_portc_read(&piu->pc_ppi); // Port C = index 2
	uint8_t pc_val_old = pc_val;
    // clear bits 4/5
    pc_val &= ~((1 << 7));
	
    // copy PC bits 6/7 into Z80 bits 4/5
    //if (z80_val & (1 << 4)) pc_val |= (1 << 6);
    if (z80_val & (1 << 5)) pc_val |= (1 << 7);

    // write back into Z80 Port C
    piu->pc_ppi.port_latches[2] = pc_val;
	//printf("[PIU][Mirror] PC<-Z80 PortC: pcC=%02X  z80C(before)=%02X -> z80C(after)=%02X\n",
    //       z80_val, pc_val_old, pc_val);
}

static void piu_ibf_cb(int port, int level, void *user) {
    piu_t *piu = (piu_t *)user;
    if (port == 0) piu->z80_ibf = (level != 0);
    if (port == 1) piu->pc_ibf  = (level != 0);
	
	uint8_t pc_c = ppi8255_portc_read(&piu->z80_ppi);
	
    piu_mirror_pc_portc(piu, pc_c);
}
/* note: we deliberately keep the signature and behaviour identical,
 * but add mirroring of port data to the opposite side where requested.
 */
static void piu_obf_cb(int port, int level, uint8_t data, void *user) {
    (void)data; /* data can be inspected if you want edge-conditional routing */
    piu_t *piu = (piu_t *)user;
	/* Also mirror PC Port C bits 6/7 to Z80 Port C 4/5 */

    if (port == 0) {
        /* Z80 side OBF: mirror Z80 Port A -> PC Port A.
         * level==1 typically means edge asserted (data available). Mirror on the assert edge.
         */
        if (!level && !piu_mirror_guard) {
            piu_mirror_guard = true;
            /* copy the data into PC PPI Port A */
            piu->pc_ppi.in_port[0] = data;
			printf("[PIU][Mirror] Z80->PC PortA write value=0x%02X\n", data);

            piu_mirror_guard = false;
        }
        piu->z80_obf = !level;  // OBF is active low
    } else if (port == 1) {
        /* PC side OBF: mirror PC Port B -> Z80 Port B.
         * Mirror on the assert edge similarly.
         */
        if (!level && !piu_mirror_guard) {
            piu_mirror_guard = true;
            /* copy the data into Z80 PPI Port B */
            piu->z80_ppi.in_port[1] = data;
			printf("[PIU][Mirror] PC->Z80 PortB write value=0x%02X\n", data);
            piu_mirror_guard = false;
        }
        piu->pc_obf  = !level;
    }
	uint8_t pc_c = ppi8255_portc_read(&piu->z80_ppi);
    piu_mirror_pc_portc(piu, pc_c);
}


static void piu_intr_cb(int port, int level, void *user) {
    piu_t *piu = (piu_t *)user;
    bool lvl = level;

    if (port == 0) {
        piu->z80_intr = lvl;
        /* bubble up to the device-level callback */
        if (piu->irq_cb) piu->irq_cb(PIU_SIDE_Z80, lvl, piu->irq_user);
    } else if (port == 1) {
        piu->pc_intr  = lvl;
        if (piu->irq_cb) piu->irq_cb(PIU_SIDE_PC,  lvl, piu->irq_user);
    }
}

/* --- API --- */
void piu_init(piu_t *piu) {
    memset(piu, 0, sizeof(*piu));

    // Hook Z80 PPI
    ppi8255_callbacks_t z80_cb = {
        .ibf_changed  = piu_ibf_cb,
        .obf_changed  = piu_obf_cb,
        .intr_changed = piu_intr_cb,
    };
    ppi_init(&piu->z80_ppi, &z80_cb, piu);

    // Hook PC PPI
    ppi8255_callbacks_t pc_cb = {
        .ibf_changed  = piu_ibf_cb,
        .obf_changed  = piu_obf_cb,
        .intr_changed = piu_intr_cb,
    };
    ppi_init(&piu->pc_ppi, &pc_cb, piu);
}

void piu_clock_tick(piu_t *piu) {
    // OBF from one side drives STB on the other
    if (!piu->z80_obf && piu->last_z80_obf) {
        piu->pc_stb = true;
        ppi_strobe(&piu->pc_ppi, 0); // drive Port A strobe
    } else {
        piu->pc_stb = false;
    }

    if (piu->pc_obf && !piu->last_pc_obf) {
        piu->z80_stb = true;
        ppi_strobe(&piu->z80_ppi, 1);
    } else {
        piu->z80_stb = false;
    }

    // IBF from one side drives ACK on the other
    if (!piu->z80_ibf && piu->last_z80_ibf) {
        piu->pc_ack = true;
        check_intr(&piu->pc_ppi, 0, 3); // ACK for PC side
    } else {
        piu->pc_ack = false;
    }

    if (piu->pc_ibf && !piu->last_pc_ibf) {
        piu->z80_ack = true;
        check_intr(&piu->z80_ppi, 1, 3); // ACK for Z80 side
    } else {
        piu->z80_ack = false;
    }

    // Update edge detectors
    piu->last_z80_obf = piu->z80_obf;
    piu->last_pc_obf  = piu->pc_obf;
    piu->last_z80_ibf = piu->z80_ibf;
    piu->last_pc_ibf  = piu->pc_ibf;
}
