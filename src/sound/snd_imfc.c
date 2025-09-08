/*
 * 86Box    IMFC (skeleton) — Z80 sidecore + ROM loader + 6 MHz timer drive
 *
 * Minimal device that:
 *  - embeds libz80 (Z80Context),
 *  - runs it at an effective 6.000 MHz using 86Box timers,
 *  - loads a ROM at reset from env IMFC_ROM (fallback "imfc.bin"),
 *  - exposes a device named "imfc" with no mapped I/O yet (stubs provided).
 *
 * Wire your actual port map later by filling z80_io_read_cb/z80_io_write_cb
 * and adding io_sethandler() calls in imfc_init().
 *
 * Author: Clara’s build (2025)
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/sound.h>
#include <86box/plat_unused.h>
#include <86box/pic.h>
/* ====== Z80 core (libz80) ====== */
#include "z80.h"   
#include "imfc/ppi_v2.h" 
#include "imfc/piu.h"
#include "imfc/opm.h"  
#include "imfc/i8253.h"
/* --- knobs ---------------------------------------------------------- */
#ifndef IMFC_Z80_CLOCK_HZ
#  define IMFC_Z80_CLOCK_HZ   6000000u     /* 6.000 MHz effective */
#endif
#ifndef IMFC_Z80_TICK_US
#  define IMFC_Z80_TICK_US    100.0        /* run every 100 µs => 600 T-states/tick */
#endif
#ifndef IMFC_ROM_ENV
#  define IMFC_ROM_ENV        "IMFC_ROM"   /* env var for ROM path */
#endif
#ifndef IMFC_ROM_DEFAULT
#  define IMFC_ROM_DEFAULT    "imfc.bin"   /* fallback file if env not set */
#endif
/* ------------------------------------------------------------------- */
#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>

static void ensure_console(void)
{
    // Try to attach to the parent’s console (works if launched from cmd.exe/pwsh)
    BOOL attached = AttachConsole(ATTACH_PARENT_PROCESS);

    if (!attached) {
        // If there was no parent console (e.g., launched from Explorer), create one
        if (!AllocConsole())
            return;
    }

    // Route C stdio to the console
    FILE *fp;
    freopen_s(&fp, "CONOUT$", "w", stdout);
    freopen_s(&fp, "CONOUT$", "w", stderr);
    freopen_s(&fp, "CONIN$",  "r", stdin);

    // Optional: unbuffer so logs appear immediately
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // Optional: UTF-8 output (if you print UTF-8)
    SetConsoleOutputCP(CP_UTF8);

    // If you use iostreams, keep them in sync
    // std::ios::sync_with_stdio();
}
#endif
/* ========= device state ========= */
typedef struct imfc_s {
    /* Z80 core + timing */
	opm_t      fm_opp;
	piu_t 	   piu;
    Z80Context z80;
    pc_timer_t z80_timer;
    unsigned   z80_clock_hz;
    double     z80_tick_us;

    /* Simple flat 64 KiB memory */
    uint8_t    mem[0x10000];
    size_t     rom_len;          /* bytes of the read-only ROM window (0..64K) */
	uint8_t test_val;
	imfc_timer_t timer;
	void *priv;
    /* future: add USART/PIU/OPP, IRQ wiring, etc. */
} imfc_t;

/* ===== Z80 memory / I/O glue ===== */
static byte z80_mem_read_cb(size_t param, ushort addr)
{
    imfc_t *d = (imfc_t *)(uintptr_t)param;
    return d->mem[addr];
}

static void z80_mem_write_cb(size_t param, ushort addr, byte data)
{
    imfc_t *d = (imfc_t *)(uintptr_t)param;
    /* keep ROM window read-only */
    if (addr < d->rom_len) return;
    d->mem[addr] = data;
}

static byte z80_io_read_cb(size_t param, ushort port)
{
    imfc_t *d = (imfc_t *)(uintptr_t)param;
    byte val = 0xFF;

    switch (port & 0xFF) {
        case 0x00:
			OPM_Clock(&d->fm_opp, NULL, NULL, NULL, NULL);
			val = OPM_Read(&d->fm_opp, 0);
			break;
		case 0x01:
			OPM_Clock(&d->fm_opp, NULL, NULL, NULL, NULL);
            val = 0x00;
            break;
		case 0x10:
			val = 0xFE;
			break;
		case 0x11:
			val = d->test_val;
			//
			break;
		case 0x21:
		case 0x22:
			
			val = ppi_read(&d->piu.z80_ppi, port & 0x0F);
			//printf("[IMFC][Z80 I/O READ ] port=%02Xh -> %02Xh\n", port & 0xFF, val);
			break;
        default:
            break;
    }
	//
    /* debug log */
    

    return val;
}

static void z80_io_write_cb(size_t param, ushort port, byte data)
{
    imfc_t *d = (imfc_t *)(uintptr_t)param;

    switch (port & 0xFF) {
        case 0x00:
            OPM_WriteBuffered(&d->fm_opp, 0, data);
            break;
        case 0x01:
            OPM_WriteBuffered(&d->fm_opp, 1, data);
            break;
			
		case 0x20:
			ppi_write(&d->piu.z80_ppi, port & 0x0F, data);
			printf("[IMFC][Z80 I/O WRITE ] port=%02Xh -> %02Xh\n", port & 0xFF, data);
			break;
			
		case 0x23:
			ppi_write(&d->piu.z80_ppi, port & 0x0F, data);
			break;
        default:
            break;
    }

    /* debug log */
    //printf("[IMFC][Z80 I/O WRITE] port=%02Xh <- %02Xh\n", port & 0xFF, data);
}

/* ===== ROM loader ===== */
static size_t imfc_load_rom(imfc_t *d)
{
    const char *path = getenv(IMFC_ROM_ENV);
    if (!path || !*path) path = IMFC_ROM_DEFAULT;

    FILE *f = fopen(path, "rb");
    if (!f) {
        /* not fatal: run with blank RAM if ROM absent */
        pclog("IMFC: ROM not found (%s); running with empty RAM\n", path);
        return 0;
    }
    size_t n = fread(d->mem, 1, sizeof(d->mem), f);
    fclose(f);
    pclog("IMFC: ROM loaded (%zu bytes) from %s\n", n, path);
    return n;
}

/* ===== timer tick: execute Z80 T-states and reschedule ===== */
static void imfc_z80_tick(void *priv)
{
    imfc_t *d = (imfc_t *)priv;

    /* T-states = clock_hz * (tick_us / 1e6). e.g., 6 MHz * 100 µs = 600 T */
    unsigned tstates = (unsigned)((double)d->z80_clock_hz * (d->z80_tick_us * 1e-6));
    if (!tstates) tstates = 1;
	
    Z80ExecuteTStates(&d->z80, tstates);
	piu_clock_tick(&d->piu);
	if(OPM_ReadIRQ(&d->fm_opp, 0))
	{
		Z80INT(&d->z80, 0xFF);
	}
	imfc_timer_tick(&d->timer, tstates);

	if (imfc_timer_check_irq(&d->timer, 0)) {
		picint(5);
	}
	if (imfc_timer_check_irq(&d->timer, 1)) {
		picint(5);
	}

	if (imfc_timer_check_irq(&d->timer, 1)) {
		picint(5);
	}
    /* keep ticking in emu time */
    timer_on_auto(&d->z80_timer, d->z80_tick_us);
}
static void imfc_get_buffer(int32_t *buffer, int len, void *priv)
{
    imfc_t *d = (imfc_t *)priv;

    enum { CHUNK = 1024 };
    int produced = 0;

    /* temp L/R for a chunk */
    int32_t lbuf[CHUNK];
    int32_t rbuf[CHUNK];

    while (produced < len) {
        int take = (len - produced > CHUNK) ? CHUNK : (len - produced);
        int32_t *ptrs[2] = { lbuf, rbuf };

        /* writes samples into lbuf/rbuf (not additive) */
        OPM_GenerateStream(&d->fm_opp, ptrs, (uint32_t)take);  /* :contentReference[oaicite:4]{index=4} */

        /* add into 86Box mixer (interleaved LR LR …) */
        for (int i = 0; i < take; ++i) {
            buffer[0] += lbuf[i];
            buffer[1] += rbuf[i];
            buffer += 2;
        }
        produced += take;
    }
}
static uint8_t pc_read(uint16_t port, void *priv)
{
	imfc_t *d = (imfc_t *)priv;
	uint8_t port_int = port - 0x2A20;
	//printf("%u\n", port_int);
	uint8_t val = 0xFF;
	if (d->piu.pc_intr) {
		picint(5);
	}
	switch(port_int)
	{
		case 0x00:
			val = ppi_read(&d->piu.pc_ppi, 0);
			break;
		case 0x01:
			val = ppi_read(&d->piu.pc_ppi, 1);
			break;
		case 0x02:
			val = ppi_read(&d->piu.pc_ppi, 2);
			break;
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
			val = imfc_timer_read(&d->timer, port);
			break;

	}
	//printf("[IMFC][PC I/O READ ] port=%02Xh -> %02Xh\n", port_int & 0xFF, val);
	return val;
	
}
static void pc_write(uint16_t port, uint8_t data, void *priv)
{
	imfc_t *d = (imfc_t *)priv;
	uint8_t port_int = port - 0x2A20;
	//printf("[IMFC][PC I/O WRITE ] port=%02Xh -> %02Xh\n", port_int & 0xFF, data);
	if (d->piu.pc_intr) {
		picint(5);
	}
	switch(port_int)
	{
		case 0x01:
			d->test_val |= 0x01;
			ppi_write(&d->piu.pc_ppi, 1, data);
			break;
		case 0x03:
			
			ppi_write(&d->piu.pc_ppi, 3, data);	
			break;
		case 0x04:
		case 0x05:
		case 0x06:
		case 0x07:
			imfc_timer_write(&d->timer, port, data);
			break;
		case 0x08:
			if (!data & 0x10){
				d->test_val |= 0x80;
				
			} else {
				d->test_val &= ~0x80;
			}
			break;
	}
}
/* ===== bring up the Z80 side ===== */
static void imfc_z80_init(imfc_t *d)
{
    memset(d->mem, 0x00, sizeof(d->mem));
    d->rom_len = imfc_load_rom(d);
	ensure_console();
	piu_init(&d->piu);
	OPM_Reset(&d->fm_opp, 44100, 3900000);
    /* connect callbacks */
    d->z80.memRead  = z80_mem_read_cb;
    d->z80.memWrite = z80_mem_write_cb;
    d->z80.memParam = (size_t)(uintptr_t)d;

    d->z80.ioRead   = z80_io_read_cb;
    d->z80.ioWrite  = z80_io_write_cb;
    d->z80.ioParam  = (size_t)(uintptr_t)d;

    /* reset core (PC=0 unless your ROM expects otherwise) */
    Z80RESET(&d->z80);
    /* d->z80.PC = 0x0000; // set explicitly if needed */
	io_sethandler(0x2A20, 0x000F,
			  pc_read, NULL, NULL,
			  pc_write, NULL, NULL,
			  d);
    d->z80_clock_hz = IMFC_Z80_CLOCK_HZ;
    d->z80_tick_us  = IMFC_Z80_TICK_US;
	music_add_handler(imfc_get_buffer, d);
    /* schedule periodic execution */
	d->test_val = 0x01;
    timer_add(&d->z80_timer, imfc_z80_tick, d, 0);
    timer_on_auto(&d->z80_timer, d->z80_tick_us);
}


/* ========= 86Box device hooks ========= */

static void *imfc_init(UNUSED(const device_t *info))
{
    imfc_t *d = (imfc_t *)calloc(1, sizeof(*d));
    if (!d) return NULL;
	
    pclog("IMFC: init\n");

    /* Optional: map I/O ports here when you’re ready
       io_sethandler(BASE, SIZE,
                     /* inb  * /  NULL, NULL, NULL,
                     /* outb * /  NULL, NULL, NULL,
                     /* priv * /  d);
     */

    imfc_z80_init(d);
    return d;
}

static void imfc_reset(void *priv)
{
    imfc_t *d = (imfc_t *)priv;
    pclog("IMFC: reset\n");

    /* stop timer while we reset */
    timer_on_auto(&d->z80_timer, 0.0);

    /* re-load ROM each reset (optional; remove if you want one-time load) */
    memset(d->mem, 0x00, sizeof(d->mem));
    d->rom_len = imfc_load_rom(d);

    Z80RESET(&d->z80);
    /* d->z80.PC = 0x0000; */

    timer_on_auto(&d->z80_timer, d->z80_tick_us);
}

static void imfc_close(void *priv)
{
    imfc_t *d = (imfc_t *)priv;
    pclog("IMFC: close\n");

    /* stop Z80 timer */
    timer_on_auto(&d->z80_timer, 0.0);

    free(d);
}

/* expose as an ISA device named "imfc" */
const device_t imfc_device = {
    .name          = "IMFC (skeleton Z80)",
    .internal_name = "imfc",
    .flags         = DEVICE_ISA,   /* 8-bit ISA device */
    .local         = 0,
    .init          = imfc_init,
    .close         = imfc_close,
    .reset         = imfc_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
