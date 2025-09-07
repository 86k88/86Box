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
#include <86box/plat_unused.h>

/* ====== Z80 core (libz80) ====== */
#include "z80.h"   /* your provided libz80 headers */

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

/* ========= device state ========= */
typedef struct imfc_s {
    /* Z80 core + timing */
    Z80Context z80;
    pc_timer_t z80_timer;
    unsigned   z80_clock_hz;
    double     z80_tick_us;

    /* Simple flat 64 KiB memory */
    uint8_t    mem[0x10000];
    size_t     rom_len;          /* bytes of the read-only ROM window (0..64K) */

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
    (void)param;
    (void)port;
    /* TODO: wire to real IMFC ports (PIU/USART/OPM/etc.) */
    return 0xFF;
}

static void z80_io_write_cb(size_t param, ushort port, byte data)
{
    (void)param;
    (void)port;
    (void)data;
    /* TODO: wire to real IMFC ports */
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

    /* keep ticking in emu time */
    timer_on_auto(&d->z80_timer, d->z80_tick_us);
}

/* ===== bring up the Z80 side ===== */
static void imfc_z80_init(imfc_t *d)
{
    memset(d->mem, 0x00, sizeof(d->mem));
    d->rom_len = imfc_load_rom(d);

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

    d->z80_clock_hz = IMFC_Z80_CLOCK_HZ;
    d->z80_tick_us  = IMFC_Z80_TICK_US;

    /* schedule periodic execution */
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
